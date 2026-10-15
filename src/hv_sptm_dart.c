/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

static const u32 sptm_dart_shifts[] = {36, 25, SPTM_PAGE_SHIFT};

struct sptm_dart *sptm_find_dart(u32 id)
{
    for (size_t index = 0; index < sptm.dart_count; index++) {
        if (sptm.darts[index].valid && sptm.darts[index].id == id)
            return &sptm.darts[index];
    }

    return NULL;
}

static bool sptm_dart_tlb_op(struct sptm_dart *dart, u32 op, u64 start, u64 end)
{
    // TLB registers are inaccessible while the DART is powered down.
    if (!dart->powered)
        return true;

    sysop("dsb sy");
    for (size_t index = 0; index < dart->instance_count; index++) {
        u64 base = dart->instances[index];
        if (op & DART_TLB_FLUSH_DVA) {
            write32(base + DART_TLB_START_DVA, start >> SPTM_PAGE_SHIFT);
            write32(base + DART_TLB_END_DVA, (end - 1) >> SPTM_PAGE_SHIFT);
        }
        write32(base + DART_TLB_OP, op);
        size_t retry = 0;
        while (read32(base + DART_TLB_OP) & DART_TLB_BUSY) {
            if (++retry == 10000) {
                printf("HV: SPTM DART %u TLB operation 0x%x timed out at 0x%lx\n", dart->id, op,
                       base);
                return false;
            }
        }
    }
    sysop("dsb sy");
    return true;
}

static bool sptm_dart_flush_sid(struct sptm_dart *dart, u32 sid)
{
    return sptm_dart_tlb_op(dart, DART_TLB_FLUSH_SID | sid, 0, 0);
}

static bool sptm_dart_flush_range(struct sptm_dart *dart, u32 sid, u64 start, u64 end)
{
    if (!(dart->flags & SPTM_DART_FLUSH_BY_DVA) || !dart->hardware_flush_supported || start >= end)
        return sptm_dart_flush_sid(dart, sid);
    return sptm_dart_tlb_op(
        dart, DART_TLB_HARDWARE_FLUSH | DART_TLB_FLUSH_DVA | DART_TLB_FLUSH_SID | sid, start, end);
}

// Flush the DVA range covered by one entry of a level-`level` table.
static bool sptm_dart_flush_entry(struct sptm_dart *dart, u32 sid, u64 dva, u32 level)
{
    u64 span = BIT(sptm_dart_shifts[level]);
    u64 start = dva & ~(span - 1);
    return sptm_dart_flush_range(dart, sid, start, start + span);
}

bool sptm_dart_flush_all(struct sptm_dart *dart)
{
    return sptm_dart_tlb_op(dart, DART_TLB_FLUSH_ALL, 0, 0);
}

struct sptm_dart_sid *sptm_dart_sid(const struct sptm_dart *dart, u32 sid)
{
    if (!dart->sid_states || sid >= dart->sid_count)
        return NULL;
    return &dart->sid_states[sid];
}

static struct sptm_dart_sid *sptm_dart_translating_sid(const struct sptm_dart *dart, u32 sid)
{
    struct sptm_dart_sid *state = dart ? sptm_dart_sid(dart, sid) : NULL;
    if (!state || !(state->flags & SPTM_DART_SID_KNOWN) ||
        (state->flags & (SPTM_DART_SID_POLICY | SPTM_DART_SID_EXCLAVE)) ||
        !(state->tcr & DART_TCR_TRANSLATE))
        return NULL;
    return state;
}

bool sptm_dart_valid_table(const struct sptm_dart_sid *state, u64 pa, size_t size)
{
    if (sptm_valid_pa(pa, size)) {
        u64 start = pa & ~SPTM_PAGE_MASK;
        u64 end = ALIGN_UP(pa + size, SPTM_PAGE_SIZE);

        for (u64 page = start; page < end; page += SPTM_PAGE_SIZE) {
            u8 *entry = sptm_frame_entry(page);
            if (!entry || entry[2] != SPTM_FRAME_XNU_IOMMU)
                return false;
        }
        return true;
    }
    return state && state->pt_start < state->pt_end && pa >= state->pt_start &&
           pa < state->pt_end && size <= state->pt_end - pa;
}

static bool sptm_dart_validate_dva(const struct sptm_dart_sid *state, u64 dva, u64 size)
{
    if (!state->dva_size)
        return true;
    if (dva < state->dva_base)
        return false;

    u64 offset = dva - state->dva_base;
    return offset <= state->dva_size && size <= state->dva_size - offset;
}

static u64 sptm_dart_desc_pa(u64 descriptor)
{
    return ((descriptor >> 10) & DART_PTE_OFFSET_MASK) << SPTM_PAGE_SHIFT;
}

static u64 sptm_dart_desc(u64 pa)
{
    return (((pa >> SPTM_PAGE_SHIFT) & DART_PTE_OFFSET_MASK) << 10) | DART_PTE_VALID;
}

/*
 * Walk from the SID's root to dva's slot in the level-`level` table. Live DART
 * tables are only accessed through their Normal-NC alias, which is returned.
 */
static u64 sptm_dart_slot(const struct sptm_dart_sid *state, u64 dva, u32 level)
{
    u64 table = state->root;
    if (!table || !sptm_dart_valid_table(state, table, SPTM_PAGE_SIZE))
        return 0;

    for (u32 current = state->root_level;; current++) {
        u64 slot =
            sptm_iommu_table_va(table) + ((dva >> sptm_dart_shifts[current]) & 0x7ff) * sizeof(u64);
        if (current == level)
            return slot;

        u64 descriptor = read64(slot);
        if (!(descriptor & DART_PTE_VALID))
            return 0;
        table = sptm_dart_desc_pa(descriptor);
        if (!sptm_dart_valid_table(state, table, SPTM_PAGE_SIZE))
            return 0;
    }
}

// Firmware-locked TTBRs cannot be written, so they must already hold the value.
static bool sptm_dart_set_ttbr(struct sptm_dart *dart, u32 sid, u32 ttbr)
{
    if (!dart->powered)
        return true;

    for (size_t index = 0; index < dart->instance_count; index++) {
        u64 base = dart->instances[index];
        u64 reg = base + DART_TTBR + sid * sizeof(u32);
        if (!(read32(base + DART_REG_PROTECT) & BIT(0)))
            write32(reg, ttbr);
        else if (read32(reg) != ttbr)
            return false;
    }
    return true;
}

static bool sptm_dart_map_table(struct exc_info *ctx)
{
    struct sptm_dart *dart = sptm_find_dart(ctx->regs[0]);
    u32 sid = ctx->regs[1];
    u64 dva = ctx->regs[2];
    u32 level = ctx->regs[3];
    u64 table_pa = ctx->regs[4];
    struct sptm_dart_sid *state = sptm_dart_translating_sid(dart, sid);

    if (!state || level > 2 || level < state->root_level || (table_pa & SPTM_PAGE_MASK) ||
        !sptm_dart_valid_table(state, table_pa, SPTM_PAGE_SIZE) ||
        !sptm_dart_validate_dva(state, dva, 1))
        return false;

    if (level == state->root_level) {
        if ((state->root && state->root != table_pa) ||
            !sptm_dart_set_ttbr(dart, sid, ((table_pa >> 12) & 0xfffffffcULL) | DART_TTBR_VALID))
            return false;
        state->root = table_pa;
        if (!sptm_dart_flush_sid(dart, sid))
            return false;
    } else {
        u64 slot = sptm_dart_slot(state, dva, level - 1);
        u64 replacement = sptm_dart_desc(table_pa);
        u64 old = slot ? read64(slot) : 0;
        if (!slot || ((old & DART_PTE_VALID) && old != replacement))
            return false;
        write64(slot, replacement);
        if (!sptm_dart_flush_entry(dart, sid, dva, level - 1))
            return false;
    }

    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_dart_unmap_table(struct exc_info *ctx)
{
    struct sptm_dart *dart = sptm_find_dart(ctx->regs[0]);
    u32 sid = ctx->regs[1];
    u64 dva = ctx->regs[2];
    u32 level = ctx->regs[3];
    struct sptm_dart_sid *state = sptm_dart_translating_sid(dart, sid);

    if (!state || level > 2 || level < state->root_level || !state->root ||
        !sptm_dart_valid_table(state, state->root, SPTM_PAGE_SIZE) ||
        !sptm_dart_validate_dva(state, dva, 1))
        return false;

    if (level == state->root_level) {
        if (!sptm_dart_set_ttbr(dart, sid, 0))
            return false;
        state->root = 0;
        if (!sptm_dart_flush_sid(dart, sid))
            return false;
    } else {
        u64 slot = sptm_dart_slot(state, dva, level - 1);
        if (!slot || !(read64(slot) & DART_PTE_VALID))
            return false;
        write64(slot, 0);
        if (!sptm_dart_flush_entry(dart, sid, dva, level - 1))
            return false;
    }

    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_dart_page_range(u64 dva, u64 size, u64 *start, u64 *end, size_t *count)
{
    if (!size) {
        *start = dva & ~SPTM_PAGE_MASK;
        *end = *start;
        *count = 0;
        return true;
    }

    if (dva > UINT64_MAX - size || dva + size > UINT64_MAX - SPTM_PAGE_MASK)
        return false;

    *start = dva & ~SPTM_PAGE_MASK;
    *end = (dva + size + SPTM_PAGE_MASK) & ~SPTM_PAGE_MASK;
    *count = (*end - *start) >> SPTM_PAGE_SHIFT;
    return *count <= SPTM_MAX_DART_PAGES;
}

static u64 sptm_dart_page_pte(const struct sptm_dart *dart, u64 dva, u64 size, u64 page_dva,
                              u64 target, u64 options)
{
    // Restrict the leaf to the 4-byte subpages inside [dva, dva + size).
    u64 sp_start = (max(dva, page_dva) - page_dva) >> 2;
    u64 sp_end = (min(dva + size, page_dva + SPTM_PAGE_SIZE) - page_dva - 1) >> 2;
    bool wrprot = options & BIT(1);
    u8 *frame = sptm_frame_entry(target);
    if (frame && (dart->flags & SPTM_DART_RELAXED_RW))
        wrprot |= frame[2] == 14 || frame[2] == 15 || frame[2] == 16 ||
                  frame[2] == SPTM_FRAME_COPROCESSOR_RO_IO;

    return ((sp_start & 0xfff) << 52) | ((sp_end & 0xfff) << 40) | sptm_dart_desc(target) |
           (options & BIT(0) ? DART_PTE_RDPROT : 0) | (wrprot ? DART_PTE_WRPROT : 0) |
           (options & BIT(2) ? DART_PTE_UNCACHABLE : 0);
}

static bool sptm_dart_map_pages(struct exc_info *ctx)
{
    struct sptm_dart *dart = sptm_find_dart(ctx->regs[0]);
    u32 sid = ctx->regs[1];
    u64 dva = ctx->regs[2];
    u64 size = ctx->regs[4];
    u64 options = ctx->regs[5];
    struct sptm_dart_sid *state = sptm_dart_translating_sid(dart, sid);
    u64 start, end;
    size_t count;

    if (!state || !state->root || (options & ~0xfULL) ||
        !sptm_dart_validate_dva(state, dva, size) ||
        !sptm_dart_page_range(dva, size, &start, &end, &count))
        return false;

    u64 targets = 0;
    if (count && !(targets = sptm_pointer_pa(ctx->regs[3], count * sizeof(u64))))
        return false;

    bool changed = false;
    bool all_fresh = true;
    for (size_t index = 0; index < count; index++) {
        u64 page_dva = start + index * SPTM_PAGE_SIZE;
        u64 target = read64(targets + index * sizeof(u64));
        u64 slot = sptm_dart_slot(state, page_dva, 2);
        if ((target & SPTM_PAGE_MASK) || !slot)
            return false;

        u64 old = read64(slot);
        if (old & DART_PTE_VALID) {
            all_fresh = false;
            // Only write-protected leaves may be retargeted, and only on request.
            if (sptm_dart_desc_pa(old) != target &&
                (!(options & BIT(3)) || !(old & DART_PTE_WRPROT)))
                return false;
        }
        changed |= old != sptm_dart_page_pte(dart, dva, size, page_dva, target, options);
    }

    // Pin every replacement target before publishing the batch.
    size_t prepared = 0;
    for (; prepared < count; prepared++) {
        u64 target = read64(targets + prepared * sizeof(u64));
        u64 old = read64(sptm_dart_slot(state, start + prepared * SPTM_PAGE_SIZE, 2));
        u64 old_target = (old & DART_PTE_VALID) ? sptm_dart_desc_pa(old) : 0;
        if (old_target == target)
            continue;
        if (!sptm_adjust_iommu_use(target, 1))
            goto rollback;
        if (old_target && !sptm_adjust_iommu_use(old_target, -1)) {
            sptm_adjust_iommu_use(target, -1);
            goto rollback;
        }
    }

    for (size_t index = 0; index < count; index++) {
        u64 page_dva = start + index * SPTM_PAGE_SIZE;
        u64 target = read64(targets + index * sizeof(u64));
        write64(sptm_dart_slot(state, page_dva, 2),
                sptm_dart_page_pte(dart, dva, size, page_dva, target, options));
    }
    sysop("dsb sy");
    bool skip_fresh = (dart->flags & SPTM_DART_AVOID_MAP_TLBI) && all_fresh;
    if (changed && !(options & BIT(3)) && !skip_fresh &&
        !sptm_dart_flush_range(dart, sid, start, end))
        return false;
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;

rollback:
    for (size_t index = 0; index < prepared; index++) {
        u64 target = read64(targets + index * sizeof(u64));
        u64 old = read64(sptm_dart_slot(state, start + index * SPTM_PAGE_SIZE, 2));
        u64 old_target = (old & DART_PTE_VALID) ? sptm_dart_desc_pa(old) : 0;
        if (old_target == target)
            continue;
        if (old_target)
            sptm_adjust_iommu_use(old_target, 1);
        sptm_adjust_iommu_use(target, -1);
    }
    return false;
}

static bool sptm_dart_unmap_pages(struct exc_info *ctx)
{
    struct sptm_dart *dart = sptm_find_dart(ctx->regs[0]);
    u32 sid = ctx->regs[1];
    u64 dva = ctx->regs[2];
    u64 size = ctx->regs[3];
    u64 options = ctx->regs[4];
    struct sptm_dart_sid *state = sptm_dart_translating_sid(dart, sid);
    u64 start, end;
    size_t count;

    if (!state || !state->root || (options & ~3ULL) ||
        ((options & BIT(0)) && !(dart->flags & SPTM_DART_ALLOW_PTE_REMAP)) ||
        !sptm_dart_validate_dva(state, dva, size) ||
        !sptm_dart_page_range(dva, size, &start, &end, &count))
        return false;

    bool changed = false;
    for (size_t index = 0; index < count; index++) {
        u64 slot = sptm_dart_slot(state, start + index * SPTM_PAGE_SIZE, 2);
        if (!slot)
            return false;
        u64 old = read64(slot);
        if (!(old & DART_PTE_VALID)) {
            if (!(options & BIT(1)))
                return false;
            continue;
        }
        u8 *frame = sptm_frame_entry(sptm_dart_desc_pa(old));
        if (frame && frame[2] == SPTM_FRAME_TXM_SECURE_CHANNEL)
            return false;
        changed = true;
    }

    size_t released = 0;
    for (; released < count; released++) {
        u64 old = read64(sptm_dart_slot(state, start + released * SPTM_PAGE_SIZE, 2));
        u64 target = sptm_dart_desc_pa(old);
        if ((old & DART_PTE_VALID) && (!target || !sptm_adjust_iommu_use(target, -1)))
            goto rollback;
    }

    for (size_t index = 0; index < count; index++)
        write64(sptm_dart_slot(state, start + index * SPTM_PAGE_SIZE, 2), 0);
    sysop("dsb sy");
    if (changed && !sptm_dart_flush_range(dart, sid, start, end))
        return false;
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;

rollback:
    for (size_t index = 0; index < released; index++) {
        u64 old = read64(sptm_dart_slot(state, start + index * SPTM_PAGE_SIZE, 2));
        u64 target = sptm_dart_desc_pa(old);
        if ((old & DART_PTE_VALID) && target)
            sptm_adjust_iommu_use(target, 1);
    }
    return false;
}

static bool sptm_dart_set_stream(struct exc_info *ctx, bool enable)
{
    struct sptm_dart *dart = sptm_find_dart(ctx->regs[0]);
    u32 sid = ctx->regs[1];
    struct sptm_dart_sid *state = dart ? sptm_dart_sid(dart, sid) : NULL;
    if (!state || !(state->flags & SPTM_DART_SID_KNOWN) || (state->flags & SPTM_DART_SID_EXCLAVE))
        return false;

    // Non-translating SIDs keep their streams, and retention DARTs keep them enabled.
    if ((state->tcr & DART_TCR_TRANSLATE) && (enable || !(dart->flags & SPTM_DART_RETENTION))) {
        u32 word = sid / 32;
        u32 bit = BIT(sid % 32);
        if (dart->powered) {
            u64 offset = (enable ? DART_ENABLE_STREAMS : DART_DISABLE_STREAMS) + word * sizeof(u32);
            for (size_t index = 0; index < dart->instance_count; index++)
                write32(dart->instances[index] + offset, bit);
        } else if (dart->saved_valid) {
            /* Power-up restores the deferred stream state. */
            for (size_t index = 0; index < dart->instance_count; index++) {
                if (enable)
                    dart->saved_streams[index][word] |= bit;
                else
                    dart->saved_streams[index][word] &= ~bit;
            }
        }
        sysop("dsb sy");
    }

    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

bool sptm_handle_dart(struct exc_info *ctx, u32 endpoint)
{
    bool handled;
    spin_lock(&sptm.service_lock);
    switch (endpoint) {
        case 0: /* MAP_TABLE */
            handled = sptm_dart_map_table(ctx);
            break;
        case 1: /* UNMAP_TABLE */
            handled = sptm_dart_unmap_table(ctx);
            break;
        case 2: /* MAP_PAGE */
            handled = sptm_dart_map_pages(ctx);
            break;
        case 3: /* UNMAP_PAGE */
            handled = sptm_dart_unmap_pages(ctx);
            break;
        case 4: /* POWERDOWN */
            handled = sptm_dart_power(ctx, false);
            break;
        case 5: /* POWERUP */
            handled = sptm_dart_power(ctx, true);
            break;
        case 6: /* INIT */
            handled = sptm_dart_init(ctx);
            break;
        case 7: /* DISABLE_TRANSLATION */
            handled = sptm_dart_set_stream(ctx, false);
            break;
        case 8: /* ENABLE_TRANSLATION */
            handled = sptm_dart_set_stream(ctx, true);
            break;
        default: /* The remaining endpoints, including error clearing, are no-ops. */
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            handled = true;
            break;
    }
    spin_unlock(&sptm.service_lock);
    return handled;
}

void sptm_dart_configure(const struct sptm_dart_config *config)
{
    u32 id = config->id;
    if (!config->sid_count || config->sid_count > SPTM_MAX_DART_SIDS) {
        printf("HV: refusing invalid SPTM DART %u configuration\n", id);
        return;
    }

    for (size_t index = 0; index < config->instance_count; index++) {
        if (!config->instances[index] || (config->instances[index] & 3)) {
            printf("HV: refusing invalid SPTM DART %u instance 0x%lx\n", id,
                   config->instances[index]);
            return;
        }
    }

    struct sptm_dart *dart = sptm_find_dart(id);
    if (!dart) {
        if (sptm.dart_count >= ARRAY_SIZE(sptm.darts)) {
            printf("HV: refusing excess SPTM DART %u\n", id);
            return;
        }
        dart = &sptm.darts[sptm.dart_count++];
    }

    memset(dart, 0, sizeof(*dart));
    dart->id = id;
    dart->sid_count = config->sid_count;
    dart->instance_count = config->instance_count;
    dart->sid_states = config->sid_states;
    dart->dapf_entries = config->dapf_entries;
    dart->clock_entries = config->clock_entries;
    dart->tunable_entries = config->tunable_entries;
    dart->dapf_count = config->dapf_count;
    dart->clock_count = config->clock_count;
    dart->tunable_count = config->tunable_count;
    dart->flags = config->flags;
    for (u32 sid = 0; sid < dart->sid_count; sid++) {
        struct sptm_dart_sid *state = sptm_dart_sid(dart, sid);
        if (state && (state->pt_start || state->pt_end)) {
            u64 size = state->pt_end - state->pt_start;
            if ((state->pt_start & SPTM_PAGE_MASK) ||
                (state->pt_end & SPTM_PAGE_MASK) || state->pt_end <= state->pt_start ||
                !sptm_valid_platform_pa(state->pt_start, size) ||
                (state->pt_start < sptm.managed_end &&
                 sptm.managed_start < state->pt_end) ||
                (state->root &&
                 (state->root < state->pt_start || state->root >= state->pt_end))) {
                printf("HV: refusing invalid SPTM DART %u SID %u page-table region "
                       "0x%lx..0x%lx root=0x%lx\n",
                       id, sid, state->pt_start, state->pt_end, state->root);
                memset(dart, 0, sizeof(*dart));
                return;
            }

            // Firmware tables bypass RETYPE, so retire their WB aliases here.
            for (u64 page = state->pt_start; page < state->pt_end;
                 page += SPTM_PAGE_SIZE) {
                u64 host_page = page;
                mmu_map_ram_pages_nc(&host_page, 1, true);
            }
        }
        if (state && (state->flags & SPTM_DART_SID_ENABLED))
            dart->boot_streams[sid / 32] |= BIT(sid % 32);
    }
    dart->valid = true;
    memcpy(dart->instances, config->instances, sizeof(dart->instances));
}
