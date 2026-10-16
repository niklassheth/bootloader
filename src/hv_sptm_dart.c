/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

static const u32 sptm_dart_shifts[] = {36, 25, SPTM_PAGE_SHIFT};

static struct sptm_dart *sptm_find_dart(u32 id)
{
    for (size_t index = 0; index < sptm.dart_count; index++) {
        if (sptm.darts[index].id == id)
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
    if (!(dart->flags & SPTM_DART_FLUSH_BY_DVA) || !dart->hardware_flush_supported)
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
    for (u32 current = state->root_level;; current++) {
        u64 slot =
            sptm_iommu_table_va(table) + ((dva >> sptm_dart_shifts[current]) & 0x7ff) * sizeof(u64);
        if (current == level)
            return slot;
        table = sptm_dart_desc_pa(read64(slot));
    }
}

// Firmware-locked TTBRs already hold their value and cannot be written.
static void sptm_dart_set_ttbr(struct sptm_dart *dart, u32 sid, u32 ttbr)
{
    if (!dart->powered)
        return;

    for (size_t index = 0; index < dart->instance_count; index++) {
        u64 base = dart->instances[index];
        if (!(read32(base + DART_REG_PROTECT) & BIT(0)))
            write32(base + DART_TTBR + sid * sizeof(u32), ttbr);
    }
}

// MAP_TABLE and UNMAP_TABLE: install or clear the SID's root, or one table descriptor.
static bool sptm_dart_set_table(struct exc_info *ctx, struct sptm_dart *dart, bool map)
{
    u32 sid = ctx->regs[1];
    u64 dva = ctx->regs[2];
    u32 level = ctx->regs[3];
    u64 table_pa = map ? ctx->regs[4] : 0;
    struct sptm_dart_sid *state = &dart->sid_states[sid];
    bool flushed;

    if (level == state->root_level) {
        sptm_dart_set_ttbr(dart, sid,
                           map ? ((table_pa >> 12) & 0xfffffffcULL) | DART_TTBR_VALID : 0);
        state->root = table_pa;
        flushed = sptm_dart_flush_sid(dart, sid);
    } else {
        write64(sptm_dart_slot(state, dva, level - 1), map ? sptm_dart_desc(table_pa) : 0);
        flushed = sptm_dart_flush_entry(dart, sid, dva, level - 1);
    }

    if (flushed)
        ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return flushed;
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

static bool sptm_dart_map_pages(struct exc_info *ctx, struct sptm_dart *dart)
{
    u32 sid = ctx->regs[1];
    u64 dva = ctx->regs[2];
    u64 targets = sptm_pointer_pa(ctx->regs[3]);
    u64 size = ctx->regs[4];
    u64 options = ctx->regs[5];
    struct sptm_dart_sid *state = &dart->sid_states[sid];
    u64 start = dva & ~SPTM_PAGE_MASK;
    u64 end = ALIGN_UP(dva + size, SPTM_PAGE_SIZE);
    bool changed = false;
    bool all_fresh = true;

    for (u64 page_dva = start; page_dva < end; page_dva += SPTM_PAGE_SIZE) {
        u64 target = read64(targets + ((page_dva - start) >> SPTM_PAGE_SHIFT) * sizeof(u64));
        u64 slot = sptm_dart_slot(state, page_dva, 2);
        u64 old = read64(slot);
        u64 old_target = (old & DART_PTE_VALID) ? sptm_dart_desc_pa(old) : 0;
        u64 pte = sptm_dart_page_pte(dart, dva, size, page_dva, target, options);

        // Pin the new target before the leaf can reach it.
        if (old_target != target && (!sptm_adjust_iommu_use(target, 1) ||
                                     (old_target && !sptm_adjust_iommu_use(old_target, -1))))
            return false;
        all_fresh &= !old_target;
        changed |= old != pte;
        write64(slot, pte);
    }
    sysop("dsb sy");

    bool skip_fresh = (dart->flags & SPTM_DART_AVOID_MAP_TLBI) && all_fresh;
    if (changed && !(options & BIT(3)) && !skip_fresh &&
        !sptm_dart_flush_range(dart, sid, start, end))
        return false;
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_dart_unmap_pages(struct exc_info *ctx, struct sptm_dart *dart)
{
    u32 sid = ctx->regs[1];
    u64 dva = ctx->regs[2];
    u64 size = ctx->regs[3];
    struct sptm_dart_sid *state = &dart->sid_states[sid];
    u64 start = dva & ~SPTM_PAGE_MASK;
    u64 end = ALIGN_UP(dva + size, SPTM_PAGE_SIZE);
    bool changed = false;

    for (u64 page_dva = start; page_dva < end; page_dva += SPTM_PAGE_SIZE) {
        u64 slot = sptm_dart_slot(state, page_dva, 2);
        u64 old = read64(slot);
        if (!(old & DART_PTE_VALID))
            continue;
        if (!sptm_adjust_iommu_use(sptm_dart_desc_pa(old), -1))
            return false;
        write64(slot, 0);
        changed = true;
    }
    sysop("dsb sy");

    if (changed && !sptm_dart_flush_range(dart, sid, start, end))
        return false;
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_dart_set_stream(struct exc_info *ctx, struct sptm_dart *dart, bool enable)
{
    u32 sid = ctx->regs[1];
    struct sptm_dart_sid *state = &dart->sid_states[sid];

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
    // The remaining endpoints, including error clearing, are no-ops.
    if (endpoint > 8) {
        ctx->regs[0] = SPTM_STATUS_SUCCESS;
        return true;
    }

    struct sptm_dart *dart = sptm_find_dart(ctx->regs[0]);
    if (!dart)
        return false;

    bool handled;
    spin_lock(&sptm.service_lock);
    switch (endpoint) {
        case 0: /* MAP_TABLE */
            handled = sptm_dart_set_table(ctx, dart, true);
            break;
        case 1: /* UNMAP_TABLE */
            handled = sptm_dart_set_table(ctx, dart, false);
            break;
        case 2: /* MAP_PAGE */
            handled = sptm_dart_map_pages(ctx, dart);
            break;
        case 3: /* UNMAP_PAGE */
            handled = sptm_dart_unmap_pages(ctx, dart);
            break;
        case 4: /* POWERDOWN */
            handled = sptm_dart_power(ctx, dart, false);
            break;
        case 5: /* POWERUP */
            handled = sptm_dart_power(ctx, dart, true);
            break;
        case 6: /* INIT */
            handled = sptm_dart_init(ctx, dart);
            break;
        case 7: /* DISABLE_TRANSLATION */
            handled = sptm_dart_set_stream(ctx, dart, false);
            break;
        default: /* ENABLE_TRANSLATION */
            handled = sptm_dart_set_stream(ctx, dart, true);
            break;
    }
    spin_unlock(&sptm.service_lock);
    return handled;
}
