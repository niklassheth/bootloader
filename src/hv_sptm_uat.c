/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

static bool sptm_uat_sentinel(u64 root)
{
    return root == UINT32_MAX || root == UINT64_MAX;
}

static void sptm_uat_handoff_acquire(u64 address, size_t size)
{
    u64 start = ALIGN_DOWN(address, SPTM_CACHE_LINE_SIZE);
    size = ALIGN_UP(address - start + size, SPTM_CACHE_LINE_SIZE);
    sysop("dsb osh");
    dc_ivac_range((void *)start, size);
    sysop("dsb osh");
}

static u64 sptm_uat_handoff_slot_addr(u32 slot)
{
    return sptm.uat_handoff_pa + UAT_HANDOFF_SLOT_BASE + slot * UAT_HANDOFF_SLOT_STRIDE;
}

// Global states share one handoff slot; per-context states use their context's slot.
static u32 sptm_uat_handoff_slot_for_state(u64 state_pa)
{
    u8 state_type = read8(state_pa);
    if (state_type == UAT_STATE_GLOBAL_MODE0 || state_type == UAT_STATE_GLOBAL_MODE1)
        return UAT_HANDOFF_SHARED_SLOT;
    return read16(state_pa + UAT_STATE_CONTEXT_ID);
}

static u64 sptm_uat_handoff_post(u64 state_pa, u32 slot, u64 va, u64 count)
{
    if (!count)
        return 0;

    u64 handoff = sptm.uat_handoff_pa;
    sptm_uat_handoff_acquire(handoff, UAT_HANDOFF_SLOT_BASE);
    bool firmware_accepting = read64(handoff + UAT_HANDOFF_MAGIC_FW) == UAT_HANDOFF_MAGIC;
    bool could_be_cached =
        slot == UAT_HANDOFF_SHARED_SLOT || read32(handoff + UAT_HANDOFF_CURRENT_CTX) == slot;
    bool request_flush = firmware_accepting && could_be_cached;
    u64 slot_addr = sptm_uat_handoff_slot_addr(slot);
    u64 address = request_flush ? va : UAT_HANDOFF_UNMAP_TAG | (va & UAT_HANDOFF_ADDR_MASK);

    write64(slot_addr + UAT_HANDOFF_SLOT_ADDR, address);
    write64(slot_addr + UAT_HANDOFF_SLOT_SIZE, count * SPTM_PAGE_SIZE);
    sysop("dsb osh");
    write64(slot_addr + UAT_HANDOFF_SLOT_STATE, request_flush ? 1 : 2);
    sysop("dsb osh");

    sptm.uat_handoff_pending[slot] = true;
    sptm.uat_handoff_owner[slot] = state_pa;
    return request_flush ? 2 : 0;
}

static void sptm_uat_handoff_complete(u64 state_pa)
{
    for (u32 slot = 0; slot < UAT_HANDOFF_SLOT_COUNT; slot++) {
        if (!sptm.uat_handoff_pending[slot] || sptm.uat_handoff_owner[slot] != state_pa)
            continue;

        u64 state_addr = sptm_uat_handoff_slot_addr(slot) + UAT_HANDOFF_SLOT_STATE;
        sptm_uat_handoff_acquire(state_addr, sizeof(u32));
        write32(state_addr, 0);
        sysop("dsb osh");
        sptm.uat_handoff_pending[slot] = false;
        sptm.uat_handoff_owner[slot] = 0;
    }
}

static void sptm_uat_publish_guard(u64 state_pa, u8 guard)
{
    sysop("dsb osh");
    write8(state_pa + UAT_STATE_GUARD, guard);
    sysop("dsb osh");
}

static void sptm_uat_write_descriptor(u64 slot, u64 descriptor)
{
    // this is where we would use the coprocessor flush op
    write64(slot, descriptor);
    sysop("dsb oshst");
}

// UAT tables, and state objects, are pinned through the frame's IOMMU refcount.
static bool sptm_uat_ref_table(u64 pa, int delta)
{
    return sptm_adjust_frame_u16(sptm_frame_entry(pa & SPTM_DESC_PA_MASK_16K), 4, delta);
}

// The firmware-owned shared L2 is not a UAT table frame and is never refcounted.
static bool sptm_uat_ref_parent(u64 pa, int delta)
{
    pa &= SPTM_DESC_PA_MASK_16K;
    return pa == sptm.uat_shared_l2_pa || sptm_uat_ref_table(pa, delta);
}

static bool sptm_uat_ref_page(u64 descriptor, int delta)
{
    // Track UAT leaves through the frame header's common in_use count.
    return sptm_adjust_iommu_use(descriptor & SPTM_DESC_PA_MASK_16K, delta);
}

static u64 sptm_uat_normalize_va(u64 va)
{
    return va & (BIT(sptm.uat_va_width) - 1);
}

void sptm_uat_tlbi_all(void)
{
    // Match VMALLE1OS's outer-shareable domain.
    sysop("dsb oshst");
    __asm__ volatile("sys #0, c8, c1, #0, xzr" ::: "memory");
    sysop("dsb osh");
    sysop("isb");
}

static u64 sptm_uat_root(u64 state_pa, u64 va)
{
    u64 root_offset = UAT_STATE_ROOT0;
    u8 state_type = read8(state_pa);
    /* Boot-seeded shared-TTBR1 objects intentionally have no root0. */
    if (state_type == UAT_STATE_GLOBAL_MODE0 || state_type == UAT_STATE_GLOBAL_MODE1 ||
        (state_type == UAT_STATE_DUAL_ROOT && ((va >> (sptm.uat_va_width - 1)) & 1)))
        root_offset = UAT_STATE_ROOT1;
    return read64(state_pa + root_offset) & SPTM_DESC_PA_MASK_16K;
}

// Return the Normal-NC alias of va's slot in the level-`level` table.
static u64 sptm_uat_slot(u64 state_pa, u64 va, u32 level)
{
    va = sptm_uat_normalize_va(va);
    u64 table = sptm_uat_root(state_pa, va);

    // The L1 index uses the VA bits above the 16 KiB three-level walk.
    u64 l1_mask = BIT(max((int)sptm.uat_va_width - 37, 0)) - 1;
    u64 indices[] = {
        (va >> 36) & l1_mask,
        (va >> 25) & 0x7ff,
        (va >> SPTM_PAGE_SHIFT) & 0x7ff,
    };

    for (u32 current_level = 1; current_level < level; current_level++)
        table = read64(sptm_iommu_table_va(table) + indices[current_level - 1] * sizeof(u64)) &
                SPTM_DESC_PA_MASK_16K;
    return sptm_iommu_table_va(table) + indices[level - 1] * sizeof(u64);
}

static bool sptm_uat_init_state(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 root0 = ctx->regs[1];
    u64 root1 = ctx->regs[2];
    bool dual = !sptm_uat_sentinel(root1);

    u64 roots[] = {state_pa, root0, root1};
    size_t root_count = dual ? ARRAY_SIZE(roots) : ARRAY_SIZE(roots) - 1;
    size_t pinned = 0;
    for (; pinned < root_count; pinned++) {
        if (!sptm_uat_ref_table(roots[pinned], 1))
            break;
    }
    if (pinned != root_count) {
        while (pinned)
            sptm_uat_ref_table(roots[--pinned], -1);
        return false;
    }

    memset((void *)state_pa, 0, SPTM_PAGE_SIZE);
    write8(state_pa, dual ? UAT_STATE_DUAL_ROOT : UAT_STATE_SINGLE_ROOT);
    write64(state_pa + UAT_STATE_ROOT0, root0);
    write64(state_pa + UAT_STATE_ROOT1, root1);
    write16(state_pa + UAT_STATE_CONTEXT_ID, UINT16_MAX);
    sptm_uat_publish_guard(state_pa, UAT_GUARD_IDLE);
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_uat_destroy_state(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 root0 = read64(state_pa + UAT_STATE_ROOT0);
    u64 root1 = read64(state_pa + UAT_STATE_ROOT1);
    u64 roots[] = {state_pa, root0, root1};
    size_t root_count = sptm_uat_sentinel(root1) ? 2 : ARRAY_SIZE(roots);
    for (size_t index = 0; index < root_count; index++)
        sptm_uat_ref_table(roots[index], -1);

    memset((void *)state_pa, 0, SPTM_PAGE_SIZE);
    write64(state_pa + UAT_STATE_ROOT0, UINT32_MAX);
    write64(state_pa + UAT_STATE_ROOT1, UINT32_MAX);
    write16(state_pa + UAT_STATE_CONTEXT_ID, UINT16_MAX);
    sysop("dsb osh");
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_uat_map_table(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 slot = sptm_uat_slot(state_pa, ctx->regs[1], ctx->regs[2]);
    u64 table_pa = ctx->regs[3] & SPTM_DESC_PA_MASK_16K;
    u64 parent_pa = slot & ~(REGION_NORMAL_NC | SPTM_PAGE_MASK);

    if (!sptm_uat_ref_table(table_pa, 1))
        return false;
    if (!sptm_uat_ref_parent(parent_pa, 1)) {
        sptm_uat_ref_table(table_pa, -1);
        return false;
    }

    sptm_uat_write_descriptor(slot, table_pa | UAT_DESC_TABLE);
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_uat_unmap_table(struct exc_info *ctx)
{
    u64 slot = sptm_uat_slot(ctx->regs[0], ctx->regs[1], ctx->regs[2]);
    u64 table_pa = read64(slot) & SPTM_DESC_PA_MASK_16K;
    u64 parent_pa = slot & ~(REGION_NORMAL_NC | SPTM_PAGE_MASK);

    sptm_uat_write_descriptor(slot, 0);
    sptm_uat_tlbi_all();
    sptm_uat_ref_table(table_pa, -1);
    sptm_uat_ref_parent(parent_pa, -1);
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static u64 sptm_uat_leaf_descriptor(u64 state_pa, u64 target, u32 options)
{
    // Preserve per-context memory types and permissions from the map options.
    static const u8 permissions[4][4] = {
        {0, 2, 2 | BIT(2), 2 | BIT(3)},
        {1, BIT(2), 0, 0},
        {1 | BIT(2), 0, BIT(3), 0},
        {1 | BIT(3), 1 | BIT(2) | BIT(3), 0, BIT(2) | BIT(3)},
    };

    u8 permission = permissions[options & 3][(options >> 8) & 3];
    u8 state_type = read8(state_pa);
    u64 attr = options & BIT(2) ? 1 : options & BIT(3) ? 2 : 0;
    u64 flags = UAT_DESC_OS | UAT_DESC_AF | UAT_DESC_VALID | BIT(1) |
                (attr << UAT_DESC_ATTR_SHIFT) | ((u64)(permission & 3) << UAT_DESC_AP_SHIFT);
    if (state_type == UAT_STATE_SINGLE_ROOT || state_type == UAT_STATE_DUAL_ROOT)
        flags |= UAT_DESC_NG;
    if (permission & BIT(2))
        flags |= UAT_DESC_PXN;
    if (permission & BIT(3))
        flags |= UAT_DESC_UXN;

    return (target & SPTM_DESC_PA_MASK_16K) | flags;
}

static bool sptm_uat_map_continue(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 va = read64(state_pa + UAT_STATE_CURSOR0);
    u64 segment_count = read64(state_pa + UAT_STATE_CURSOR1);
    u64 segment_index = read64(state_pa + UAT_STATE_CURSOR2);
    u64 page_index = read64(state_pa + UAT_STATE_CURSOR3);
    u32 options = read32(state_pa + UAT_STATE_OPTIONS);
    u32 budget = min(sptm.uat_mapping_limit, SPTM_UAT_BATCH_PAGES);

    while (budget && segment_index < segment_count) {
        struct sptm_uat_segment *segment =
            (void *)(state_pa + UAT_STATE_MAP_SEGMENTS + segment_index * sizeof(*segment));
        if (page_index >= segment->count) {
            segment_index++;
            page_index = 0;
            continue;
        }

        u64 slot = sptm_uat_slot(state_pa, va, 3);
        u64 target = segment->first + page_index * SPTM_PAGE_SIZE;
        u64 descriptor = sptm_uat_leaf_descriptor(state_pa, target, options);
        u64 old_descriptor = read64(slot);
        if (old_descriptor != descriptor) {
            bool old_valid = old_descriptor & UAT_DESC_VALID;
            bool changes_frame = !old_valid || (old_descriptor & SPTM_DESC_PA_MASK_16K) !=
                                                   (descriptor & SPTM_DESC_PA_MASK_16K);
            if (changes_frame && !sptm_uat_ref_page(descriptor, 1))
                return false;

            sptm_uat_write_descriptor(slot, descriptor);
            if (old_valid) {
                // Keep the old target pinned until the replacement is invalidated.
                sptm_uat_tlbi_all();
                if (changes_frame)
                    sptm_uat_ref_page(old_descriptor, -1);
            }
        }
        va += SPTM_PAGE_SIZE;
        page_index++;
        budget--;
    }

    write64(state_pa + UAT_STATE_CURSOR0, va);
    write64(state_pa + UAT_STATE_CURSOR2, segment_index);
    write64(state_pa + UAT_STATE_CURSOR3, page_index);
    bool done = segment_index >= segment_count;
    sptm_uat_publish_guard(state_pa, done ? UAT_GUARD_IDLE : UAT_GUARD_MAP);
    ctx->regs[0] = done ? 0 : 1;
    return true;
}

static bool sptm_uat_map_begin(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 segment_count = ctx->regs[3];

    memcpy((void *)(state_pa + UAT_STATE_MAP_SEGMENTS), (void *)ctx->regs[2],
           segment_count * sizeof(struct sptm_uat_segment));
    write64(state_pa + UAT_STATE_CURSOR0, sptm_uat_normalize_va(ctx->regs[1]));
    write64(state_pa + UAT_STATE_CURSOR1, segment_count);
    write64(state_pa + UAT_STATE_CURSOR2, 0);
    write64(state_pa + UAT_STATE_CURSOR3, 0);
    write32(state_pa + UAT_STATE_OPTIONS, ctx->regs[4]);
    write8(state_pa + UAT_STATE_FLUSH_LEVEL, 0);
    sptm_uat_publish_guard(state_pa, UAT_GUARD_MAP);
    return sptm_uat_map_continue(ctx);
}

static bool sptm_uat_prepare_unmap_continue(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 va = read64(state_pa + UAT_STATE_CURSOR0);
    u64 count = read64(state_pa + UAT_STATE_CURSOR1);
    u64 processed = read64(state_pa + UAT_STATE_CURSOR2);
    u32 budget = sptm.uat_mapping_limit;

    for (; budget && processed < count; processed++, budget--) {
        u64 slot = sptm_uat_slot(state_pa, va + processed * SPTM_PAGE_SIZE, 3);
        u64 descriptor = read64(slot);
        u32 attr = (descriptor >> UAT_DESC_ATTR_SHIFT) & 7;
        u32 ap = (descriptor >> UAT_DESC_AP_SHIFT) & 3;
        bool firmware_candidate =
            !attr && ((!ap && (descriptor & UAT_DESC_UXN)) ||
                      (ap == 1 && (descriptor & (UAT_DESC_PXN | UAT_DESC_UXN))));
        if ((descriptor & UAT_DESC_VALID) && firmware_candidate)
            sptm_uat_write_descriptor(slot, descriptor | UAT_DESC_FIRMWARE_OWNED);
    }

    write64(state_pa + UAT_STATE_CURSOR2, processed);
    bool done = processed >= count;
    u64 result = 1;
    if (done) {
        // CURSOR3 holds the handoff slot plus one, or zero for an empty range.
        u64 slot_cookie = read64(state_pa + UAT_STATE_CURSOR3);
        result = slot_cookie ? sptm_uat_handoff_post(state_pa, slot_cookie - 1, va, count) : 0;
    }
    sptm_uat_publish_guard(state_pa, done ? UAT_GUARD_IDLE : UAT_GUARD_PREPARE_UNMAP);
    ctx->regs[0] = result;
    return true;
}

static bool sptm_uat_prepare_unmap_begin(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 count = ctx->regs[2];

    write64(state_pa + UAT_STATE_CURSOR0, sptm_uat_normalize_va(ctx->regs[1]));
    write64(state_pa + UAT_STATE_CURSOR1, count);
    write64(state_pa + UAT_STATE_CURSOR2, 0);
    write64(state_pa + UAT_STATE_CURSOR3,
            count ? sptm_uat_handoff_slot_for_state(state_pa) + 1 : 0);
    sptm_uat_publish_guard(state_pa, UAT_GUARD_PREPARE_UNMAP);
    return sptm_uat_prepare_unmap_continue(ctx);
}

static bool sptm_uat_unmap_continue(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 segment_count = read64(state_pa + UAT_STATE_CURSOR0);
    u64 segment_index = read64(state_pa + UAT_STATE_CURSOR1);
    u64 page_index = read64(state_pa + UAT_STATE_CURSOR2);
    u64 phase = read64(state_pa + UAT_STATE_CURSOR3);
    u32 budget = min(sptm.uat_mapping_limit, SPTM_UAT_BATCH_PAGES);
    bool cleared = false;

    if (!segment_count) {
        sptm_uat_publish_guard(state_pa, UAT_GUARD_IDLE);
        ctx->regs[0] = 0;
        return true;
    }

    // Validate, then clear VALID/TYPE while keeping the payload for handoff, then release.
    while (budget && segment_index < segment_count) {
        struct sptm_uat_segment *segment =
            (void *)(state_pa + UAT_STATE_UNMAP_SEGMENTS + segment_index * sizeof(*segment));
        if (page_index >= segment->count) {
            segment_index++;
            page_index = 0;
            continue;
        }

        u64 slot = sptm_uat_slot(state_pa, segment->first + page_index * SPTM_PAGE_SIZE, 3);
        u64 descriptor = read64(slot);
        if (phase == UAT_UNMAP_PHASE_CLEAR) {
            sptm_uat_write_descriptor(slot, descriptor & ~3ULL);
            cleared = true;
        } else if (phase == UAT_UNMAP_PHASE_RELEASE) {
            sptm_uat_ref_page(descriptor | 3, -1);
        }
        page_index++;
        budget--;
    }

    // Invalidate each cleared chunk before releasing target references.
    if (cleared)
        sptm_uat_tlbi_all();

    write64(state_pa + UAT_STATE_CURSOR1, segment_index);
    write64(state_pa + UAT_STATE_CURSOR2, page_index);
    if (segment_index < segment_count) {
        sptm_uat_publish_guard(state_pa, UAT_GUARD_UNMAP);
        ctx->regs[0] = 1;
        return true;
    }

    if (phase == UAT_UNMAP_PHASE_RELEASE) {
        sptm_uat_publish_guard(state_pa, UAT_GUARD_IDLE);
        ctx->regs[0] = 0;
        return true;
    }
    if (phase == UAT_UNMAP_PHASE_CLEAR)
        sptm_uat_handoff_complete(state_pa);

    write64(state_pa + UAT_STATE_CURSOR1, 0);
    write64(state_pa + UAT_STATE_CURSOR2, 0);
    write64(state_pa + UAT_STATE_CURSOR3, phase + 1);
    sptm_uat_publish_guard(state_pa, UAT_GUARD_UNMAP);
    ctx->regs[0] = 1;
    return true;
}

static bool sptm_uat_unmap_begin(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 segment_count = ctx->regs[2];

    memcpy((void *)(state_pa + UAT_STATE_UNMAP_SEGMENTS), (void *)ctx->regs[1],
           segment_count * sizeof(struct sptm_uat_segment));
    write64(state_pa + UAT_STATE_CURSOR0, segment_count);
    write64(state_pa + UAT_STATE_CURSOR1, 0);
    write64(state_pa + UAT_STATE_CURSOR2, 0);
    write64(state_pa + UAT_STATE_CURSOR3, UAT_UNMAP_PHASE_VALIDATE);
    write64(state_pa + UAT_STATE_UNMAP_FLUSH_COUNT, 0);
    sptm_uat_publish_guard(state_pa, UAT_GUARD_UNMAP);
    return sptm_uat_unmap_continue(ctx);
}

static u64 sptm_uat_ttbr(u64 root, u16 context_id)
{
    if (sptm_uat_sentinel(root))
        return 0;
    return (root & SPTM_DESC_PA_MASK_16K) | ((u64)context_id << 48) | 1;
}

static bool sptm_uat_set_context(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u64 context_id = ctx->regs[1];
    u64 context = sptm.uat_gpu_contexts + context_id * 16;
    u64 root0 = read64(state_pa + UAT_STATE_ROOT0);
    u64 root1 = read64(state_pa + UAT_STATE_ROOT1);
    // Single-root contexts share the global state's TTBR1.
    if (read8(state_pa) == UAT_STATE_SINGLE_ROOT)
        root1 = read64(sptm.uat_global_state + UAT_STATE_ROOT1);

    write64(context, sptm_uat_ttbr(root0, context_id));
    write64(context + 8, sptm_uat_ttbr(root1, context_id));
    sysop("dsb osh");
    write16(state_pa + UAT_STATE_CONTEXT_ID, context_id);
    sptm_uat_publish_guard(state_pa, UAT_GUARD_IDLE);
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_uat_remove_context(struct exc_info *ctx)
{
    u64 state_pa = ctx->regs[0];
    u16 context_id = read16(state_pa + UAT_STATE_CONTEXT_ID);
    if (context_id != UINT16_MAX) {
        u64 context = sptm.uat_gpu_contexts + context_id * 16;
        write64(context, read64(context) & ~1ULL);
        write64(context + 8, read64(context + 8) & ~1ULL);
        sysop("dsb osh");
        u64 asid = (u64)context_id << 48;
        __asm__ volatile("sys #0, c8, c1, #2, %0" : : "r"(asid) : "memory");
        sysop("dsb osh");
        sysop("isb");
    }
    write16(state_pa + UAT_STATE_CONTEXT_ID, UINT16_MAX);
    sptm_uat_publish_guard(state_pa, UAT_GUARD_IDLE);
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_uat_get_info(struct exc_info *ctx)
{
    const u64 values[] = {
        sptm.uat_mode,
        sptm.uat_mode == 0 ? sptm.uat_global_state : UINT64_MAX,
        sptm.uat_mode == 1 ? sptm.uat_global_state : UINT64_MAX,
        sptm.uat_va_width - 1,
        // The L1 bits, in their original VA positions.
        (BIT(max((int)sptm.uat_va_width - 37, 0)) - 1) << 36,
        sptm.uat_segment_limit,
        0,
        BIT(sptm.uat_va_width),
        sptm.uat_state_size,
        sptm.uat_shared_l2_va,
    };
    ctx->regs[0] = ctx->regs[0] < ARRAY_SIZE(values) ? values[ctx->regs[0]] : UINT64_MAX;
    return true;
}

bool sptm_handle_uat(struct exc_info *ctx, u32 endpoint)
{
    static bool (*const handlers[])(struct exc_info *) = {
        sptm_uat_init_state,
        sptm_uat_destroy_state,
        sptm_uat_map_table,
        sptm_uat_unmap_table,
        sptm_uat_map_begin,
        sptm_uat_map_continue,
        sptm_uat_prepare_unmap_begin,
        sptm_uat_prepare_unmap_continue,
        sptm_uat_unmap_begin,
        sptm_uat_unmap_continue,
        sptm_uat_set_context,
        sptm_uat_remove_context,
        sptm_uat_get_info,
    };
    if (!sptm.uat_configured || endpoint >= ARRAY_SIZE(handlers))
        return false;

    spin_lock(&sptm.service_lock);
    bool handled = handlers[endpoint](ctx);
    spin_unlock(&sptm.service_lock);
    return handled;
}

// Boot fills in the UAT parameters; leave UAT disabled if the device tree is unusable.
void sptm_uat_configure(u64 global_root_pa, u64 shared_l2_size, u64 handoff_pa, u64 handoff_size)
{
    sptm.uat_state_size =
        UAT_STATE_UNMAP_SEGMENTS + sptm.uat_segment_limit * sizeof(struct sptm_uat_segment);
    if ((sptm.uat_shared_l2_pa & SPTM_PAGE_MASK) || shared_l2_size != SPTM_PAGE_SIZE ||
        !sptm_valid_platform_pa(sptm.uat_shared_l2_pa, shared_l2_size) || sptm.uat_va_width < 37 ||
        sptm.uat_va_width > 48 || sptm.uat_mode > 1 || !sptm.uat_segment_limit ||
        !sptm.uat_mapping_limit || sptm.uat_state_size > SPTM_PAGE_SIZE ||
        (sptm.uat_gpu_contexts & (2 * sizeof(u64) - 1)) ||
        !sptm_valid_platform_pa(sptm.uat_gpu_contexts, 64 * 2 * sizeof(u64)) || !handoff_pa ||
        (handoff_pa & SPTM_PAGE_MASK) || handoff_size < UAT_HANDOFF_MIN_SIZE ||
        (handoff_size & SPTM_PAGE_MASK) || !sptm_valid_platform_pa(handoff_pa, handoff_size)) {
        printf("HV: refusing invalid SPTM UAT configuration\n");
        return;
    }

    // Boot-seeded UAT tables bypass RETYPE, so retire their WB aliases here.
    u64 host_pages[] = {global_root_pa, sptm.uat_shared_l2_pa};
    mmu_map_ram_pages_nc(host_pages, ARRAY_SIZE(host_pages), true);

    // GFXHandoff still exposes CUR_CTX under its historical UNK name.
    sptm.uat_handoff_pa = handoff_pa;
    write64(handoff_pa + UAT_HANDOFF_MAGIC_AP, 0);
    write64(handoff_pa + UAT_HANDOFF_MAGIC_FW, 0);
    write8(handoff_pa + UAT_HANDOFF_LOCK_AP, 0);
    write8(handoff_pa + UAT_HANDOFF_LOCK_FW, 0);
    write32(handoff_pa + UAT_HANDOFF_TURN, 0);
    write32(handoff_pa + UAT_HANDOFF_CURRENT_CTX, UINT32_MAX);
    for (u32 slot = 0; slot < UAT_HANDOFF_SLOT_COUNT; slot++) {
        u64 slot_addr = sptm_uat_handoff_slot_addr(slot);
        write64(slot_addr + UAT_HANDOFF_SLOT_STATE, 0);
        write64(slot_addr + UAT_HANDOFF_SLOT_ADDR, 0);
        write64(slot_addr + UAT_HANDOFF_SLOT_SIZE, 0);
    }
    write64(handoff_pa + UAT_HANDOFF_UNK3, 0);
    sysop("dsb osh");
    write64(handoff_pa + UAT_HANDOFF_MAGIC_AP, UAT_HANDOFF_MAGIC);
    sysop("dsb osh");
    sptm.uat_configured = true;
}
