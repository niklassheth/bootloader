/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

struct sptm_surt_root *sptm_find_surt_root(u64 root)
{
    for (size_t index = 0; index < ARRAY_SIZE(sptm.surt_roots); index++) {
        struct sptm_surt_root *candidate = &sptm.surt_roots[index];
        if (candidate->valid && candidate->root == root)
            return candidate;
    }

    return NULL;
}

static void sptm_set_geometry(u8 attr_index, struct sptm_geometry *geometry)
{
    if (attr_index == 1 || attr_index == 5) {
        geometry->descriptor_mask = 0x0000fffffffff000ULL;
        geometry->entries_mask = 0x1ff;
        geometry->page_shift = 12;
        geometry->start_level = 0;
        geometry->page_ratio = SPTM_PAGE_SIZE >> 12;
    } else {
        geometry->descriptor_mask = SPTM_DESC_PA_MASK_16K;
        geometry->entries_mask = 0x7ff;
        geometry->page_shift = 14;
        geometry->start_level = 1;
        geometry->page_ratio = 1;
    }
}

static bool sptm_root_geometry(u64 root, struct sptm_geometry *geometry)
{
    struct sptm_surt_root *surt = sptm_find_surt_root(root);
    if (surt) {
        sptm_set_geometry(surt->attr_index, geometry);
        return true;
    }

    u8 *entry = sptm_frame_entry(root);
    if (!entry)
        return false;

    sptm_set_geometry(entry[12], geometry);
    return true;
}

static u64 sptm_walk(u64 root, u64 va, unsigned int target_level,
                     struct sptm_geometry *geometry_out)
{
    static const unsigned int shifts_16k[] = {0, 36, 25, 14};
    static const unsigned int shifts_4k[] = {39, 30, 21, 12};
    struct sptm_geometry geometry;
    u64 table = root;

    if (!sptm_root_geometry(root, &geometry) || target_level < geometry.start_level)
        return 0;

    const unsigned int *shifts = geometry.page_shift == 12 ? shifts_4k : shifts_16k;

    for (unsigned int level = geometry.start_level; level < target_level; level++) {
        u64 descriptor = read64(table + ((va >> shifts[level]) & geometry.entries_mask) * 8);
        // Do not descend into a condemned table.
        if (!(descriptor & 1) || (descriptor & BIT(2)))
            return 0;
        table = descriptor & geometry.descriptor_mask;
    }

    u64 slot = table + ((va >> shifts[target_level]) & geometry.entries_mask) * 8;
    if (geometry_out)
        *geometry_out = geometry;
    return slot;
}

static void sptm_cancel_host_rt_restore(u64 pa)
{
    for (size_t index = 0; index < sptm.host_rt_restore_count; index++) {
        if (sptm.host_rt_restore_pages[index] != pa)
            continue;

        sptm.host_rt_restore_count--;
        sptm.host_rt_restore_pages[index] =
            sptm.host_rt_restore_pages[sptm.host_rt_restore_count];
        return;
    }
}

static void sptm_queue_host_rt_restore(u64 pa)
{
    for (size_t index = 0; index < sptm.host_rt_restore_count; index++) {
        if (sptm.host_rt_restore_pages[index] == pa)
            return;
    }

    // Queue overflow leaves the page safely protected.
    if (sptm.host_rt_restore_count < ARRAY_SIZE(sptm.host_rt_restore_pages))
        sptm.host_rt_restore_pages[sptm.host_rt_restore_count++] = pa;
}

static void sptm_flush_stage1_tlb(void)
{
    // Publish page-table writes before invalidating guest translations.
    sysop("dsb sy");
    sysop("tlbi vmalle1is");
    sysop("dsb ish");
    sysop("isb");
}

void sptm_publish_stage1(void)
{
    sptm_flush_stage1_tlb();

    // Restore WB aliases only after stale RT translations are gone.
    if (sptm.host_rt_restore_count) {
        mmu_restore_ram_pages_wb(sptm.host_rt_restore_pages,
                                 sptm.host_rt_restore_count, false);
        sptm.host_rt_restore_count = 0;
    }
}

// Publish the updated mappings unless the caller deferred TLB invalidation.
static bool sptm_finish_update(struct exc_info *ctx, bool changed, bool deferred)
{
    if (changed && !deferred)
        sptm_publish_stage1();
    else
        sysop("dsb sy");
    ctx->regs[0] = deferred ? SPTM_STATUS_UPDATE_DELAYED_TLBI : SPTM_STATUS_SUCCESS;
    return true;
}

void sptm_publish_commpage_policy(void)
{
    if (sptm.commpage_policy_published)
        return;

    for (size_t index = 0; index < sptm.commpage_rw_count; index++) {
        u64 pa = sptm.commpage_rw[index];
        u64 caps = read64(pa + SPTM_COMMPAGE_CPU_CAPS64_OFFSET);
        u64 replacement = caps;
        /* Other commpage frames do not contain the CPU capability fields. */
        if (!memcmp((void *)pa, "commpage 64-bit", sizeof("commpage 64-bit") - 1)) {
            replacement &= ~SPTM_CPU_CAP_AMX_MASK;
            write64(pa + SPTM_COMMPAGE_CPU_CAPS64_OFFSET, replacement);
            clear32(pa + SPTM_COMMPAGE_CPU_CAPS32_OFFSET, SPTM_CPU_CAP_AMX_MASK);
        }
        write8(pa + SPTM_COMMPAGE_USER_TIMEBASE_OFFSET, 0);
        write8(pa + SPTM_COMMPAGE_CONT_HWCLOCK_OFFSET, 0);
        write8(pa + SPTM_COMMPAGE_HW_TPRO_OFFSET, 0);
        printf("HV: SPTM commpage fallback policy at PA 0x%lx "
               "(caps 0x%lx -> 0x%lx)\n",
               pa, caps, replacement);
    }
    sysop("dsb sy");
    sptm.commpage_policy_published = true;
}

bool sptm_retype_frame(struct exc_info *ctx)
{
    u64 pa = ctx->regs[0] & ~SPTM_PAGE_MASK;
    u8 new_type = ctx->regs[2];

    u8 *entry = sptm_frame_entry(pa);
    if (!entry)
        return false;

    u8 old_type = entry[2];
    bool entering_iommu = old_type != SPTM_FRAME_XNU_IOMMU &&
                          new_type == SPTM_FRAME_XNU_IOMMU;
    bool leaving_iommu = old_type == SPTM_FRAME_XNU_IOMMU &&
                         new_type != SPTM_FRAME_XNU_IOMMU;
    bool iommu_transition = entering_iommu || leaving_iommu;
    u64 physmap_slot = 0;
    u64 physmap_replacement = 0;

    if (iommu_transition) {
        // Keep the physmap leaf invalid during the host memory-type transition.
        u64 type_mask = (7ULL << 2) | (3ULL << 8);
        u64 type = entering_iommu ? PTE_MAIR_IDX(MAIR_IDX_NORMAL_NC) | PTE_SH_NS
                                  : PTE_MAIR_IDX(MAIR_IDX_NORMAL) | PTE_SH_OS;
        physmap_slot = sptm_walk(sptm.kernel_root, sptm_physmap_va(pa), 3, NULL);
        physmap_replacement = (read64(physmap_slot) & ~type_mask) | type;
        write64(physmap_slot, 0);
        sptm_flush_stage1_tlb();
    }

    // Remove WB aliases to avoid speculative accesses to them
    if (entering_iommu) {
        u64 host_page = pa;
        mmu_map_ram_pages_nc(&host_page, 1, true);
    }

    if (sptm_is_table_type(old_type))
        write16((u64)entry + 8, 0);
    entry[2] = new_type;

    if (new_type == SPTM_FRAME_XNU_IOMMU) {
        // Zero IOMMU tables through their NC alias.
        memset((void *)sptm_iommu_table_va(pa), 0, SPTM_PAGE_SIZE);
        sysop("dsb sy");
        write16((u64)entry + 4, 0);
    } else if (sptm_is_table_type(new_type)) {
        memset((void *)pa, 0, SPTM_PAGE_SIZE);
    }
    if (sptm_is_table_type(new_type) || new_type == SPTM_FRAME_XNU_IOMMU) {
        write16((u64)entry + 6, 0);
        write16((u64)entry + 8, 0);

        if (sptm_is_root_type(new_type)) {
            write16((u64)entry + 10, (ctx->regs[3] >> 32) & 0xffff);
            entry[12] = ctx->regs[3] & 0xff;
        }
    }

    // Defer commpage policy changes until the shared region is configured.
    if (new_type == SPTM_FRAME_XNU_COMMPAGE_RW &&
        sptm.commpage_rw_count < ARRAY_SIZE(sptm.commpage_rw)) {
        sptm.commpage_rw[sptm.commpage_rw_count++] = pa;
    }

    if (sptm_is_table_type(old_type) || sptm_is_table_type(new_type) || iommu_transition ||
        new_type == SPTM_FRAME_XNU_IOMMU || new_type == SPTM_FRAME_XNU_IO ||
        new_type == SPTM_FRAME_PROTECTED_IO || new_type == SPTM_FRAME_COPROCESSOR_RO_IO ||
        new_type == SPTM_FRAME_RESTRICTED_IO || new_type == SPTM_FRAME_RESTRICTED_IO_TELEMETRY ||
        new_type == SPTM_FRAME_SK_IO)
        sptm_publish_stage1();
    else
        sysop("dsb ishst");
    if (iommu_transition && sptm.uat_configured && sptm.uat_tlbi_at_retype)
        sptm_uat_tlbi_all();

    // Restore WB aliases only after guest and IOMMU invalidation.
    if (leaving_iommu) {
        u64 host_page = pa;
        mmu_restore_ram_pages_wb(&host_page, 1, true);
    }
    if (iommu_transition) {
        write64(physmap_slot, physmap_replacement);
        sptm_publish_stage1();
    }
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

bool sptm_configure_shared_region(struct exc_info *ctx)
{
    u64 pa = ctx->regs[0] & ~SPTM_PAGE_MASK;
    u8 *entry = sptm_frame_entry(pa);
    if (!entry)
        return false;

    entry[2] = SPTM_FRAME_SHARED_ROOT;
    write16((u64)entry + 10, 0);
    entry[12] = 0;
    sysop("dsb ishst");
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

bool sptm_map_page(struct exc_info *ctx)
{
    u64 root = ctx->regs[0];
    u64 va = ctx->regs[1];
    u64 pte = ctx->regs[2];
    struct sptm_geometry geometry;
    u64 slot = sptm_walk(root, va, 3, &geometry);
    if (!slot) {
        ctx->regs[0] = SPTM_STATUS_TABLE_NOT_PRESENT;
        return true;
    }

    u64 old = read64(slot);
    u64 scratch = sptm_scratch_pa(ctx);
    write64(scratch, old);
    write64(scratch + sizeof(u64), sptm_physmap_va(slot));

    if ((old & 1) && ((old & geometry.descriptor_mask) != (pte & geometry.descriptor_mask))) {
        sysop("dsb sy");
        ctx->regs[0] = SPTM_STATUS_MAP_PADDR_CONFLICT;
        return true;
    }

    if (!(old & 1) && (pte & 1)) {
        sptm_adjust_mapping_ref(pte, geometry.descriptor_mask, 1);
        sptm_adjust_table_valid(slot, 1);
    }

    ctx->regs[0] = old & 1 ? SPTM_STATUS_MAP_VALID : SPTM_STATUS_SUCCESS;
    write64(slot, pte);

    if ((old & 1) && old != pte)
        sptm_publish_stage1();
    else
        sysop("dsb sy");

    return true;
}

bool sptm_map_table(struct exc_info *ctx)
{
    u64 root = ctx->regs[0];
    u64 va = ctx->regs[1];
    unsigned int target_level = ctx->regs[2];
    u64 tte = ctx->regs[3];

    struct sptm_geometry geometry;
    u64 slot = sptm_walk(root, va, target_level, &geometry);
    if (!slot) {
        ctx->regs[0] = SPTM_STATUS_TABLE_NOT_PRESENT;
        return true;
    }

    slot &= ~(geometry.page_ratio * sizeof(u64) - 1);
    for (size_t index = 0; index < geometry.page_ratio; index++) {
        if (read64(slot + index * sizeof(u64)) & 1) {
            ctx->regs[0] = SPTM_STATUS_TABLE_PRESENT;
            return true;
        }
    }

    u64 child = tte & geometry.descriptor_mask;
    u64 attrs = tte & ~geometry.descriptor_mask;
    sptm_adjust_parent_links(child, 1);
    for (size_t index = 0; index < geometry.page_ratio; index++) {
        u64 entry = slot + index * sizeof(u64);
        u64 replacement =
            ((child + index * BIT(geometry.page_shift)) & geometry.descriptor_mask) | attrs;
        write64(entry, replacement);
    }
    sysop("dsb sy");
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

bool sptm_unmap_table(struct exc_info *ctx)
{
    struct sptm_geometry geometry;
    u64 slot = sptm_walk(ctx->regs[0], ctx->regs[1], ctx->regs[2], &geometry);
    slot &= ~(geometry.page_ratio * sizeof(u64) - 1);
    sptm_adjust_parent_links(read64(slot) & geometry.descriptor_mask, -1);
    for (size_t index = 0; index < geometry.page_ratio; index++)
        write64(slot + index * sizeof(u64), 0);
    sptm_publish_stage1();
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

// Merge a template into a leaf, recording the displaced PTE in scratch.
static void sptm_update_leaf(u64 slot, u64 template, u64 mask, u64 scratch)
{
    u64 old = read64(slot);
    write64(scratch, old);
    write64(slot, (old & ~mask) | (template & mask));
}

// Clear a leaf if it is mapped, recording the displaced PTE in scratch.
static void sptm_unmap_leaf(u64 slot, u64 descriptor_mask, u64 scratch, bool *changed)
{
    u64 old = slot ? read64(slot) : 0;
    write64(scratch, old);
    if (!(old & 1))
        return;
    sptm_adjust_mapping_ref(old, descriptor_mask, -1);
    sptm_adjust_table_valid(slot, -1);
    write64(slot, 0);
    *changed = true;
}

static u64 sptm_update_mask(u64 flags)
{
    u64 mask = 0;

    if (flags & BIT(0))
        mask |= BIT(58);
    if (flags & BIT(1))
        mask |= (3ULL << 6) | BIT(53) | BIT(54) | BIT(59);
    if (flags & BIT(2))
        mask |= BIT(11);
    if (flags & BIT(3))
        mask |= BIT(10);
    if (flags & BIT(4))
        mask |= 3ULL << 8;
    if (flags & BIT(5))
        mask |= 7ULL << 2;

    return mask;
}

bool sptm_update_region(struct exc_info *ctx)
{
    u64 root = ctx->regs[0];
    u64 start = ctx->regs[1];
    size_t count = ctx->regs[2];
    u64 flags = ctx->regs[4];

    struct sptm_geometry geometry;
    if (!sptm_root_geometry(root, &geometry))
        return false;

    u64 page_size = BIT(geometry.page_shift);
    u64 templates = sptm_pointer_pa(ctx->regs[3]);
    u64 scratch = sptm_scratch_pa(ctx);
    u64 mask = sptm_update_mask(flags);
    for (size_t index = 0; index < count; index++)
        sptm_update_leaf(sptm_walk(root, start + index * page_size, 3, NULL),
                         read64(templates + index * sizeof(u64)), mask,
                         scratch + index * sizeof(u64));

    // Do not invalidate a partial deferred alias batch.
    return sptm_finish_update(ctx, count, flags & BIT(8));
}

bool sptm_update_disjoint(struct exc_info *ctx)
{
    size_t count = ctx->regs[2];
    u64 flags = ctx->regs[3];

    u64 ops = sptm_pointer_pa(ctx->regs[1]);
    u64 scratch = sptm_scratch_pa(ctx);
    u64 mask = sptm_update_mask(flags);
    for (size_t index = 0; index < count; index++) {
        u64 op = ops + index * 24;
        sptm_update_leaf(sptm_walk(read64(op), read64(op + 8), 3, NULL), read64(op + 16), mask,
                         scratch + index * sizeof(u64));
    }

    return sptm_finish_update(ctx, count, flags & BIT(8));
}

bool sptm_update_disjoint_multipage(struct exc_info *ctx)
{
    size_t slots = ctx->regs[1];
    u64 stream = sptm_pointer_pa(ctx->regs[0]);
    u64 scratch = sptm_scratch_pa(ctx);
    size_t used;

    if (!cpu_features->apple_sysregs_unlocked) {
        // Mirror RT/WCOMB memory types in m1n1, restoring WB only after TLBI. The
        // last transition of each page in the stream wins.
        u64 *protect = (u64 *)scratch;
        size_t protect_count = 0;
        for (used = 0; used < slots; used += 1 + read32(stream + used * 24 + 16)) {
            u64 header = stream + used * 24;
            u64 paddr = read64(header);
            if ((read32(header + 20) & BIT(9)) || !sptm_is_managed_range(paddr, 1))
                continue;

            for (size_t index = 0; index < protect_count; index++) {
                if (protect[index] == paddr) {
                    protect[index] = protect[--protect_count];
                    break;
                }
            }
            if ((read64(header + 8) & (7ULL << 2)) == PTE_MAIR_IDX(1)) {
                sptm_cancel_host_rt_restore(paddr);
                protect[protect_count++] = paddr;
            } else {
                sptm_queue_host_rt_restore(paddr);
            }
        }
        mmu_map_ram_pages_nc(protect, protect_count, false);
    }

    bool changed = false;
    bool deferred = false;
    size_t scratch_index = 0;
    used = 0;

    while (used < slots) {
        u64 header = stream + used * 24;
        u64 paddr = read64(header);
        u64 papt_template = read64(header + 8);
        size_t inner_count = read32(header + 16);
        u64 options = read32(header + 20);
        u64 mask = sptm_update_mask(options);
        used++;

        deferred |= options & BIT(8);

        // Keep the physmap memory type in sync unless SKIP_PAPT is set.
        if (!(options & BIT(9)) && sptm_is_managed_range(paddr, 1)) {
            u64 physmap_slot = sptm_walk(sptm.kernel_root, sptm_physmap_va(paddr), 3, NULL);
            u64 physmap_mask = (3ULL << 8) | (7ULL << 2);
            write64(physmap_slot,
                    (read64(physmap_slot) & ~physmap_mask) | (papt_template & physmap_mask));
            changed = true;
        }

        for (size_t index = 0; index < inner_count; index++, scratch_index++) {
            u64 op = stream + (used + index) * 24;
            sptm_update_leaf(sptm_walk(read64(op), read64(op + 8), 3, NULL), read64(op + 16), mask,
                             scratch + scratch_index * sizeof(u64));
            changed = true;
        }
        used += inner_count;
    }

    return sptm_finish_update(ctx, changed, deferred);
}

bool sptm_unmap_region(struct exc_info *ctx)
{
    u64 root = ctx->regs[0];
    u64 start = ctx->regs[1];
    size_t count = ctx->regs[2];
    u64 options = ctx->regs[3];

    struct sptm_geometry geometry;
    if (!sptm_root_geometry(root, &geometry))
        return false;

    u64 page_size = BIT(geometry.page_shift);
    u64 scratch = sptm_scratch_pa(ctx);
    bool changed = false;
    for (size_t index = 0; index < count; index++)
        sptm_unmap_leaf(sptm_walk(root, start + index * page_size, 3, NULL),
                        geometry.descriptor_mask, scratch + index * sizeof(u64), &changed);

    return sptm_finish_update(ctx, changed, changed && (options & BIT(8)));
}

bool sptm_unmap_disjoint(struct exc_info *ctx)
{
    size_t count = ctx->regs[2];
    u64 ops = sptm_pointer_pa(ctx->regs[1]);
    u64 scratch = sptm_scratch_pa(ctx);
    bool changed = false;
    for (size_t index = 0; index < count; index++) {
        u64 op = ops + index * 24;
        struct sptm_geometry geometry;
        u64 slot = sptm_walk(read64(op), read64(op + 8), 3, &geometry);
        sptm_unmap_leaf(slot, geometry.descriptor_mask, scratch + index * sizeof(u64), &changed);
    }

    return sptm_finish_update(ctx, changed, false);
}

bool sptm_nest_region(struct exc_info *ctx, bool nest)
{
    u64 user_root = ctx->regs[0];
    u64 shared_root = ctx->regs[1];
    u64 start = ctx->regs[2];
    u64 count = ctx->regs[3];
    struct sptm_geometry geometry;
    if (!sptm_root_geometry(user_root, &geometry))
        return false;

    // Nesting copies whole L2 entries from the shared root into the user root.
    u64 bytes = count << geometry.page_shift;
    u8 l2_shift = geometry.page_shift == 12 ? 21 : 25;
    u64 first_block = start >> l2_shift;
    u64 iterations = bytes ? ((start + bytes - 1) >> l2_shift) - first_block + 1 : 0;
    bool changed = false;
    for (u64 index = 0; index < iterations; index++) {
        u64 va = (first_block + index) << l2_shift;
        u64 user_slot = sptm_walk(user_root, va, 2, NULL);
        u64 value = nest ? read64(sptm_walk(shared_root, va, 2, NULL)) : 0;
        if (read64(user_slot) != value) {
            write64(user_slot, value);
            changed = true;
        }
    }

    return sptm_finish_update(ctx, changed, false);
}

bool sptm_surt_update(struct exc_info *ctx, bool allocate)
{
    // Subpage roots are 128-byte slots inside a SUBPAGE frame.
    u64 root = (ctx->regs[0] & ~SPTM_PAGE_MASK) + ctx->regs[1] * 128;
    struct sptm_surt_root *surt = sptm_find_surt_root(root);

    if (allocate) {
        if (!surt) {
            for (size_t index = 0; index < ARRAY_SIZE(sptm.surt_roots); index++) {
                if (!sptm.surt_roots[index].valid) {
                    surt = &sptm.surt_roots[index];
                    break;
                }
            }
        }
        if (!surt)
            return false;

        surt->root = root;
        surt->attr_index = ctx->regs[2];
        surt->asid = ctx->regs[4];
        surt->valid = true;
    } else if (surt) {
        memset(surt, 0, sizeof(*surt));
    }

    memset((void *)root, 0, 128);
    sysop("dsb sy");
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

bool sptm_condemn_leaf_table(struct exc_info *ctx, bool condemn)
{
    u64 slot = sptm_walk(ctx->regs[0], ctx->regs[1], 2, NULL);
    u64 value = slot ? read64(slot) : 0;
    if (!(value & 1)) {
        ctx->regs[0] = SPTM_STATUS_TABLE_NOT_PRESENT;
        return true;
    }

    // Only one caller may condemn a table; clearing is idempotent.
    if (!!(value & BIT(2)) == condemn) {
        ctx->regs[0] = condemn ? SPTM_STATUS_TABLE_NOT_PRESENT : SPTM_STATUS_SUCCESS;
        return true;
    }

    write64(slot, value ^ BIT(2));
    sptm_publish_stage1();
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}
