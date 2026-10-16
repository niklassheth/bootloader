/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

bool sptm_is_table_type(u8 type)
{
    switch (type) {
        case SPTM_FRAME_KERNEL_ROOT:
        case SPTM_FRAME_PAGE_TABLE:
        case SPTM_FRAME_USER_ROOT:
        case SPTM_FRAME_SHARED_ROOT:
        case SPTM_FRAME_XNU_ROOT:
        case SPTM_FRAME_SHARED_PT:
        case SPTM_FRAME_ROZONE_PT:
        case SPTM_FRAME_COMMPAGE_PT:
        case SPTM_FRAME_STAGE2_ROOT:
        case SPTM_FRAME_STAGE2_PT:
        case SPTM_FRAME_SUBPAGE:
            return true;
        default:
            return false;
    }
}

bool sptm_is_root_type(u8 type)
{
    switch (type) {
        case SPTM_FRAME_KERNEL_ROOT:
        case SPTM_FRAME_USER_ROOT:
        case SPTM_FRAME_SHARED_ROOT:
        case SPTM_FRAME_XNU_ROOT:
        case SPTM_FRAME_STAGE2_ROOT:
        case SPTM_FRAME_SUBPAGE:
            return true;
        default:
            return false;
    }
}

u8 *sptm_frame_entry(u64 pa)
{
    if ((pa & SPTM_PAGE_MASK) || !sptm_is_managed_range(pa, SPTM_PAGE_SIZE))
        return NULL;

    u64 index = (pa - sptm.managed_start) >> SPTM_PAGE_SHIFT;
    return (u8 *)(sptm.frame_table_pa + index * 16);
}

// Callers have already resolved pa's frame entry.
u32 *sptm_external_ref_entry(u64 pa)
{
    u64 index = (pa - sptm.managed_start) >> SPTM_PAGE_SHIFT;
    return (u32 *)(sptm.external_ref_table_pa + index * (2 * sizeof(u32)));
}

bool sptm_adjust_frame_u16(u8 *entry, size_t offset, int delta)
{
    spin_lock(&sptm.frame_lock);
    u16 value = read16((u64)entry + offset);
    if ((delta < 0 && value < (u16)-delta) || (delta > 0 && value > UINT16_MAX - (u16)delta)) {
        spin_unlock(&sptm.frame_lock);
        return false;
    }
    write16((u64)entry + offset, value + delta);
    spin_unlock(&sptm.frame_lock);
    return true;
}

bool sptm_adjust_data_ref(u64 pa, bool writable, int delta)
{
    u8 *entry = sptm_frame_entry(pa & ~SPTM_PAGE_MASK);
    if (!entry)
        return true; /* MMIO and unmanaged carveouts have no frame entry. */
    u32 *external = sptm_external_ref_entry(pa & ~SPTM_PAGE_MASK);

    spin_lock(&sptm.frame_lock);
    size_t offset = writable ? 12 : 8;
    size_t external_index = writable ? 1 : 0;
    u32 value = read32((u64)entry + offset);
    u32 reserved = external[external_index];
    if (value < reserved || (delta < 0 && value < (u32)-delta) ||
        (delta < 0 && reserved < (u32)-delta) || (delta > 0 && value > UINT32_MAX - (u32)delta) ||
        (delta > 0 && reserved > UINT32_MAX - (u32)delta)) {
        spin_unlock(&sptm.frame_lock);
        return false;
    }
    write32((u64)entry + offset, value + delta);
    external[external_index] = reserved + delta;
    spin_unlock(&sptm.frame_lock);
    return true;
}

bool sptm_adjust_data_range(u64 pa, u64 size, bool writable, int delta)
{
    u64 start = pa & ~SPTM_PAGE_MASK;
    u64 end = ALIGN_UP(pa + size, SPTM_PAGE_SIZE);
    u64 frame = start;
    for (; frame < end; frame += SPTM_PAGE_SIZE) {
        if (sptm_adjust_data_ref(frame, writable, delta))
            continue;
        for (u64 rollback = start; rollback < frame; rollback += SPTM_PAGE_SIZE)
            sptm_adjust_data_ref(rollback, writable, -delta);
        return false;
    }
    return true;
}

bool sptm_adjust_iommu_use(u64 pa, int delta)
{
    u8 *entry = sptm_frame_entry(pa & ~SPTM_PAGE_MASK);
    if (!entry)
        return true; /* MMIO and unmanaged carveouts have no frame entry. */
    return sptm_adjust_frame_u16(entry, 0, delta);
}

bool sptm_adjust_table_valid(u64 slot, int delta)
{
    return sptm_adjust_frame_u16(sptm_frame_entry(slot & ~SPTM_PAGE_MASK), 8, delta);
}

bool sptm_adjust_parent_links(u64 child, int delta)
{
    return sptm_adjust_frame_u16(sptm_frame_entry(child & ~SPTM_PAGE_MASK), 6, delta);
}

bool sptm_adjust_mapping_ref(u64 pte, u64 descriptor_mask, int delta)
{
    if (!(pte & 1))
        return true;

    u64 pa = (pte & descriptor_mask) & ~SPTM_PAGE_MASK;
    u8 *entry = sptm_frame_entry(pa);
    if (!entry)
        return true; /* MMIO and unmanaged carveouts have no frame entry. */

    u32 *external = sptm_external_ref_entry(pa);

    // Track all CPU mappings in ro_refcount; DMA reservations remain external.
    spin_lock(&sptm.frame_lock);
    u32 value = read32((u64)entry + 8);
    if (value < external[0] || read32((u64)entry + 12) < external[1] ||
        (delta < 0 && value - external[0] < (u32)-delta) ||
        (delta > 0 && value > UINT32_MAX - (u32)delta)) {
        spin_unlock(&sptm.frame_lock);
        return false;
    }
    write32((u64)entry + 8, value + delta);
    spin_unlock(&sptm.frame_lock);
    return true;
}
