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

// Every frame-table update runs under sptm.service_lock.
void sptm_adjust_frame_u16(u8 *entry, size_t offset, int delta)
{
    write16((u64)entry + offset, read16((u64)entry + offset) + delta);
}

// DMA reservations and CPU mappings count against the frame's RO or WX reference count.
void sptm_adjust_data_ref(u64 pa, bool writable, int delta)
{
    u8 *entry = sptm_frame_entry(pa & ~SPTM_PAGE_MASK);
    if (!entry)
        return; /* MMIO and unmanaged carveouts have no frame entry. */

    size_t offset = writable ? 12 : 8;
    write32((u64)entry + offset, read32((u64)entry + offset) + delta);
}

void sptm_adjust_data_range(u64 pa, u64 size, bool writable, int delta)
{
    for (u64 frame = pa & ~SPTM_PAGE_MASK; frame < pa + size; frame += SPTM_PAGE_SIZE)
        sptm_adjust_data_ref(frame, writable, delta);
}

void sptm_adjust_iommu_use(u64 pa, int delta)
{
    u8 *entry = sptm_frame_entry(pa & ~SPTM_PAGE_MASK);
    if (entry) /* MMIO and unmanaged carveouts have no frame entry. */
        sptm_adjust_frame_u16(entry, 0, delta);
}

void sptm_adjust_table_valid(u64 slot, int delta)
{
    sptm_adjust_frame_u16(sptm_frame_entry(slot & ~SPTM_PAGE_MASK), 8, delta);
}

void sptm_adjust_parent_links(u64 child, int delta)
{
    sptm_adjust_frame_u16(sptm_frame_entry(child & ~SPTM_PAGE_MASK), 6, delta);
}

// Every CPU mapping counts as a read-only reference.
void sptm_adjust_mapping_ref(u64 pte, u64 descriptor_mask, int delta)
{
    if (pte & 1)
        sptm_adjust_data_ref(pte & descriptor_mask, false, delta);
}
