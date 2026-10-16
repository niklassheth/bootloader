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

// XNU reads a page table's valid-entry count to decide when the table is empty.
void sptm_adjust_table_valid(u64 slot, int delta)
{
    u64 count = (u64)sptm_frame_entry(slot & ~SPTM_PAGE_MASK) + 8;
    write16(count, read16(count) + delta);
}
