/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef HV_SPTM_H
#define HV_SPTM_H

#include "exception.h"
#include "types.h"

u64 hv_sptm_init(u64 guest_adt, u64 cons_ops, u64 page_shift_const, u64 xnu_text);
bool hv_sptm_handle_hvc(struct exc_info *ctx, u32 immediate);
void sptm_boot_prepare_start(void **entry, u64 regs[4], bool secondary);
bool hv_handle_clpc_hvc(struct exc_info *ctx, u32 immediate);

#endif
