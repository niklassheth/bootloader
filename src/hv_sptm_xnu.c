/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

static char sptm_serial_line[SPTM_SERIAL_LINE_SIZE];
static size_t sptm_serial_line_len;

static bool sptm_batch_sign_user_pointers(struct exc_info *ctx)
{
    const size_t operation_size = 24;
    size_t count = ctx->regs[1];
    if (count > SPTM_MAX_SCRATCH_ENTRIES)
        return false;

    u64 operations = 0;
    if (count) {
        operations = sptm_pointer_pa(ctx->regs[0], count * operation_size);
        if (!operations)
            return false;
    }

    u64 scratch = sptm_scratch_pa(ctx);
    if (!sptm_valid_pa(scratch, count * sizeof(u64)))
        return false;

    for (size_t index = 0; index < count; index++)
        write64(scratch + index * sizeof(u64),
                read64(operations + index * operation_size));
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static void sptm_serial_flush(void)
{
    if (!sptm_serial_line_len)
        return;

    sptm_serial_line[sptm_serial_line_len] = '\0';
    printf("HV: SPTM serial: %s\n", sptm_serial_line);
    sptm_serial_line_len = 0;
}

static void sptm_serial_putc(u8 character)
{
    if (!character || character == '\n' || character == '\r') {
        sptm_serial_flush();
        return;
    }
    if (sptm_serial_line_len == sizeof(sptm_serial_line) - 1)
        sptm_serial_flush();
    sptm_serial_line[sptm_serial_line_len++] = character;
}

static bool sptm_switch_root(struct exc_info *ctx)
{
    u64 root = ctx->regs[0] & 0x0000ffffffffffffULL;
    struct sptm_surt_root *surt = sptm_find_surt_root(root);
    u8 type = SPTM_FRAME_SUBPAGE;
    u64 asid;
    if (surt) {
        asid = surt->asid;
    } else {
        u8 *entry = sptm_frame_entry(root);
        if (!entry || !sptm_is_root_type(entry[2]))
            return false;
        type = entry[2];
        asid = read16((u64)entry + 10);
    }

    if (type == SPTM_FRAME_KERNEL_ROOT)
        msr(TTBR1_EL12, root);
    else
        msr(TTBR0_EL12, root | (asid << 48));
    sptm_publish_stage1();
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

bool sptm_handle_xnu_bootstrap(struct exc_info *ctx, u32 endpoint)
{
    switch (endpoint) {
        case 0:  /* LOCKDOWN */
        case 12: /* CONFIGURE_ROOT */
        case 15: /* FIXUPS_COMPLETE */
        case 18: /* REGISTER_EXC_RETURN */
        case 20: /* SLIDE_REGION */
        case 23: /* REG_WRITE */
        case 28: /* GUEST_EXIT */
        case 29: /* MAP_SK_DOMAIN */
        case 30: /* HIB_BEGIN */
        case 31: /* HIB_VERIFY_HASH_NON_WIRED */
        case 32: /* HIB_FINALIZE_NON_WIRED */
        case 37: /* SPTM_SYSCTL */
        case 38: /* DISABLE_KERNEL_MODE_CPA2 */
        case 39: /* SET_SHARED_REGION */
        case 48: /* PROGRAM_IRGKEY */
        case 49: /* REG_SNAPSHOT */
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            return true;
        case 1: /* RETYPE */
            return sptm_retype_frame(ctx);
        case 2: /* MAP_PAGE */
            return sptm_map_page(ctx);
        case 3: /* MAP_TABLE */
            return sptm_map_table(ctx);
        case 4: /* UNMAP_TABLE */
            return sptm_unmap_table(ctx);
        case 5: /* UPDATE_REGION */
            return sptm_update_region(ctx);
        case 6: /* UPDATE_DISJOINT */
            return sptm_update_disjoint(ctx);
        case 7: /* UNMAP_REGION */
            return sptm_unmap_region(ctx);
        case 8: /* UNMAP_DISJOINT */
            return sptm_unmap_disjoint(ctx);
        case 9: /* CONFIGURE_SHAREDREGION */
            return sptm_configure_shared_region(ctx);
        case 10: /* NEST_REGION */
            if (!sptm_nest_region(ctx, true))
                return false;
            sptm_publish_commpage_policy();
            return true;
        case 11: /* UNNEST_REGION */
            return sptm_nest_region(ctx, false);
        case 13: /* SWITCH_ROOT */
            return sptm_switch_root(ctx);
        case 14: /* REGISTER_CPU */
            if (!sptm_register_cpu(ctx->regs[0]))
                return false;
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            return true;
        case 16: /* SIGN_USER_POINTER */
        case 17: /* AUTH_USER_POINTER */
            /* Pointer-valued calls: leave x0 as the unmodified pointer. */
            return true;
        case 19: { /* CPU_ID */
            struct sptm_cpu *cpu = sptm_find_cpu(ctx->regs[0]);
            if (!cpu)
                return false;
            ctx->regs[0] = cpu->logical_id;
            return true;
        }
        case 21: /* UPDATE_DISJOINT_MULTIPAGE */
            return sptm_update_disjoint_multipage(ctx);
        case 22: /* REG_READ */
            ctx->regs[0] = 0;
            return true;
        case 24: /* GUEST_VA_TO_IPA */
            ctx->regs[0] = ~0ULL;
            return true;
        case 25: /* GUEST_STAGE1_TLBOP */
            sptm_publish_stage1();
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            return true;
        case 26: /* GUEST_STAGE2_TLBOP */
            sysop("dsb ishst");
            sysop("tlbi vmalls12e1is");
            sysop("dsb ish");
            sysop("isb");
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            return true;
        case 40: /* BATCH_SIGN_USER_POINTER */
            return sptm_batch_sign_user_pointers(ctx);
        case 41: /* SURT_ALLOC */
            return sptm_surt_update(ctx, true);
        case 42: /* SURT_FREE */
            return sptm_surt_update(ctx, false);
        case 43: /* CONDEMN_LEAF_TABLE */
            return sptm_condemn_leaf_table(ctx, true);
        case 44: /* UNCONDEMN_LEAF_TABLE */
            return sptm_condemn_leaf_table(ctx, false);
        case 45: /* SPTM_SERIAL_PUTC */
            sptm_serial_putc(ctx->regs[0]);
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            return true;
        case 46: /* SPTM_SERIAL_DISABLE */
            sptm_serial_flush();
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            return true;
        default:
            return false;
    }
}
