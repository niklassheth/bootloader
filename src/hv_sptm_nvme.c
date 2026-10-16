/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

static struct sptm_nvme_command *sptm_nvme_command(u32 cid)
{
    return &((struct sptm_nvme_command *)sptm.nvme.command_state)[cid];
}

static u64 sptm_nvme_tcb(u32 qid, u32 cid)
{
    return (qid ? sptm.nvme.io_tcbs : sptm.nvme.admin_tcbs) + cid * SPTM_NVME_TCB_SIZE;
}

static bool sptm_nvme_map_pages(struct exc_info *ctx)
{
    u32 qid = ctx->regs[0];
    u32 cid = ctx->regs[1];
    u64 segment_pa = sptm_pointer_pa(ctx->regs[3]);
    size_t count = ctx->regs[4];
    struct sptm_nvme_command *command = sptm_nvme_command(cid);
    u8 template[SPTM_NVME_TCB_SIZE];

    memcpy(template, (void *)sptm_pointer_pa(ctx->regs[2]), sizeof(template));
    u8 dma_flags = template[0x01];
    template[0x02] = cid;
    write64((u64)template + 0x18, 0);
    write64((u64)template + 0x20, 0);

    // Pin every DMA target for the lifetime of the command.
    for (size_t index = 0; index < count; index++)
        sptm_adjust_data_ref(read64(segment_pa + index * sizeof(u64)), dma_flags & BIT(0), 1);

    // PRP1 is the first target; PRP2 is the second target or a list of the rest.
    u64 first = count ? read64(segment_pa) : 0;
    u64 prp_aux = 0;
    if (count == 2) {
        prp_aux = read64(segment_pa + sizeof(u64));
    } else if (count > 2) {
        prp_aux = sptm.nvme.prp_scratch + cid * SPTM_NVME_PRP_SLOT_SIZE;
        memcpy((void *)prp_aux, (void *)(segment_pa + sizeof(u64)), (count - 1) * sizeof(u64));
        if (sptm.nvme.prp_flush_wa && count > 0x10) {
            size_t flush_size =
                count < 0x100 ? (count * sizeof(u64) + 0x7f) & 0x1f80 : SPTM_NVME_PRP_PAGE_SIZE;
            sysop("dsb sy");
            dc_civac_range((void *)prp_aux, flush_size);
        }
    }
    write64((u64)template + 0x18, first);
    write64((u64)template + 0x20, prp_aux);

    memset(command, 0, sizeof(*command));
    command->prp_aux = prp_aux;
    command->count = count;
    command->dma_flags = dma_flags;
    command->first = first;

    memcpy((void *)sptm_nvme_tcb(qid, cid), template, sizeof(template));
    sysop("dsb sy");
    return true;
}

static bool sptm_nvme_tl_idle(struct sptm_nvme_command *command, bool retry)
{
    u32 directions = command->dma_flags & 3;
    if (!sptm.nvme.tl_wa || !directions)
        return true;

    if (!retry) {
        u32 mask = ((directions & 1) ? 0x800000 : 0) | ((directions & 2) ? 0x80 : 0);
        u32 control = ((directions & 1) ? 0x400000 : 0) | ((directions & 2) ? 0x40 : 0);
        write32(sptm.nvme.tl_mask + 0x4, mask);
        write32(sptm.nvme.tl_mask + 0xc, mask);
        write32(sptm.nvme.tl_control + 0x4, control);
        command->drained_slots = 0;
        sysop("dsb sy");
    }

    u64 all_slots = BIT(sptm.nvme.tl_slots) - 1;
    for (size_t poll = 0; poll < 250; poll++) {
        for (u32 slot = 0; slot < sptm.nvme.tl_slots; slot++) {
            if (command->drained_slots & BIT(slot))
                continue;
            u32 status = read32(sptm.nvme.tl_status + slot * 0x10000);
            bool writes_done = !(directions & 1) || (status & 0xff) == 0xff;
            bool reads_done = !(directions & 2) || (status & 0x7f0000) == 0x7f0000;
            if (writes_done && reads_done)
                command->drained_slots |= BIT(slot);
        }
        if (command->drained_slots == all_slots) {
            write32(sptm.nvme.tl_mask + 0x4, 0x800080);
            write32(sptm.nvme.tl_mask + 0xc, 0x800080);
            write32(sptm.nvme.tl_control + 0x4, 0x400040);
            command->drained_slots = 0;
            sysop("dsb sy");
            return true;
        }
        udelay(1);
    }

    return false;
}

static u64 sptm_nvme_command_target(const struct sptm_nvme_command *command, size_t index)
{
    if (!index)
        return command->first;
    if (command->count == 2)
        return command->prp_aux;
    return read64(command->prp_aux + (index - 1) * sizeof(u64));
}

static bool sptm_nvme_unmap_pages(struct exc_info *ctx)
{
    u32 qid = ctx->regs[0];
    u32 cid = ctx->regs[1];
    // A retry follows an earlier call that returned 0 while the TL drained.
    bool retry = ctx->regs[2];
    struct sptm_nvme_command *command = sptm_nvme_command(cid);

    if (!retry) {
        if (sptm.nvme.vdma_wa && (read32(sptm.nvme.vdma_status + 0x20000 + cid * 0x20) & 0x300))
            return false;
        memset((void *)sptm_nvme_tcb(qid, cid), 0, SPTM_NVME_TCB_SIZE);
        sysop("dsb sy");
        write32(sptm.nvme.nvmmu + NVMMU_TCB_INVALIDATE, cid);
        sysop("dsb sy");
        u32 packed_status = read32(sptm.nvme.nvmmu + NVMMU_TCB_STATUS + (cid / 4) * sizeof(u32));
        if ((packed_status >> ((cid % 4) * 5)) & 0xf)
            return false;
    }

    if (!sptm_nvme_tl_idle(command, retry)) {
        ctx->regs[0] = 0;
        return true;
    }

    for (size_t index = 0; index < command->count; index++)
        sptm_adjust_data_ref(sptm_nvme_command_target(command, index), command->dma_flags & BIT(0),
                             -1);

    if (command->count > 2) {
        memset((void *)command->prp_aux, 0, SPTM_NVME_PRP_SLOT_SIZE);
        // Seems weird but the documentation says so...
        if (sptm.nvme.prp_flush_wa && command->count > 0xff) {
            sysop("dsb sy");
            dc_civac_range((void *)(command->prp_aux + SPTM_NVME_PRP_SLOT_SIZE),
                           SPTM_NVME_PRP_SLOT_SIZE);
        }
    }
    sysop("dsb sy");
    memset(command, 0, sizeof(*command));
    ctx->regs[0] = 1;
    return true;
}

static void sptm_nvme_mmio_write32(u64 address, u32 value)
{
    write32(address, value);
    sysop("dsb sy");
}

static void sptm_nvme_mmio_write64_lo_hi(u64 address, u64 value)
{
    sptm_nvme_mmio_write32(address, value);
    sptm_nvme_mmio_write32(address + sizeof(u32), value >> 32);
}

bool sptm_handle_nvme(struct exc_info *ctx, u32 endpoint)
{
    struct sptm_nvme *nvme = &sptm.nvme;
    u64 *args = ctx->regs;
    bool handled = true;
    spin_lock(&sptm.service_lock);
    switch (endpoint) {
        case 0:
            sysop("dsb sy");
            sptm_nvme_mmio_write64_lo_hi(nvme->nvmmu + NVMMU_ADMIN_TCB_BASE, nvme->admin_tcbs);
            sptm_nvme_mmio_write64_lo_hi(nvme->nvmmu + NVMMU_IO_TCB_BASE, nvme->io_tcbs);
            sptm_nvme_mmio_write32(nvme->nvmmu + NVMMU_QUEUE_COUNT, nvme->queue_entries - 1);
            break;
        case 1:
            handled = sptm_nvme_map_pages(ctx);
            break;
        case 2:
            // UNMAP_PAGES reports its own completion state in x0.
            handled = sptm_nvme_unmap_pages(ctx);
            spin_unlock(&sptm.service_lock);
            return handled;
        case 4:
            // Admin queues: submission base and size, then completion base and size.
            sptm_nvme_mmio_write32(nvme->queue_bar + NVME_AQA, args[1] | (args[3] << 16));
            sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_ASQ, args[0]);
            sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_ACQ, args[2]);
            break;
        case 5:
            sptm_nvme_mmio_write32(nvme->queue_bar + NVME_IOQA, args[0] | (args[1] << 16));
            break;
        case 6:
            sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_IOSQ, args[0]);
            break;
        case 7:
            sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_IOCQ, args[0]);
            break;
        case 8:
            write32(nvme->sha_base, args[0] >> SPTM_PAGE_SHIFT);
            write32(nvme->sha_base + 4, args[2]);
            sysop("dsb sy");
            break;
    }
    spin_unlock(&sptm.service_lock);
    if (handled)
        ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return handled;
}
