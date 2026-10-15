/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

static bool sptm_nvme_xnu_default_range(u64 pa, size_t size)
{
    if (!size || !sptm_is_managed_range(pa, size))
        return false;

    u64 first = pa & ~SPTM_PAGE_MASK;
    u64 last = (pa + size - 1) & ~SPTM_PAGE_MASK;
    for (u64 frame = first;; frame += SPTM_PAGE_SIZE) {
        u8 *entry = sptm_frame_entry(frame);
        if (!entry || entry[2] != SPTM_FRAME_DEFAULT)
            return false;
        if (frame == last)
            return true;
    }
}

static bool sptm_nvme_range_allowed(u64 pa, size_t size)
{
    return (pa >= sptm.nvme.trusted_start && pa < sptm.nvme.trusted_end &&
            size <= sptm.nvme.trusted_end - pa) ||
           (pa >= sptm.nvme.xnu_managed_start && pa < sptm.nvme.xnu_managed_end &&
            size <= sptm.nvme.xnu_managed_end - pa) ||
           // Static kernelcache frames before top_of_kernel_data may back late I/O.
           sptm_nvme_xnu_default_range(pa, size);
}

static bool sptm_nvme_queue_allowed(u64 pa, size_t size)
{
    return !(pa & SPTM_NVME_PRP_PAGE_MASK) && sptm_nvme_range_allowed(pa, size);
}

static struct sptm_nvme_command *sptm_nvme_command(u32 cid)
{
    if (cid >= sptm.nvme.queue_entries)
        return NULL;
    return (struct sptm_nvme_command *)(sptm.nvme.command_state + cid * SPTM_NVME_COMMAND_SIZE);
}

static u64 sptm_nvme_tcb(u32 qid, u32 cid)
{
    u64 base;

    if (qid == 0)
        base = sptm.nvme.admin_tcbs;
    else if (qid == 1)
        base = sptm.nvme.io_tcbs;
    else
        return 0;

    return base + cid * SPTM_NVME_TCB_SIZE;
}

static bool sptm_nvme_map_pages(struct exc_info *ctx)
{
    u32 qid = ctx->regs[0];
    u32 cid = ctx->regs[1];
    u64 template_pointer = ctx->regs[2];
    u64 segment_pointer = ctx->regs[3];
    size_t count = ctx->regs[4];
    u8 template[SPTM_NVME_TCB_SIZE];
    struct sptm_nvme_command *command = sptm_nvme_command(cid);
    u64 tcb_pa = sptm_nvme_tcb(qid, cid);
    u64 template_pa = sptm_pointer_pa(template_pointer, sizeof(template));

    if (!sptm.nvme.coastguard_enabled || !command || !tcb_pa || count > SPTM_NVME_MAX_PRPS ||
        !template_pa || command->state)
        return false;

    memcpy(template, (void *)template_pa, sizeof(template));
    u32 length = read32((u64)template + 0x04);
    u8 dma_flags = template[0x01];
    u64 template_prp1 = read64((u64)template + 0x18);
    u64 template_prp2 = read64((u64)template + 0x20);
    write64((u64)template + 0x18, 0);
    write64((u64)template + 0x20, 0);
    template[0x02] = cid;

    u64 segment_pa = 0;
    u64 first = 0;
    if (count) {
        if (!segment_pointer)
            return false;
        size_t segment_size = count * sizeof(u64);
        segment_pa = sptm_pointer_pa(segment_pointer, segment_size);
        if (!segment_pa || (segment_pa & SPTM_PAGE_MASK) + segment_size > SPTM_PAGE_SIZE)
            return false;

        first = read64(segment_pa);
        if (count == 1 && (first & SPTM_NVME_PRP_PAGE_MASK))
            return false;

        u32 expected_length = (first & SPTM_NVME_PRP_PAGE_MASK) ? count - 2 : count - 1;
        if (length != expected_length || (template_prp1 && template_prp1 != first))
            return false;
        if (count == 2) {
            u64 second = read64(segment_pa + sizeof(u64));
            if (template_prp2 && template_prp2 != second)
                return false;
        }

        for (size_t index = 0; index < count; index++) {
            u64 target = read64(segment_pa + index * sizeof(u64));
            u64 page = target & ~SPTM_NVME_PRP_PAGE_MASK;
            if ((index && target != page) ||
                !sptm_nvme_range_allowed(page, SPTM_NVME_PRP_PAGE_SIZE))
                return false;
        }
    } else if (length) {
        return false;
    }

    bool writable = dma_flags & BIT(0);
    size_t acquired = 0;
    for (; acquired < count; acquired++) {
        u64 target = read64(segment_pa + acquired * sizeof(u64));
        if (sptm_adjust_data_ref(target, writable, 1))
            continue;
        for (size_t rollback = 0; rollback < acquired; rollback++) {
            u64 rollback_target = read64(segment_pa + rollback * sizeof(u64));
            sptm_adjust_data_ref(rollback_target, writable, -1);
        }
        return false;
    }

    u64 prp_list_pa = 0;
    u64 second = 0;
    if (count)
        write64((u64)template + 0x18, first);
    if (count == 2) {
        second = read64(segment_pa + sizeof(u64));
        write64((u64)template + 0x20, second);
    } else if (count > 2) {
        prp_list_pa = sptm.nvme.prp_scratch + cid * SPTM_NVME_PRP_SLOT_SIZE;
        for (size_t index = 1; index < count; index++)
            write64(prp_list_pa + (index - 1) * sizeof(u64),
                    read64(segment_pa + index * sizeof(u64)));

        if (sptm.nvme.prp_flush_wa && count > 0x10) {
            size_t flush_size =
                count < 0x100 ? (count * sizeof(u64) + 0x7f) & 0x1f80 : SPTM_NVME_PRP_PAGE_SIZE;
            sysop("dsb sy");
            dc_civac_range((void *)prp_list_pa, flush_size);
        }
        write64((u64)template + 0x20, prp_list_pa);
    }

    memset(command, 0, sizeof(*command));
    command->prp_aux = count == 2 ? second : prp_list_pa;
    command->count = count;
    command->qid = qid;
    command->dma_flags = dma_flags;
    command->state = 1;
    command->first = first;

    memcpy((void *)tcb_pa, template, sizeof(template));
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
    bool retry = ctx->regs[2];
    struct sptm_nvme_command *command = sptm_nvme_command(cid);
    u64 tcb_pa = sptm_nvme_tcb(qid, cid);

    if (ctx->regs[2] > 1 || !command || !tcb_pa || !command->state || command->qid != qid ||
        (retry && (!sptm.nvme.tl_wa || command->state != 2)) || (!retry && command->state != 1))
        return false;

    if (!retry) {
        if (sptm.nvme.vdma_wa) {
            u32 status = read32(sptm.nvme.vdma_status + 0x20000 + cid * 0x20);
            if (status & 0x300)
                return false;
        }
        memset((void *)tcb_pa, 0, SPTM_NVME_TCB_SIZE);
        sysop("dsb sy");
        write32(sptm.nvme.nvmmu + NVMMU_TCB_INVALIDATE, cid);
        sysop("dsb sy");
        u32 packed_status = read32(sptm.nvme.nvmmu + NVMMU_TCB_STATUS + (cid / 4) * sizeof(u32));
        u32 cid_status = (packed_status >> ((cid % 4) * 5)) & 0xf;
        if (cid_status)
            return false;
    }

    if (!sptm_nvme_tl_idle(command, retry)) {
        command->state = 2;
        ctx->regs[0] = 0;
        return true;
    }

    bool writable = command->dma_flags & BIT(0);
    size_t released = 0;
    for (; released < command->count; released++) {
        u64 target = sptm_nvme_command_target(command, released);
        if (sptm_adjust_data_ref(target, writable, -1))
            continue;
        for (size_t rollback = 0; rollback < released; rollback++) {
            u64 rollback_target = sptm_nvme_command_target(command, rollback);
            sptm_adjust_data_ref(rollback_target, writable, 1);
        }
        return false;
    }

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

static bool sptm_nvme_cache_values(bool *valid, u64 *cache, const u64 *values, size_t count)
{
    if (*valid) {
        for (size_t index = 0; index < count; index++) {
            if (cache[index] != values[index])
                return false;
        }
        return true;
    }

    memcpy(cache, values, count * sizeof(u64));
    *valid = true;
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
    if (!sptm.nvme.configured)
        return false;

    struct sptm_nvme *nvme = &sptm.nvme;
    u64 *args = ctx->regs;
    bool handled = false;
    spin_lock(&sptm.service_lock);
    switch (endpoint) {
        case 0:
            sysop("dsb sy");
            sptm_nvme_mmio_write64_lo_hi(nvme->nvmmu + NVMMU_ADMIN_TCB_BASE, nvme->admin_tcbs);
            sptm_nvme_mmio_write64_lo_hi(nvme->nvmmu + NVMMU_IO_TCB_BASE, nvme->io_tcbs);
            sptm_nvme_mmio_write32(nvme->nvmmu + NVMMU_QUEUE_COUNT, nvme->queue_entries - 1);
            nvme->coastguard_enabled = true;
            handled = true;
            break;
        case 1:
            handled = sptm_nvme_map_pages(ctx);
            break;
        case 2:
            // UNMAP_PAGES reports its own completion state in x0.
            handled = sptm_nvme_unmap_pages(ctx);
            spin_unlock(&sptm.service_lock);
            return handled;
        case 3:
            handled = args[0] == nvme->queue_entries && args[1] == nvme->protocol;
            break;
        case 4:
            // Admin queues: submission base and size, then completion base and size.
            handled =
                args[1] && args[1] <= nvme->queue_entries && args[3] &&
                args[3] <= nvme->queue_entries &&
                sptm_nvme_queue_allowed(args[0], args[1] * 0x40) &&
                sptm_nvme_queue_allowed(args[2], args[3] * 0x10) &&
                sptm_nvme_cache_values(&nvme->admin_queues_valid, nvme->admin_queues, args, 4);
            if (handled) {
                sptm_nvme_mmio_write32(nvme->queue_bar + NVME_AQA, args[1] | (args[3] << 16));
                sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_ASQ, args[0]);
                sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_ACQ, args[2]);
            }
            break;
        case 5:
            handled = args[0] && args[0] <= nvme->queue_entries && args[1] &&
                      args[1] <= nvme->queue_entries &&
                      sptm_nvme_cache_values(&nvme->io_sizes_valid, nvme->io_sizes, args, 2);
            if (handled)
                sptm_nvme_mmio_write32(nvme->queue_bar + NVME_IOQA, args[0] | (args[1] << 16));
            break;
        case 6: {
            u64 size = nvme->io_sizes_valid ? nvme->io_sizes[0] : nvme->queue_entries;
            handled = sptm_nvme_queue_allowed(args[0], size * 0x40) &&
                      sptm_nvme_cache_values(&nvme->io_sq_valid, &nvme->io_sq, args, 1);
            if (handled)
                sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_IOSQ, args[0]);
            break;
        }
        case 7: {
            u64 size = nvme->io_sizes_valid ? nvme->io_sizes[1] : nvme->queue_entries;
            handled = sptm_nvme_queue_allowed(args[0], size * 0x10) &&
                      sptm_nvme_cache_values(&nvme->io_cq_valid, &nvme->io_cq, args, 1);
            if (handled)
                sptm_nvme_mmio_write64_lo_hi(nvme->queue_bar + NVME_IOCQ, args[0]);
            break;
        }
        case 8:
            handled = nvme->sha_present && !(args[0] & SPTM_PAGE_MASK) &&
                      args[1] == SPTM_PAGE_SIZE && !(args[2] & ~3ULL) &&
                      sptm_nvme_range_allowed(args[0], args[1]) &&
                      sptm_nvme_cache_values(&nvme->sha_valid, nvme->sha, args, 3);
            if (handled) {
                write32(nvme->sha_base, args[0] >> SPTM_PAGE_SHIFT);
                write32(nvme->sha_base + 4, args[2]);
                sysop("dsb sy");
            }
            break;
    }
    spin_unlock(&sptm.service_lock);
    if (handled)
        ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return handled;
}
