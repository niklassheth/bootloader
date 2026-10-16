/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

struct sptm_state sptm;

// Whether [pa, pa + size) lies in XNU-managed RAM, which has frame-table entries.
bool sptm_is_managed_range(u64 pa, size_t size)
{
    return pa >= sptm.managed_start && pa < sptm.managed_end && size <= sptm.managed_end - pa;
}

// XNU passes its own data as physmap addresses; physical addresses pass through.
u64 sptm_pointer_pa(u64 pointer)
{
    if (pointer >= sptm.physmap_base && pointer < sptm.physmap_end)
        return sptm.managed_start + pointer - sptm.physmap_base;
    return pointer;
}

u64 sptm_physmap_va(u64 pa)
{
    return sptm.physmap_base + pa - sptm.managed_start;
}

u64 sptm_iommu_table_va(u64 pa)
{
    return pa | REGION_NORMAL_NC;
}

struct sptm_cpu *sptm_find_cpu(u32 phys_id)
{
    for (size_t index = 0; index < ARRAY_SIZE(sptm.cpus); index++) {
        struct sptm_cpu *cpu = &sptm.cpus[index];
        if (cpu->valid && cpu->phys_id == phys_id)
            return cpu;
    }

    return NULL;
}

struct sptm_cpu *sptm_register_cpu(u32 phys_id)
{
    struct sptm_cpu *cpu = sptm_find_cpu(phys_id);
    if (cpu)
        return cpu;

    if (sptm.cpu_count >= sptm.max_cpus)
        return NULL;

    cpu = &sptm.cpus[sptm.cpu_count];
    cpu->phys_id = phys_id;
    cpu->logical_id = sptm.cpu_count;
    cpu->valid = true;
    sptm.cpu_count++;
    return cpu;
}

u64 sptm_scratch_pa(struct exc_info *ctx)
{
    struct sptm_cpu *cpu = sptm_find_cpu(sptm.hv_phys_ids[ctx->cpu_id]);
    return cpu ? sptm.scratch_pa + cpu->logical_id * SPTM_PAGE_SIZE : 0;
}
