/* SPDX-License-Identifier: GPL-2.0-only */

#include "hv_sptm_internal.h"

// Powered-down or absent DART registers read as DART_UNAVAILABLE or all ones.
static bool sptm_dart_live(u32 value)
{
    return value != DART_UNAVAILABLE && value != UINT32_MAX;
}

static bool sptm_dart_clear_errors(const struct sptm_dart *dart)
{
    for (size_t index = 0; index < dart->instance_count; index++) {
        u64 base = dart->instances[index];

        if (dart->sid_exceptions) {
            // DART 2.2 uses write-one-to-clear per-SID exception words.
            for (size_t word = 0; word < dart->sid_words; word++) {
                u64 address = base + DART_SID_EXCEPTIONS + word * sizeof(u32);
                u32 pending = read32(address);
                if (pending && sptm_dart_live(pending))
                    write32(address, pending);
            }
        } else {
            u32 error = read32(base + DART_ERROR);
            if (!sptm_dart_live(error))
                return false;
            if (error & DART_ERROR_SECONDARY) {
                if (!(dart->flags & SPTM_DART_IGNORE_SECONDARY))
                    return false;
                u32 secondary = read32(base + 0x1c0);
                if (secondary && sptm_dart_live(secondary))
                    write32(base + 0x1c0, secondary);
            }
            if (error)
                write32(base + DART_ERROR, error);
        }

        write32(base + dart->interrupt_status_offset, UINT32_MAX);
    }

    sysop("dsb sy");
    return true;
}

// Set or clear an FPAG slice's clock protection; fails if the slice is unreadable.
static bool sptm_dart_clock_protect(u64 address, bool protect)
{
    u32 value = read32(address);
    if (!sptm_dart_live(value))
        return false;
    write32(address, protect ? value | FPAG_CLOCK_PROTECTION : value & ~FPAG_CLOCK_PROTECTION);
    return true;
}

static struct sptm_dart_clock_ref *sptm_dart_clock_ref(u64 address, bool create)
{
    struct sptm_dart_clock_ref *empty = NULL;

    for (size_t index = 0; index < ARRAY_SIZE(sptm.dart_clock_refs); index++) {
        struct sptm_dart_clock_ref *ref = &sptm.dart_clock_refs[index];
        if (ref->valid && ref->address == address)
            return ref;
        if (!ref->valid && !empty)
            empty = ref;
    }

    if (!create || !empty)
        return NULL;
    empty->address = address;
    empty->valid = true;
    return empty;
}

static bool sptm_dart_shared_clock_acquire(u64 address)
{
    struct sptm_dart_clock_ref *ref = sptm_dart_clock_ref(address, true);
    if (!ref || ref->refs == UINT16_MAX)
        return false;

    if (!ref->refs) {
        u32 value = read32(address);
        if (!sptm_dart_live(value))
            return false;
        ref->original = value & FPAG_CLOCK_PROTECTION;
        if (!ref->original)
            write32(address, value | FPAG_CLOCK_PROTECTION);
    }
    ref->refs++;
    return true;
}

static void sptm_dart_shared_clock_release(u64 address)
{
    struct sptm_dart_clock_ref *ref = sptm_dart_clock_ref(address, false);
    if (!ref || !ref->refs)
        return;

    ref->refs--;
    if (ref->refs)
        return;

    if (!ref->original)
        sptm_dart_clock_protect(address, false);
    memset(ref, 0, sizeof(*ref));
}

static bool sptm_dart_powerup_sidebands(struct sptm_dart *dart)
{
    if (dart->clock_count) {
        if (dart->flags & SPTM_DART_UNGANG_SHARED_PS) {
            // apcie0 and apcie2 share this FPAG slice on T8132.
            if (dart->clock_active)
                return true;

            size_t acquired = 0;
            for (; acquired < dart->clock_count; acquired++) {
                if (!sptm_dart_shared_clock_acquire(dart->clock_entries[acquired]))
                    break;
            }
            if (acquired != dart->clock_count) {
                while (acquired)
                    sptm_dart_shared_clock_release(dart->clock_entries[--acquired]);
                return false;
            }
            dart->clock_active = true;
            sysop("dsb sy");
            return true;
        }

        for (size_t index = 0; index < dart->clock_count; index++) {
            if (!sptm_dart_clock_protect(dart->clock_entries[index], true))
                return false;
        }
    }
    sysop("dsb sy");
    return true;
}

static u64 sptm_dart_read_sized(u64 address, u32 size)
{
    switch (size) {
        case 1:
            return read8(address);
        case 2:
            return read16(address);
        case 4:
            return read32(address);
        default:
            return read64(address);
    }
}

static void sptm_dart_write_sized(u64 address, u32 size, u64 value)
{
    switch (size) {
        case 1:
            write8(address, value);
            break;
        case 2:
            write16(address, value);
            break;
        case 4:
            write32(address, value);
            break;
        default:
            write64(address, value);
            break;
    }
}

static bool sptm_dart_apply_tunables(const struct sptm_dart *dart)
{
    for (size_t index = 0; index < dart->tunable_count; index++) {
        const struct sptm_dart_tunable *entry = &dart->tunable_entries[index];
        u64 address = entry->base + entry->offset;
        u64 wanted = entry->value & entry->mask;
        u64 current = sptm_dart_read_sized(address, entry->size);
        if ((current & entry->mask) != wanted) {
            sptm_dart_write_sized(address, entry->size, (current & ~entry->mask) | wanted);
            if ((sptm_dart_read_sized(address, entry->size) & entry->mask) != wanted)
                return false;
        }
    }
    sysop("dsb sy");
    return true;
}

static void sptm_dart_program_dapf(const struct sptm_dart *dart)
{
    for (size_t index = 0; index < dart->dapf_count; index++) {
        const struct sptm_dart_dapf *entry = &dart->dapf_entries[index];
        write32(entry->base + 0x04, entry->r4);
        write64(entry->base + 0x08, entry->start);
        write64(entry->base + 0x10, entry->end);
        write32(entry->base + 0x00, entry->control);
        write32(entry->base + 0x20, entry->r20);
    }
    sysop("dsb sy");
}

static void sptm_dart_powerdown_sidebands(struct sptm_dart *dart)
{
    if (dart->flags & SPTM_DART_UNGANG_SHARED_PS) {
        if (!dart->clock_active)
            return;
        for (size_t index = 0; index < dart->clock_count; index++)
            sptm_dart_shared_clock_release(dart->clock_entries[index]);
        dart->clock_active = false;
        sysop("dsb sy");
        return;
    }

    for (size_t index = 0; index < dart->clock_count; index++)
        sptm_dart_clock_protect(dart->clock_entries[index], false);
    sysop("dsb sy");
}

static bool sptm_dart_program_sids(struct sptm_dart *dart)
{
    for (u32 sid = 0; sid < dart->sid_count; sid++) {
        struct sptm_dart_sid *state = &dart->sid_states[sid];
        if (!(state->flags & SPTM_DART_SID_KNOWN) || (state->flags & SPTM_DART_SID_EXCLAVE))
            continue;

        u32 ttbr = 0;
        if ((state->tcr & DART_TCR_TRANSLATE) && state->root)
            ttbr = ((state->root >> 12) & 0xfffffffcULL) | DART_TTBR_VALID;

        // Firmware-locked instances keep the SID configuration it programmed.
        for (size_t index = 0; index < dart->instance_count; index++) {
            u64 base = dart->instances[index];
            if (read32(base + DART_REG_PROTECT) & BIT(0))
                continue;
            write32(base + DART_TTBR + sid * sizeof(u32), ttbr);
            write32(base + DART_TCR + sid * sizeof(u32), state->tcr);
        }
    }
    return sptm_dart_flush_all(dart);
}

// INIT discovers the version and SID capacity, which all instances of one DART share.
bool sptm_dart_init(struct exc_info *ctx, struct sptm_dart *dart)
{
    u32 params_8 = read32(dart->instances[0] + DART_PARAMS_8);
    u32 params_c = read32(dart->instances[0] + DART_PARAMS_C);
    if (!sptm_dart_live(params_8))
        return false;

    u8 major = (params_8 >> 8) & 0xff;
    u8 minor = params_8 & 0xff;
    dart->sid_exceptions = major > 2 || (major == 2 && minor >= 2);
    dart->sid_words = ((params_c & 0x1ff) + 31) / 32;
    dart->interrupt_status_offset = dart->sid_exceptions ? 0x164 : 0x160;
    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}

static bool sptm_dart_save_power_state(struct sptm_dart *dart)
{
    for (size_t index = 0; index < dart->instance_count; index++) {
        u64 base = dart->instances[index];
        for (size_t word = 0; word < dart->sid_words; word++) {
            u32 streams = read32(base + DART_ENABLE_STREAMS + word * sizeof(u32));
            if (!sptm_dart_live(streams))
                return false;
            dart->saved_streams[index][word] = streams;
        }
    }
    dart->saved_valid = true;
    return true;
}

static void sptm_dart_disable_translating_streams(struct sptm_dart *dart)
{
    u32 translating[(SPTM_MAX_DART_SIDS + 31) / 32] = {};
    for (u32 sid = 0; sid < dart->sid_count; sid++) {
        struct sptm_dart_sid *state = &dart->sid_states[sid];
        if ((state->flags & SPTM_DART_SID_KNOWN) && !(state->flags & SPTM_DART_SID_EXCLAVE) &&
            (state->tcr & DART_TCR_TRANSLATE))
            translating[sid / 32] |= BIT(sid % 32);
    }
    for (size_t index = 0; index < dart->instance_count; index++) {
        for (size_t word = 0; word < dart->sid_words; word++) {
            if (translating[word])
                write32(dart->instances[index] + DART_DISABLE_STREAMS + word * sizeof(u32),
                        translating[word]);
        }
    }
    sysop("dsb sy");
}

static void sptm_dart_restore_runtime_state(struct sptm_dart *dart)
{
    for (size_t index = 0; index < dart->instance_count; index++) {
        u64 base = dart->instances[index];
        if (dart->limits_valid && dart->saved_valid) {
            write32(base + DART_TEQRESERVE, dart->saved_teqreserve[index]);
            write32(base + DART_TLIMIT, dart->saved_tlimit[index]);
        }
        for (size_t word = 0; word < dart->sid_words; word++) {
            u32 streams =
                dart->saved_valid ? dart->saved_streams[index][word] : dart->boot_streams[word];
            if (streams)
                write32(base + DART_ENABLE_STREAMS + word * sizeof(u32), streams);
        }
    }
    sysop("dsb sy");
}

bool sptm_dart_power(struct exc_info *ctx, struct sptm_dart *dart, bool power_up)
{
    if (power_up) {
        if (!sptm_dart_powerup_sidebands(dart))
            return false;

        // Mark the DART powered before programming SIDs and flushing.
        dart->powered = true;

        if (!sptm_dart_apply_tunables(dart) || !sptm_dart_program_sids(dart))
            return false;
        if (!dart->limits_valid) {
            for (size_t index = 0; index < dart->instance_count; index++) {
                u64 base = dart->instances[index];
                dart->saved_tlimit[index] = read32(base + DART_TLIMIT);
                dart->saved_teqreserve[index] = read32(base + DART_TEQRESERVE);
            }
            dart->limits_valid = true;
        }
        sptm_dart_program_dapf(dart);
        if (!sptm_dart_clear_errors(dart))
            return false;
        sptm_dart_restore_runtime_state(dart);
    } else {
        if (!dart->powered) {
            ctx->regs[0] = SPTM_STATUS_SUCCESS;
            return true;
        }
        if (!sptm_dart_save_power_state(dart))
            return false;
        sptm_dart_disable_translating_streams(dart);
        sptm_dart_powerdown_sidebands(dart);
        dart->powered = false;
    }

    ctx->regs[0] = SPTM_STATUS_SUCCESS;
    return true;
}
