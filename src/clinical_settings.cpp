#include "clinical_settings.h"
#include "custom_settings_protocol.h"
#include "hex_util.h"
#include <limits.h>
#include "clinical_jobs.h"
#include "settings_defs.h"
#include "uart_arbiter.h"
#include "memory_manager.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace ClinicalSettings {
namespace {
constexpr uint8_t VALID = 1, CUSTOM = 2;
const char *const GROUPS[] = {"therapy", "comfort", "accessories", "options", "configuration"};

enum Stage : uint8_t {
    ARRAY_START, ENTRY_START, LABEL, GROUP, GROUP_PATH_START, GROUP_PATH_ITEM,
    VALUE, EDITABLE, SOURCE, TYPE,
    OPTIONS_START, OPTION_START, OPTION_VALUE, OPTION_LABEL, OPTION_END,
    SCALE, RAW_STEP, DECIMALS, RAW_MIN, RAW_MAX, UNITS, DISPLAY_VALUE, MINIMUM,
    MAXIMUM, STEP, ENTRY_END, DONE
};

}

Snapshot::~Snapshot() { reset(); }

void Snapshot::reset() {
    aircannect::Memory::free(values_);
    values_ = nullptr;
    count_ = 0;
    length_ = 0;
    error_ = nullptr;
    metadata_.reset();
}

bool read_raw(const char *cmd, int &value) {
    uint16_t remaining = ClinicalJobs::remaining_ms();
    if (!remaining) return false;
    uint32_t raw;
    if (Arbiter::read_var_hex(cmd, CMD_SRC_TCP, CMD_PRIO_NORMAL, raw,
                             0, nullptr, remaining) !=
        Arbiter::VarResult::Ok || raw > INT_MAX) return false;
    value = static_cast<int>(raw);
    return true;
}

bool known_stock(const char *cmd) { return var_lookup(cmd) != nullptr; }

void mode_label(int mode, char *out, size_t capacity) {
    if (capacity && !setting_enum_label(var_lookup("MOP"), mode, out, capacity)) out[0] = 0;
}

int collect(Snapshot &snapshot) {
    snapshot.reset();
    auto fail = [&](const char *error) {
        snapshot.reset();
        snapshot.error_ = error;
        return 503;
    };
    if (!snapshot.metadata_.acquire())
        return fail("{\"error\":\"settings_metadata_unavailable\"}");
    int mop = 0;
    if (!read_raw("MOP", mop)) return fail("{\"error\":\"settings_mode_unavailable\"}");
    if (mop < 0 || mop >= MODE_COUNT) mop = 0;

    int tmu = -1;
    read_raw("TMU", tmu);

    int aoa = -1;
    read_raw("AOA", aoa);
    bool leak_alert_active = false;
    if (aoa == 0) {
        char response[32] = {};
        uint16_t length = sizeof(response);
        const uint16_t remaining = ClinicalJobs::remaining_ms();
        uint8_t flags = 0;
        const bool ok = remaining && Arbiter::send_cmd("G C #ALR", CMD_SRC_TCP,
            CMD_PRIO_NORMAL, response, &length, 0, remaining);
        // ACT describes availability; the value of ALR only enables/disables it.
        leak_alert_active = ok && length < sizeof(response) &&
            CustomSettingsProtocol::parse_enum_flags(qframe_response_value(response), flags) &&
            (flags & 0x01);
    }
    auto visible = [&](const char *name) {
        if (!strcmp(name, "HTS")) return tmu == 0;
        if (!strcmp(name, "HTF")) return tmu == 1;
        if (!strcmp(name, "ALR")) return aoa != 1 && (aoa != 0 || leak_alert_active);
        if (aoa != 0) return true;
        for (const char *alarm : {"ALV", "HLE", "NMF", "LMA", "APX", "SPX"})
            if (!strcmp(name, alarm)) return false;
        return true;
    };

    // Count the selected layout first, then allocate exactly its compact values.
    auto layout = [&](bool read) {
        uint16_t count = 0;
        auto stock = [&](const char *name, uint8_t group) {
            if (!visible(name) || snapshot.metadata_.find(name) >= 0) return;
            const var_def_t *v = var_lookup(name);
            if (!v) return;
            if (read) {
                int raw = 0;
                bool ok = true;
                if (!strcmp(name, "MOP")) raw = mop;
                else if (!strcmp(name, "TMU")) { raw = tmu; ok = tmu >= 0; }
                else ok = read_raw(name, raw);
                snapshot.values_[count] = {(uint32_t)raw, (uint16_t)(v - VAR_CATALOG),
                                          group, (uint8_t)(ok ? VALID : 0)};
            }
            count++;
        };
        auto list = [&](const char *const *names, uint8_t group) {
            for (; *names; names++) stock(*names, group);
        };
        for (uint8_t group = 0; group < 5; group++) {
            switch (group) {
                case 0: stock("MOP", 0); list(MODE_LAYOUT[mop], 0); break;
                case 1: list(COMFORT_LAYOUT[mop], 1); list(EPR_VARS, 1); break;
                case 2: list(ACCESSORY_VARS, 2); break;
                case 3: list(OPTION_VARS, 3); break;
                case 4: list(CONFIGURATION_VARS, 4); break;
            }
            for (uint16_t i = 0; i < snapshot.metadata_.count(); i++) {
                CustomSettings::entry_view_t entry;
                snapshot.metadata_.entry(i, entry);
                if (entry.category != group || !(entry.mop_mask & (1u << mop)) ||
                    !visible(entry.name)) continue;
                if (read) {
                    uint32_t raw = 0;
                    bool ok;
                    if (!strcmp(entry.name, "TMU")) { raw = (uint32_t)tmu; ok = tmu >= 0; }
                    else ok = snapshot.metadata_.read_raw(i, raw);
                    snapshot.values_[count] = {raw, i, group, (uint8_t)(CUSTOM | (ok ? VALID : 0))};
                }
                count++;
            }
        }
        return count;
    };
    snapshot.count_ = layout(false);
    size_t bytes = snapshot.storage_bytes();
    if (bytes) {
        snapshot.values_ = static_cast<Value *>(aircannect::Memory::alloc_large(bytes));
        if (!snapshot.values_) return fail("{\"error\":\"settings_allocation_failed\"}");
    }
    layout(true);
    if (snapshot.metadata_.generation() != CustomSettings::generation())
        return fail("{\"error\":\"settings_invalidated\"}");

    Cursor counter;
    size_t count;
    while ((count = counter.read(snapshot, nullptr, 512))) snapshot.length_ += count;
    return 200;
}

bool Cursor::next(const Snapshot &snapshot) {
    if (stage_ == ARRAY_START) {
        stage_ = ENTRY_START; token("["); return true;
    }
    if (row_ == snapshot.count_) {
        if (stage_ == DONE) return false;
        stage_ = DONE; token("]"); return true;
    }

    const Value &value = snapshot.values_[row_];
    bool custom = value.flags & CUSTOM, valid = value.flags & VALID;
    CustomSettings::entry_view_t entry = {};
    const var_def_t *stock = nullptr;
    if (custom) snapshot.metadata_.entry(value.descriptor, entry);
    else stock = &VAR_CATALOG[value.descriptor];
    bool enumerated = custom ? entry.kind == CustomSettings::KIND_ENUM : stock->type == SET_ENUM;
    bool scaled = !custom && stock->type == SET_SCALED && stock->scale_div > 1;
    uint8_t decimals = custom ? entry.decimals : stock->decimals;

    while (true) {
        switch (stage_++) {
            case ENTRY_START:
                field(row_ ? ",{\"cmd\":" : "{\"cmd\":", custom ? entry.name : stock->cmd); return true;
            case LABEL: field(",\"label\":", custom ? entry.label : stock->label); return true;
            case GROUP: field(",\"group\":", GROUPS[value.group]); return true;
            case GROUP_PATH_START:
                if (!custom || !entry.group_count) { stage_ = VALUE; break; }
                option_ = 0;
                option_offset_ = 0;
                token(",\"groups\":["); return true;
            case GROUP_PATH_ITEM:
                if (option_ == entry.group_count) { token("]"); return true; }
                field(option_ ? "," : "", entry.groups + option_offset_);
                option_offset_ += strlen(entry.groups + option_offset_) + 1;
                option_++;
                stage_ = GROUP_PATH_ITEM;
                return true;
            case VALUE: integer(",\"value\":", valid ? (int64_t)value.raw : -1); return true;
            case EDITABLE: field(",\"editable\":", !custom || (entry.flags & 0x04) ? "true" : "false", false); return true;
            case SOURCE: if (custom) { field(",\"source\":", "custom"); return true; } break;
            case TYPE:
                field(",\"type\":", enumerated ? "enum" : custom ? "numeric" : scaled ? "scaled" : "int"); return true;
            case OPTIONS_START:
                if (enumerated) {
                    if (!custom && !stock->enum_options) { stage_ = ENTRY_END; break; }
                    option_ = 0;
                    option_offset_ = 0;
                    token(",\"options\":["); return true;
                }
                stage_ = custom || scaled ? SCALE : ENTRY_END;
                break;
            case OPTION_START:
                if ((custom && option_ == entry.option_count) || (!custom && option_offset_ == SIZE_MAX)) {
                    stage_ = ENTRY_END; token("]"); return true;
                }
                token(option_ ? (custom ? ",{" : ",") : (custom ? "{" : "")); return true;
            case OPTION_VALUE:
                if (custom) {
                    CustomSettings::option_view_t option;
                    snapshot.metadata_.option(value.descriptor, option_, option);
                    integer("\"value\":", option.value); return true;
                } else {
                    const char *start = stock->enum_options + option_offset_;
                    size_t length = strcspn(start, ",");
                    option_offset_ = start[length] ? option_offset_ + length + 1 : SIZE_MAX;
                    token(start, true, length); stage_ = OPTION_END; return true;
                }
            case OPTION_LABEL: {
                CustomSettings::option_view_t option;
                snapshot.metadata_.option(value.descriptor, option_, option);
                field(",\"label\":", option.label); return true;
            }
            case OPTION_END:
                option_++; stage_ = OPTION_START;
                if (custom) { token("}"); return true; }
                break;
            case SCALE: integer(custom ? ",\"scale\":" : ",\"scale_div\":", custom ? entry.scale : stock->scale_div); return true;
            case RAW_STEP: if (custom) { integer(",\"raw_step\":", entry.step); return true; } break;
            case DECIMALS: integer(",\"decimals\":", decimals); return true;
            case RAW_MIN: if (custom) { integer(",\"raw_min\":", entry.minimum); return true; } break;
            case RAW_MAX: if (custom) { integer(",\"raw_max\":", entry.maximum); return true; } break;
            case UNITS: if (custom) { field(",\"units\":", entry.units); return true; } break;
            case DISPLAY_VALUE:
                if (valid) {
                    decimal(",\"display\":", value.raw,
                            custom ? entry.scale : stock->scale_div, decimals); return true;
                }
                break;
            case MINIMUM:
                if (custom) { decimal(",\"min\":", entry.minimum, entry.scale, decimals); return true; } break;
            case MAXIMUM:
                if (custom) { decimal(",\"max\":", entry.maximum, entry.scale, decimals); return true; } break;
            case STEP:
                if (custom) {
                    decimal(",\"step\":", entry.step, entry.scale, decimals); return true;
                }
                if (valid) {
                    // Stock precision is 0, 1 or 2 decimal places.
                    snprintf(number_, sizeof(number_), "%s",
                             decimals == 0 ? "1" : decimals == 1 ? "0.1" : "0.01");
                    field(",\"step\":", number_); return true;
                }
                break;
            case ENTRY_END:
                row_++; stage_ = ENTRY_START; token("}"); return true;
            default: return false;
        }
    }
}

size_t Cursor::read(const Snapshot &snapshot, char *out, size_t capacity) {
    return read_tokens(out, capacity, [&] { return next(snapshot); });
}

}  // namespace ClinicalSettings
