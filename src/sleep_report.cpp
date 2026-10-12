#include "sleep_report.h"
#include "air10_stored.h"
#include "airsense_state.h"
#include "app_config.h"
#include "hex_util.h"
#include "uart_arbiter.h"
#include "board.h"
#include <algorithm>
#include <stdio.h>
#include <string.h>

namespace SleepReport {

void init() { EdfReport::init(); }
void tick() { EdfReport::tick(); }
Source default_source() {
    return AB_STORAGE_HAS_SDCARD ? Source::Edf : Source::Device;
}
const char *source_name(Source source) {
    return source == Source::Edf ? "edf" : "device";
}
void get_publication(Publication &out) {
    out.source = default_source();
    EdfReport::get_status(out.local);
}
bool submit(const LocalRequest &request, uint32_t &id, const char **error) {
    return EdfReport::submit(request, id, error);
}
int poll(uint32_t id, Result &out) { return EdfReport::poll(id, out); }
void cancel(uint32_t id) { EdfReport::cancel(id); }

namespace {
const Field DAY_FIELDS[] = {
    {"THD", "Usage", 60, 0, "", Format::Duration},
    {"OND", "Mask interval duration", 60, 0, "", Format::Duration},
    {"AHI", "AHI", 10, 1, "/hr"},
    {"AIS", "Total AI", 10, 1, "/hr"},
    {"CLI", "Obstructive AI", 10, 1, "/hr"},
    {"OPI", "Central AI", 10, 1, "/hr"},
    {"HIS", "Hypopnea index", 10, 1, "/hr"},
    {"UAI", "Unknown AI", 10, 1, "/hr"},
    {"RIN", "RERA index", 10, 1, "/hr"},
    {"PIM", "Pressure median", 50, 1, "cmH2O"},
    {"PI9", "Pressure P95", 50, 1, "cmH2O"},
    {"PIA", "Pressure maximum", 50, 1, "cmH2O"},
    {"PE9", "Expiratory pressure P95", 50, 1, "cmH2O"},
    {"LKM", "Leak median", 50, 2, "L/s"},
    {"LK9", "Leak P95", 50, 2, "L/s"},
    {"LMX", "Leak maximum", 50, 2, "L/s"},
};
const Field PERIOD_FIELDS[] = {
    {"DRD", "Days used", 1, 0, "", Format::DaysPeriod},
    {"VRD", "Days used 4 hours or more", 1, 0, "", Format::DaysPeriod},
    {"WRD", "Average usage on used days", 60, 0, "", Format::Duration},
    {"XRD", "Total usage", 60, 0, "", Format::Duration},
    {"ARD", "Average daily AHI", 10, 1, "/hr"},
    {"TRD", "Average daily total AI", 10, 1, "/hr"},
    {"CRD", "Average daily central AI", 10, 1, "/hr"},
    {"ZAI", "Average daily pressure P95", 50, 1, "cmH2O"},
    {"LRS", "Average daily leak P95", 50, 2, "L/s"},
};
static_assert(sizeof(DAY_FIELDS) / sizeof(Field) <= MAX_FIELDS);
static_assert(sizeof(PERIOD_FIELDS) / sizeof(Field) <= MAX_FIELDS);

struct ReadBudget {
    Budget remaining;
    bool interrupted = false;

    uint16_t operator()(uint32_t reserve_ms) {
        interrupted |= !AirSenseState::device_standby();
        return interrupted ? 0 : remaining(reserve_ms);
    }
};

using Scalar = Arbiter::VarResult;
Scalar read_scalar(const char *tag, uint32_t &value, uint16_t timeout) {
    if (!timeout) return Scalar::Failed;
    return Arbiter::read_var_hex(tag, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, value,
                                 0, nullptr, timeout);
}

bool select_period(uint16_t selection, uint16_t timeout) {
    if (!timeout) return false;
    char command[20], response[40] = {};
    snprintf(command, sizeof(command), "P S #SEP %04X", selection);
    uint16_t length = sizeof(response);
    if (!Arbiter::send_cmd(command, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                           response, &length, 0, timeout) || length >= sizeof(response)) return false;
    char expected[40];
    snprintf(expected, sizeof(expected), "%s = %04X", command, selection);
    return strcmp(response, expected) == 0;
}

int fail(Snapshot &out, const char *error, int code = 503) {
    out.error = error;
    return code;
}

int collect_day(Snapshot &out, ReadBudget &budget) {
    uint32_t before, after, day = out.request.selection, current;
    if (read_scalar("ZEN", before, budget(0)) != Scalar::Ok ||
        read_scalar("DSD", current, budget(0)) != Scalar::Ok || current >= 0xFFFF)
        return fail(out, "report_read_failed");
    out.current_day = current;
    if (!day && read_scalar("SSD", day, budget(0)) != Scalar::Ok)
        return fail(out, "report_read_failed");
    if (day == 0xFFFF || !day) return 200;  // No saved day yet.
    if (day < 0x1000 || day > current)
        return fail(out, "report_day_unavailable", 400);
    out.day = day;
    Air10Stored::Value value;
    uint16_t remaining = budget(0);
    if (!remaining) return fail(out, "report_read_failed");
    auto result = Air10Stored::read("LSD", out.day, value,
        Config::get().uart_cmd_timeout_ms, false, remaining);
    if (result == Air10Stored::ReadResult::Unsupported)
        return fail(out, "report_history_unsupported", 422);
    if (result == Air10Stored::ReadResult::Failed)
        return fail(out, "report_read_failed");
    if (value.present && value.sample_count != 1)
        return fail(out, "report_invalid_samples");
    out.present = value.present && uint16_t(value.samples[0]) != 0xFFFF;
    if (out.present && uint16_t(value.samples[0]) != day)
        return fail(out, "report_day_mismatch");
    if (out.present) {
        size_t count;
        const Field *table = fields(View::Day, count);
        for (size_t i = 0; i < count + 2; i++) {
            const char *tag = i < count ? table[i].tag : i == count ? "ONT" : "OFT";
            remaining = budget(0);
            if (!remaining) return fail(out, "report_read_failed");
            result = Air10Stored::read(tag, out.day, value,
                Config::get().uart_cmd_timeout_ms, false, remaining);
            if (result == Air10Stored::ReadResult::Failed)
                return fail(out, "report_read_failed");
            if (result == Air10Stored::ReadResult::Unsupported || !value.present) continue;
            if (i < count) {
                if (value.sample_count != 1) return fail(out, "report_invalid_samples");
                uint16_t raw = value.samples[0];
                if (raw != 0xFFFF) out.values[i] = raw;
            } else {
                uint16_t *minutes = i == count ? out.on : out.off;
                for (uint8_t j = 0; j < value.sample_count; j++) {
                    uint16_t raw = value.samples[j];
                    if (raw != 0xFFFF && raw > 1440) return fail(out, "report_invalid_samples");
                    minutes[j] = raw;
                }
            }
        }
    }
    if (read_scalar("ZEN", after, budget(0)) != Scalar::Ok)
        return fail(out, "report_read_failed");
    if (before != after) return fail(out, "report_history_changed", 409);
    return 200;
}

int collect_period(Snapshot &out, ReadBudget &budget) {
    uint32_t original, current, noticed;
    // Reserve cleanup inside the existing job deadline, not after it expires.
    uint32_t reserve = 6u * std::max(Config::get().uart_cmd_timeout_ms,
                                    Arbiter::SCALAR_TIMEOUT_MIN_MS);
    if (read_scalar("SEP", original, budget(reserve)) != Scalar::Ok || original > 5 ||
        read_scalar("DSD", current, budget(reserve)) != Scalar::Ok || current >= 0xFFFF)
        return fail(out, "report_read_failed");
    out.current_day = out.day = current;
    bool changed = original != out.request.selection;
    int code = 200;
    if (changed && !select_period(out.request.selection, budget(reserve)))
        code = fail(out, "report_period_write_failed");
    if (code == 200 &&
        (read_scalar("SET", noticed, budget(reserve)) != Scalar::Ok ||
         noticed != period_days(out.request.selection)))
        code = fail(out, "report_period_not_selected", 409);
    if (code == 200) {
        size_t count;
        const Field *table = fields(View::Period, count);
        if (read_scalar("URD", current, budget(reserve)) != Scalar::Ok || current > 365)
            code = fail(out, "report_read_failed");
        else out.days = current;
        for (size_t i = 0; code == 200 && i < count; i++) {
            auto result = read_scalar(table[i].tag, current, budget(reserve));
            if (result == Scalar::Failed) code = fail(out, "report_read_failed");
            else if (result == Scalar::Ok) out.values[i] = current;
        }
        out.present = out.values[0] != 0;  // Missing DRD does not prove no therapy.
    }
    // Undo our temporary GUI setting even if therapy interrupted collection.
    if (read_scalar("SEP", current, changed ? budget.remaining(0) : budget(0)) != Scalar::Ok)
        return fail(out, changed ? "report_restore_failed" : "report_read_failed");
    if (current != out.request.selection) {
        // Do not overwrite an observed concurrent GUI/TCP selection.
        if (code != 200 && current == original) return code;
        return fail(out, "report_period_changed", 409);
    }
    if (changed && (!select_period(original, budget.remaining(0)) ||
        read_scalar("SEP", current, budget.remaining(0)) != Scalar::Ok || current != original))
        return fail(out, "report_restore_failed");
    return code;
}
}

uint16_t period_days(uint16_t selection) {
    static const uint16_t days[] = {1, 7, 30, 90, 180, 365};
    return selection < 6 ? days[selection] : 0;
}

bool valid(const Request &request) {
    return request.view == View::Period ? request.selection < 6 :
        request.view == View::Day && (!request.selection ||
            (request.selection >= 0x1000 && request.selection < 0xFFFF));
}

const Field *fields(View view, size_t &count) {
    count = view == View::Day ? sizeof(DAY_FIELDS) / sizeof(Field) : sizeof(PERIOD_FIELDS) / sizeof(Field);
    return view == View::Day ? DAY_FIELDS : PERIOD_FIELDS;
}

int collect(Snapshot &out, Request request, Budget budget) {
    out = {};
    out.request = request;
    for (auto &value : out.values) value = MISSING;
    for (auto &value : out.on) value = 0xFFFF;
    for (auto &value : out.off) value = 0xFFFF;
    if (!valid(request)) return fail(out, "report_invalid_selection", 400);
    if (!AirSenseState::device_standby()) return fail(out, "report_not_idle", 409);
    ReadBudget reads{budget};
    int code = request.view == View::Day ? collect_day(out, reads) : collect_period(out, reads);
    if ((reads.interrupted || !AirSenseState::device_standby()) &&
        (!out.error || strcmp(out.error, "report_restore_failed") != 0))
        return fail(out, "report_not_idle", 409);
    return code;
}

}  // namespace SleepReport
