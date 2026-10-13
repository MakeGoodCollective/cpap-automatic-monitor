#include "airsense_state.h"
#include "uart_arbiter.h"
#include "device_uptime.h"
#include "air10_clock.h"
#include "hex_util.h"
#include "custom_settings.h"
#include "live_stream.h"
#include "edf_recorder.h"
#include "export_sync.h"
#include "web_ui.h"
#include "debug_log.h"
#include <atomic>
#include <limits.h>

namespace AirSenseState {
namespace {
constexpr uint32_t ROP_POLL_INTERVAL_MS = 5000;
constexpr uint32_t HEALTH_POLL_INTERVAL_MS = 10000;
constexpr uint16_t HEALTH_TIMEOUT_MS = 500;
constexpr uint32_t MHR_POLL_INTERVAL_MS = 30UL * 60 * 1000;

uint32_t last_rop_poll = 0, last_health_poll = 0, last_mhr_poll = 0;
uint32_t consecutive_timeouts = 0, airsense_seen_ms = 0;
bool airsense_present = false;
std::atomic<int> cached_rop{-1}, cached_mhr{-1}, cached_mop{-1};
std::atomic<bool> refresh_requested{true};
DeviceUptime::Tracker device_uptime;
Identity device_identity;
portMUX_TYPE identity_mux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<uint32_t> device_revision{0};
const char *identity_failed_tag = nullptr, *identity_failure = nullptr;

enum class ReadStep : uint8_t { None, Rop, Stk, Srn, Mid, Vid, Pna, Mop, Mhr, Bls };
ReadStep read_step = ReadStep::None;
uart_transaction_t *read_ticket = nullptr;
uint32_t read_generation = 0;
Identity identity_read;
bool health_requested = false, mhr_requested = false;

static const char *read_tag() {
    static const char *tags[] = {"", "ROP", "STK", "SRN", "MID", "VID", "PNA", "MOP", "MHR", "BLS"};
    return tags[static_cast<unsigned>(read_step)];
}

static bool accept_identity_field(const char *tag, const char *text,
                                   Arbiter::VarResult result, Arbiter::VarReadTrace &trace,
                                   char *out, size_t capacity, uint16_t *number = nullptr) {
    uint32_t parsed = 0;
    if (result == Arbiter::VarResult::Ok) {
        if (!text[0] || strlen(text) >= capacity || strchr(text, '\r') || strchr(text, '\n')) {
            result = Arbiter::VarResult::Failed;
            trace.outcome = "invalid_text";
        } else if (number && !aircannect::parse_hex(text, strlen(text), parsed)) {
            result = Arbiter::VarResult::Failed;
            trace.outcome = "invalid_hex";
        } else if (number && parsed > UINT16_MAX) {
            result = Arbiter::VarResult::Failed;
            trace.outcome = "out_of_range";
        }
    }
    if (result != Arbiter::VarResult::Ok) {
        const bool changed = !identity_failed_tag || strcmp(identity_failed_tag, tag) ||
            !identity_failure || strcmp(identity_failure, trace.outcome);
        identity_failed_tag = tag;
        identity_failure = trace.outcome;
        Log::logf(CAT_HEALTH, changed ? LOG_WARN : LOG_DEBUG,
                  "Identity #%s result=%s queue=%lums sent=%u wait=%lums value=%s\n",
                  tag, trace.outcome, (unsigned long)trace.queue_ms,
                  unsigned(trace.sent), (unsigned long)trace.wait_ms, text);
        return false;
    }
    memcpy(out, text, strlen(text) + 1);
    if (number) *number = static_cast<uint16_t>(parsed);
    return true;
}

static void accept_rop(const char *text, Arbiter::VarResult result,
                        const Arbiter::VarReadTrace &trace) {
    airsense_present = result == Arbiter::VarResult::Ok &&
        (!strcmp(text, "0000") || !strcmp(text, "0001"));
    if (airsense_present) {
        consecutive_timeouts = 0;
        airsense_seen_ms = millis();
        const int new_rop = text[3] - '0';
        const int prev_rop = cached_rop.exchange(new_rop);
        const auto state = Arbiter::get_state();
        if (new_rop == 1 && state == SYS_IDLE) {
            Arbiter::set_state(SYS_THERAPY);
            Log::logf(CAT_HEALTH, LOG_INFO, "Therapy started\n");
            ExportSync::therapy_started();
        } else if (new_rop == 0 && state == SYS_THERAPY) {
            Arbiter::set_state(SYS_IDLE);
            Log::logf(CAT_HEALTH, LOG_INFO, "Therapy ended\n");
            EdfRecorder::request_stop();
            Air10Clock::request_post_therapy_sync();
            mhr_requested = true;
        }
        if (new_rop != prev_rop) WebUI::push_status_event();
        health_requested |= new_rop != prev_rop;
        if (health_requested) last_health_poll = last_rop_poll;
        read_step = health_requested ? ReadStep::Stk : ReadStep::None;
        return;
    }

    ++consecutive_timeouts;
    Log::logf(CAT_HEALTH, LOG_DEBUG,
              "ROP poll failed (%lu consecutive) result=%s queue=%lums sent=%u wait=%lums value=%s\n",
              (unsigned long)consecutive_timeouts, trace.outcome,
              (unsigned long)trace.queue_ms, unsigned(trace.sent),
              (unsigned long)trace.wait_ms, text);
    if (consecutive_timeouts >= 3) {
        invalidate_identity();
        Arbiter::set_state(SYS_ERROR);
        Log::logf(CAT_HEALTH, LOG_WARN, "AirSense unavailable: UART unresponsive\n");
    }
    Air10Clock::invalidate();
    cached_mop = -1;
    read_step = ReadStep::None;
}

static void accept_read(const char *text, Arbiter::VarResult result,
                         Arbiter::VarReadTrace &trace) {
    const bool ok = result == Arbiter::VarResult::Ok;
    uint32_t raw = 0;
    switch (read_step) {
        case ReadStep::Rop:
            accept_rop(text, result, trace);
            break;
        case ReadStep::Stk:
            if (ok && DeviceUptime::parse(text, raw) && device_uptime.observe(raw, millis())) {
                Log::logf(CAT_HEALTH, LOG_INFO, "AirSense restart detected\n");
                invalidate_identity();
                Air10Clock::invalidate();
                CustomSettings::invalidate("STK reset");
                LiveStream::reattach();
                Air10Clock::request_sync();
                cached_mhr = cached_mop = -1;
                if (Arbiter::get_state() == SYS_THERAPY) Arbiter::set_state(SYS_IDLE);
                read_step = ReadStep::None;
                break;
            }
            identity(identity_read);
            read_step = !identity_read.valid ? ReadStep::Srn :
                !identity_read.pna[0] ? ReadStep::Pna : ReadStep::Mop;
            break;
        case ReadStep::Srn:
            read_step = accept_identity_field("SRN", text, result, trace,
                identity_read.srn, sizeof(identity_read.srn)) ? ReadStep::Mid : ReadStep::Mop;
            break;
        case ReadStep::Mid:
            read_step = accept_identity_field("MID", text, result, trace,
                identity_read.mid_text, sizeof(identity_read.mid_text), &identity_read.mid)
                ? ReadStep::Vid : ReadStep::Mop;
            break;
        case ReadStep::Vid:
        case ReadStep::Pna: {
            const bool core = read_step == ReadStep::Vid;
            const bool valid = core
                ? accept_identity_field("VID", text, result, trace, identity_read.vid_text,
                    sizeof(identity_read.vid_text), &identity_read.vid)
                : accept_identity_field("PNA", text, result, trace, identity_read.pna,
                    sizeof(identity_read.pna));
            if (valid) {
                portENTER_CRITICAL(&identity_mux);
                if (identity_read.generation == device_identity.generation) {
                    identity_read.valid = true;
                    device_identity = identity_read;
                    ++device_revision;
                }
                portEXIT_CRITICAL(&identity_mux);
                identity_failed_tag = identity_failure = nullptr;
            }
            read_step = core && valid ? ReadStep::Pna : ReadStep::Mop;
            break;
        }
        case ReadStep::Mop:
            cached_mop = ok && aircannect::parse_hex(text, strlen(text), raw) && raw <= INT_MAX
                ? static_cast<int>(raw) : -1;
            read_step = mhr_requested || cached_mhr.load() < 0 ||
                millis() - last_mhr_poll >= MHR_POLL_INTERVAL_MS ? ReadStep::Mhr : ReadStep::None;
            break;
        case ReadStep::Mhr:
            if (ok && aircannect::parse_hex(text, strlen(text), raw) && raw <= INT_MAX) {
                const int previous = cached_mhr.exchange(static_cast<int>(raw));
                last_mhr_poll = millis();
                mhr_requested = false;
                if (previous != static_cast<int>(raw)) WebUI::push_status_event();
            }
            read_step = ReadStep::None;
            break;
        case ReadStep::Bls:
            if (ok) {
                Log::logf(CAT_HEALTH, LOG_INFO, "Device responded, clearing error\n");
                consecutive_timeouts = 0;
                Arbiter::set_state(SYS_IDLE);
                CustomSettings::invalidate("UART recovery");
                LiveStream::reattach();
                request_refresh();
            }
            read_step = ReadStep::None;
            break;
        case ReadStep::None:
            break;
    }
}

}  // namespace

void request_refresh() { refresh_requested = true; }

bool identity(Identity &out) {
    portENTER_CRITICAL(&identity_mux);
    out = device_identity;
    portEXIT_CRITICAL(&identity_mux);
    return out.valid;
}

uint32_t identity_generation() {
    portENTER_CRITICAL(&identity_mux);
    const uint32_t generation = device_identity.generation;
    portEXIT_CRITICAL(&identity_mux);
    return generation;
}

uint32_t identity_revision() { return device_revision.load(); }

void invalidate_identity() {
    portENTER_CRITICAL(&identity_mux);
    const uint32_t generation = device_identity.generation + 1;
    device_identity = {};
    device_identity.generation = generation;
    ++device_revision;
    portEXIT_CRITICAL(&identity_mux);
    request_refresh();
    EdfRecorder::request_stop();
}

int rop() { return cached_rop.load(); }
int mhr() { return cached_mhr.load(); }
int mop() { return cached_mop.load(); }

bool present_recently() {
    return airsense_present && millis() - airsense_seen_ms <= HEALTH_POLL_INTERVAL_MS;
}

bool system_idle() { return Arbiter::get_state() == SYS_IDLE; }

bool device_standby() { return system_idle() && rop() == 0; }

bool local_background_allowed() {
    const auto state = Arbiter::get_state();
    return (state == SYS_IDLE || state == SYS_ERROR) && rop() != 1;
}

void poll() {
    const auto state = Arbiter::get_state();
    const bool allowed = state == SYS_IDLE || state == SYS_THERAPY || state == SYS_ERROR;
    if (!allowed || (read_step != ReadStep::None && read_generation != identity_generation())) {
        if (read_ticket) Arbiter::cancel_transaction(read_ticket);
        read_ticket = nullptr;
        read_step = ReadStep::None;
        if (!allowed) return;
    }

    if (read_step == ReadStep::None) {
        const uint32_t now = millis();
        const uint32_t interval = state == SYS_ERROR ? HEALTH_POLL_INTERVAL_MS : ROP_POLL_INTERVAL_MS;
        if (now - last_rop_poll < interval && !refresh_requested.load()) return;
        const bool refresh = refresh_requested.exchange(false);
        health_requested = refresh || now - last_health_poll >= HEALTH_POLL_INTERVAL_MS;
        last_rop_poll = now;
        read_generation = identity_generation();
        read_step = state == SYS_ERROR ? ReadStep::Bls : ReadStep::Rop;
    }

    char response[64] = {};
    Arbiter::VarReadTrace trace;
    auto result = Arbiter::VarResult::Failed;
    if (!read_ticket) {
        const bool urgent = read_step == ReadStep::Rop || read_step == ReadStep::Bls;
        const uart_send_window_t window = {0, 0, false, !urgent};
        read_ticket = Arbiter::begin_var(read_tag(), CMD_SRC_INTERNAL,
            urgent ? CMD_PRIO_HIGH : CMD_PRIO_NORMAL, sizeof(response), HEALTH_TIMEOUT_MS, window);
        if (read_ticket) return;
    } else {
        if (!Arbiter::transaction_done(read_ticket)) {
            if (!Arbiter::transaction_expired(read_ticket)) return;
            Arbiter::cancel_transaction(read_ticket);
            trace.outcome = "deadline";
        } else {
            result = Arbiter::finish_var(read_ticket, response, sizeof(response), &trace);
        }
        read_ticket = nullptr;
    }
    if (read_generation != identity_generation()) {
        read_step = ReadStep::None;
        return;
    }
    accept_read(response, result, trace);
    if (read_step == ReadStep::None && airsense_present && health_requested) {
        Air10Clock::poll_status();
        LiveStream::resync();
    }
}

}  // namespace AirSenseState
