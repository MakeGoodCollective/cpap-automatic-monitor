#include "air10_clock.h"
#include "app_config.h"
#include "airsense_state.h"
#include "uart_arbiter.h"
#include "wifi_setup.h"
#include "edf_recorder.h"
#include "debug_log.h"
#include "qframe.h"
#include "board.h"
#include <Arduino.h>
#include <atomic>
#include <esp_timer.h>
#include <sys/time.h>

static constexpr uint32_t DEVICE_TIME_POLL_INTERVAL_MS = 60000;
static constexpr uint32_t DEVICE_TIME_MAX_AGE_MS = 90000;
static constexpr uint16_t CLOCK_TIMEOUT_MS = 500;
static constexpr int64_t CLOCK_SEND_WINDOW_US = 10000;
static constexpr uint32_t PHASE_PROBE_BUDGET_MS = 1500;
static constexpr uint32_t PHASE_MAX_BRACKET_MS = 200;
static constexpr uint32_t PHASE_RETRY_INTERVAL_MS = 5000;
static constexpr uint32_t CLOCK_POST_THERAPY_DELAY_MS = 60000;

static constexpr uint8_t CLOCK_SYNC_AUTO = 1;
static constexpr uint8_t CLOCK_SYNC_MANUAL = 2;
static std::atomic<uint8_t> clock_sync_requests{CLOCK_SYNC_AUTO};
static std::atomic<bool> priority_work{false};
static bool clock_sync_pending = false;
static bool clock_sync_manual = false;
static bool clock_sync_settling = false;
static uint32_t clock_sync_therapy_end_ms = 0;
static uint32_t clock_sync_attempt_ms = 0;
static bool clock_sync_attempted = false;
static uint8_t clock_sync_window_retries = 0;
static uart_transaction_t *clock_sync_ticket = nullptr;
static time_t clock_sync_target = 0;
static int64_t clock_sync_edge_us = 0;
static char clock_sync_tic[16];
static portMUX_TYPE device_time_mux = portMUX_INITIALIZER_UNLOCKED;
static char device_time[20] = "--";
static uint32_t device_time_ms = 0;

struct ClockObservation {
    int64_t civil = 0;
    uint32_t sent_ms = 0;
    uint32_t received_ms = 0;
    char date[9] = {};
};

// Shared clock observations; only handle() owns probe tickets.
static Air10Clock::PhaseAnchor device_phase;
static uint32_t phase_generation = 1;
static ClockObservation last_clock_read;
static ClockObservation phase_previous;
static char phase_date[9];
static uint32_t phase_started_ms = 0, phase_probe_generation = 0;
static bool phase_measuring = false;
static uint32_t phase_attempt_ms = 0;
static bool phase_attempted = false;
static uart_transaction_t *phase_ticket = nullptr;

// One main-loop calendar read serves the display and the phase baseline.
static bool status_read_requested = false, calendar_for_phase = false;
static uart_transaction_t *calendar_ticket = nullptr;
static uint8_t calendar_step = 0, calendar_attempt = 0;
static uint32_t calendar_generation = 0;
static char calendar_date[9], calendar_tic[7];
static Arbiter::VarReadTrace calendar_trace;

void Air10Clock::status_time(char (&out)[20]) {
    system_state_t state = Arbiter::get_state();
    portENTER_CRITICAL(&device_time_mux);
    bool fresh = (state == SYS_IDLE || state == SYS_THERAPY) &&
        uint32_t(millis() - device_time_ms) < DEVICE_TIME_MAX_AGE_MS;
    memcpy(out, device_time, sizeof(out));
    portEXIT_CRITICAL(&device_time_mux);
    if (!fresh) strcpy(out, "--");
}

static void publish_device_time(const char *text) {
    portENTER_CRITICAL(&device_time_mux);
    snprintf(device_time, sizeof(device_time), "%s", text);
    device_time_ms = millis();
    portEXIT_CRITICAL(&device_time_mux);
}

void Air10Clock::invalidate() {
    portENTER_CRITICAL(&device_time_mux);
    device_phase = {};
    last_clock_read = {};
    phase_generation++;
    portEXIT_CRITICAL(&device_time_mux);
    publish_device_time("--");
}

bool Air10Clock::phase_anchor(PhaseAnchor &out) {
    portENTER_CRITICAL(&device_time_mux);
    out = device_phase;
    portEXIT_CRITICAL(&device_time_mux);
    if (out.generation && uint32_t(millis() - out.captured_ms) < DEVICE_TIME_MAX_AGE_MS)
        return true;
    out = {};
    return false;
}

bool Air10Clock::phase_unchanged(const PhaseAnchor &anchor) {
    portENTER_CRITICAL(&device_time_mux);
    const bool valid = anchor.generation && anchor.generation == phase_generation;
    portEXIT_CRITICAL(&device_time_mux);
    return valid;
}

static void observe_clock_read(const Air10Clock::Calendar &value,
                                const Arbiter::VarReadTrace &trace) {
    const int64_t civil = Air10Clock::civil_seconds(value);
    char date[9] = {}, text[20] = {};
    snprintf(date, sizeof(date), "%02d%02d%04d", value.day, value.month, value.year);
    snprintf(text, sizeof(text), "%04d-%02d-%02d %02d:%02d",
             value.year, value.month, value.day, value.hour, value.minute);
    portENTER_CRITICAL(&device_time_mux);
    if (device_phase.generation &&
        (device_phase.native_at_ms(trace.received_ms) + device_phase.uncertainty_ms < civil * 1000 ||
         device_phase.native_at_ms(trace.sent_ms) - device_phase.uncertainty_ms >= (civil + 1) * 1000)) {
        device_phase = {};
        phase_generation++;
    }
    last_clock_read = {civil, trace.sent_ms, trace.received_ms};
    memcpy(last_clock_read.date, date, sizeof(date));
    memcpy(device_time, text, sizeof(text));
    device_time_ms = trace.received_ms;
    portEXIT_CRITICAL(&device_time_mux);
}

bool Air10Clock::read(Calendar &out, uint16_t timeout_ms, uint32_t *captured_ms) {
    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        char before[9], tic[7], after[9];
        Arbiter::VarReadTrace trace;
        if (!Arbiter::get_var("DAC", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                              before, sizeof(before), timeout_ms) ||
            Arbiter::read_var("TIC", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                              tic, sizeof(tic), timeout_ms, &trace) != Arbiter::VarResult::Ok)
            return false;
        const uint32_t sampled_ms = millis();
        if (!Arbiter::get_var("DAC", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                              after, sizeof(after), timeout_ms)) return false;
        if (strcmp(before, after) != 0) continue;
        if (!parse_calendar(before, tic, out)) return false;
        observe_clock_read(out, trace);
        if (captured_ms) *captured_ms = sampled_ms;
        return true;
    }
    return false;
}

void Air10Clock::poll_status() {
    portENTER_CRITICAL(&device_time_mux);
    bool due = device_time[0] == '-' ||
        uint32_t(millis() - device_time_ms) >= DEVICE_TIME_POLL_INTERVAL_MS;
    portEXIT_CRITICAL(&device_time_mux);
    if (due) status_read_requested = true;
}

static void stop_phase_measurement(const char *reason = nullptr) {
    if (phase_measuring && reason)
        Log::logf(CAT_TIME, LOG_DEBUG, "RTC phase stopped: %s\n", reason);
    if (phase_ticket) Arbiter::cancel_transaction(phase_ticket);
    phase_ticket = nullptr;
    phase_measuring = false;
}

static void stop_calendar_read() {
    if (calendar_ticket) Arbiter::cancel_transaction(calendar_ticket);
    calendar_ticket = nullptr;
    calendar_step = 0;
}

static bool start_phase_from_observation() {
    portENTER_CRITICAL(&device_time_mux);
    const bool fresh = last_clock_read.date[0] &&
        millis() - last_clock_read.sent_ms <= PHASE_MAX_BRACKET_MS;
    if (fresh) {
        phase_previous = last_clock_read;
        phase_probe_generation = phase_generation;
        memcpy(phase_date, last_clock_read.date, sizeof(phase_date));
    }
    portEXIT_CRITICAL(&device_time_mux);
    phase_measuring = fresh;
    return fresh;
}

static void handle_calendar_read() {
    const auto state = Arbiter::get_state();
    portENTER_CRITICAL(&device_time_mux);
    const uint32_t generation = phase_generation;
    const bool phase_due = !device_phase.generation ||
        uint32_t(millis() - device_phase.captured_ms) >= DEVICE_TIME_POLL_INTERVAL_MS;
    portEXIT_CRITICAL(&device_time_mux);
    if ((state != SYS_IDLE && state != SYS_THERAPY) || clock_sync_ticket ||
        !AirSenseState::present_recently() ||
        (calendar_step && (calendar_generation != generation ||
            (calendar_for_phase && (!AirSenseState::device_standby() ||
                millis() - phase_started_ms >= PHASE_PROBE_BUDGET_MS))))) {
        stop_calendar_read();
        return;
    }
    if (phase_measuring) return;
    if (!calendar_step) {
        // A fallback, session read or write verification may have satisfied it.
        portENTER_CRITICAL(&device_time_mux);
        if (device_time[0] != '-' && millis() - device_time_ms < DEVICE_TIME_POLL_INTERVAL_MS)
            status_read_requested = false;
        portEXIT_CRITICAL(&device_time_mux);
        calendar_for_phase = AB_STORAGE_HAS_SDCARD && phase_due &&
            AirSenseState::device_standby() &&
            (!phase_attempted || millis() - phase_attempt_ms >= PHASE_RETRY_INTERVAL_MS);
        if (!status_read_requested && !calendar_for_phase) return;
        status_read_requested = false;
        calendar_generation = generation;
        calendar_attempt = 0;
        calendar_step = 1;
        if (calendar_for_phase) {
            phase_attempted = true;
            phase_started_ms = phase_attempt_ms = millis();
            if (start_phase_from_observation()) {
                calendar_step = 0;
                return;
            }
        }
    }
    if (!calendar_ticket) {
        const uart_send_window_t window = {0, 0, calendar_for_phase, true};
        calendar_ticket = Arbiter::begin_var(calendar_step == 2 ? "TIC" : "DAC",
            CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, 9, CLOCK_TIMEOUT_MS, window);
        if (calendar_ticket) return;
    } else if (!Arbiter::transaction_done(calendar_ticket)) {
        if (!Arbiter::transaction_expired(calendar_ticket)) return;
        stop_calendar_read();
    } else {
        char response[9] = {};
        Arbiter::VarReadTrace trace;
        const auto result = Arbiter::finish_var(calendar_ticket, response, sizeof(response), &trace);
        calendar_ticket = nullptr;
        if (result == Arbiter::VarResult::Ok) {
            if (calendar_step == 1) {
                memcpy(calendar_date, response, sizeof(calendar_date));
                calendar_step = 2;
                return;
            }
            if (calendar_step == 2 && strlen(response) < sizeof(calendar_tic)) {
                strcpy(calendar_tic, response);
                calendar_trace = trace;
                calendar_step = 3;
                return;
            }
            if (calendar_step == 3) {
                Air10Clock::Calendar value;
                if (strcmp(calendar_date, response) && calendar_attempt++ == 0) {
                    calendar_step = 1;
                    return;
                }
                if (!strcmp(calendar_date, response) &&
                    Air10Clock::parse_calendar(calendar_date, calendar_tic, value)) {
                    observe_clock_read(value, calendar_trace);
                    if (calendar_for_phase) start_phase_from_observation();
                    calendar_step = 0;
                    return;
                }
            }
        }
    }
    calendar_step = 0;
    publish_device_time("--");
    if (calendar_for_phase)
        Log::logf(CAT_TIME, LOG_DEBUG, "RTC phase failed: calendar read\n");
}

static void handle_phase_measurement() {
    if (!phase_measuring) return;
    portENTER_CRITICAL(&device_time_mux);
    const bool unchanged = phase_probe_generation == phase_generation;
    portEXIT_CRITICAL(&device_time_mux);
    const char *blocked = !unchanged ? "clock invalidated" :
        !AirSenseState::device_standby() || !AirSenseState::present_recently() ? "not in standby" :
        clock_sync_ticket ? "clock write" :
        uint32_t(millis() - phase_started_ms) >= PHASE_PROBE_BUDGET_MS ? "probe deadline" : nullptr;
    if (blocked) {
        stop_phase_measurement(blocked);
        return;
    }

    if (phase_ticket) {
        if (!Arbiter::transaction_done(phase_ticket)) return;
        char response[32] = {};
        uint16_t length = sizeof(response);
        Arbiter::VarReadTrace trace;
        const bool ok = Arbiter::finish_cmd(phase_ticket, response, &length, nullptr, &trace);
        phase_ticket = nullptr;
        Air10Clock::Calendar value;
        const char *tic = qframe_response_value(response);
        if (!ok || !trace.sent || length >= sizeof(response) ||
            !Air10Clock::parse_calendar(phase_date, tic, value)) {
            stop_phase_measurement("TIC read failed");
            return;
        }
        int64_t civil = Air10Clock::civil_seconds(value);
        if (civil == phase_previous.civil - 86399) civil += 86400;
        if (civil != phase_previous.civil) {
            const uint32_t width = trace.received_ms - phase_previous.sent_ms;
            if (civil == phase_previous.civil + 1 && width <= PHASE_MAX_BRACKET_MS) {
                // The edge is after the old read's TX and before the new RX.
                // RX(old)..RX(new) would claim precision the protocol lacks.
                Air10Clock::PhaseAnchor measured;
                measured.civil_ms = civil * 1000;
                measured.captured_ms = phase_previous.sent_ms + width / 2;
                measured.uncertainty_ms = (width + 1) / 2;
                measured.generation = phase_probe_generation;
                portENTER_CRITICAL(&device_time_mux);
                if (phase_generation == measured.generation) device_phase = measured;
                portEXIT_CRITICAL(&device_time_mux);
                Log::logf(CAT_TIME, LOG_DEBUG,
                          "RTC phase civil=%lld mono=%lu uncertainty=%ums\n",
                          (long long)civil, (unsigned long)measured.captured_ms,
                          unsigned(measured.uncertainty_ms));
            } else if (civil != phase_previous.civil + 1) {
                Log::logf(CAT_TIME, LOG_DEBUG, "RTC phase rejected: clock discontinuity\n");
                Air10Clock::invalidate();
            } else {
                Log::logf(CAT_TIME, LOG_DEBUG, "RTC phase rejected: bracket=%lums\n",
                          (unsigned long)width);
            }
            stop_phase_measurement();
            return;
        }
        phase_previous = {civil, trace.sent_ms, trace.received_ms};
    }

    const uart_send_window_t idle_only = {0, 0, true};
    phase_ticket = Arbiter::begin_cmd("G S #TIC", CMD_SRC_INTERNAL,
        CMD_PRIO_NORMAL, 32, CLOCK_TIMEOUT_MS, idle_only);
    if (!phase_ticket) stop_phase_measurement("UART queue unavailable");
}

void Air10Clock::request_sync(bool manual) {
    clock_sync_requests.fetch_or(manual ? CLOCK_SYNC_MANUAL : CLOCK_SYNC_AUTO);
}

void Air10Clock::request_post_therapy_sync() {
    clock_sync_therapy_end_ms = millis();
    clock_sync_settling = true;
    request_sync();
    Log::logf(CAT_TIME, LOG_DEBUG, "ResMed clock sync deferred: 60s after therapy\n");
}

static bool clock_write_applied(const char *command, bool ok, const char *response) {
    const char *value = qframe_response_value(response);
    // The device can refuse the change and return R with its unchanged value.
    if (ok && value && strcmp(value, command + 9) == 0) return true;

    const bool rejected = !ok && response[0];
    if (rejected) clock_sync_pending = false;
    Log::logf(CAT_TIME, LOG_WARN, "ResMed clock write %s: %s -> %s\n",
              rejected ? "rejected (use TIMESYNC to retry)" : "not applied; will retry",
              command, response[0] ? response : "timeout");
    if (ok && value && Log::get_cat_level(CAT_TIME) >= LOG_DEBUG) {
        // Native setters may leave the value unchanged despite an R reply.
        // Observe their state guards after refusal, without inferring a cause.
        static const char *tags[] = {"ZRM", "MST"};
        for (const char *tag : tags) {
            char state[16] = {};
            Arbiter::VarReadTrace trace;
            (void)Arbiter::read_var(tag, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                                   state, sizeof(state), CLOCK_TIMEOUT_MS, &trace);
            Log::logf(CAT_TIME, LOG_DEBUG,
                      "Clock refusal #%s=%s result=%s tx=%lu rx=%lu queue=%lums wait=%lums\n",
                      tag, state[0] ? state : "--", trace.outcome,
                      (unsigned long)trace.sent_ms, (unsigned long)trace.received_ms,
                      (unsigned long)trace.queue_ms, (unsigned long)trace.wait_ms);
        }
    }
    return false;
}

static bool write_clock_value(const char *command) {
    char response[64] = {};
    uint16_t length = sizeof(response);
    bool ok = Arbiter::send_cmd(command, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                                response, &length, CLOCK_TIMEOUT_MS);
    return clock_write_applied(command, ok, response);
}

static void push_time_to_resmed() {
    stop_phase_measurement("clock write");
    stop_calendar_read();
    Air10Clock::invalidate();
    struct tm t;
    time_t now = time(nullptr);
    localtime_r(&now, &t);

    char dac_cmd[32];
    snprintf(dac_cmd, sizeof(dac_cmd), "P S #DAC %02d%02d%04d",
             t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);

    if (!write_clock_value(dac_cmd)) return;
    publish_device_time("--");

    // Sample after DAC; waiting for the next second belongs to the UART queue.
    struct timeval wall;
    const int64_t before_us = esp_timer_get_time();
    gettimeofday(&wall, nullptr);
    const int64_t after_us = esp_timer_get_time();
    if (after_us - before_us >= 1000) return;
    clock_sync_target = wall.tv_sec + 1;
    struct tm target;
    localtime_r(&clock_sync_target, &target);
    // If DAC straddled midnight, retry the date/time pair on the new day.
    if (target.tm_year != t.tm_year || target.tm_yday != t.tm_yday) return;

    clock_sync_edge_us = after_us + 1000000 - wall.tv_usec;
    snprintf(clock_sync_tic, sizeof(clock_sync_tic), "P S #TIC %02d%02d%02d",
             target.tm_hour, target.tm_min, target.tm_sec);
    const uart_send_window_t window = {
        clock_sync_edge_us, clock_sync_edge_us + CLOCK_SEND_WINDOW_US, true,
    };
    clock_sync_ticket = Arbiter::begin_cmd(clock_sync_tic, CMD_SRC_INTERNAL,
        CMD_PRIO_NORMAL, 64, CLOCK_TIMEOUT_MS, window);
}

static bool finish_clock_sync() {
    char response[64] = {};
    uint16_t length = sizeof(response);
    uart_transaction_result_t result = {};
    Arbiter::VarReadTrace trace;
    bool ok = Arbiter::finish_cmd(clock_sync_ticket, response, &length, &result, &trace);
    clock_sync_ticket = nullptr;
    if (result.send_window_missed) {
        const bool retry_edge = clock_sync_window_retries++ < 2;
        if (retry_edge) clock_sync_attempted = false;
        Log::logf(CAT_TIME, LOG_DEBUG, "ResMed clock TX window missed; retry=%s\n",
                  retry_edge ? "next edge" : "backoff");
        return false;
    }
    Log::logf(CAT_TIME, LOG_DEBUG,
              "ResMed clock TIC=%s edge_us=%lld sent=%u TX late=%ldus tx=%lu rx=%lu result=%s\n",
              clock_sync_tic + 9, (long long)clock_sync_edge_us,
              unsigned(trace.sent), (long)result.send_lateness_us,
              (unsigned long)trace.sent_ms, (unsigned long)trace.received_ms, trace.outcome);
    if (!clock_write_applied(clock_sync_tic, ok, response)) return false;

    Air10Clock::Calendar observed;
    if (!Air10Clock::read(observed, CLOCK_TIMEOUT_MS)) {
        Log::logf(CAT_TIME, LOG_WARN, "ResMed clock verification read failed\n");
        return false;
    }

    struct tm t;
    localtime_r(&clock_sync_target, &t);
    const Air10Clock::Calendar requested = {
        t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
        t.tm_hour, t.tm_min, t.tm_sec,
    };
    const int64_t advance = Air10Clock::civil_seconds(observed) -
                            Air10Clock::civil_seconds(requested);
    // RTC reads have whole-second precision; bound advancement by UART time.
    const int64_t elapsed_us = esp_timer_get_time() - clock_sync_edge_us;
    const int64_t max_advance = elapsed_us / 1000000 + 1;
    char text[20];
    snprintf(text, sizeof(text), "%04d-%02d-%02d %02d:%02d",
             observed.year, observed.month, observed.day, observed.hour, observed.minute);
    // A wall-clock step while the ticket waited also requires a fresh attempt.
    struct timeval wall;
    gettimeofday(&wall, nullptr);
    const int64_t wall_elapsed_us = (int64_t(wall.tv_sec) - clock_sync_target) * 1000000 + wall.tv_usec;
    if (advance < 0 || advance > max_advance ||
        llabs(wall_elapsed_us - elapsed_us) > CLOCK_SEND_WINDOW_US) {
        Log::logf(CAT_TIME, LOG_WARN,
                  "ResMed clock verification failed; will retry: requested=%04d-%02d-%02d %s actual=%s:%02d\n",
                  requested.year, requested.month, requested.day,
                  clock_sync_tic + 9, text, observed.second);
        return false;
    }

    Log::logf(CAT_TIME, LOG_INFO, "ResMed clock synchronized: %s:%02d\n",
              text, observed.second);
    return true;
}

bool Air10Clock::pull_time(bool force) {
    Air10Clock::Calendar t;
    if (!Air10Clock::read(t) ||
        !WiFiSetup::set_fallback_time(t.year, t.month, t.day,
                                      t.hour, t.minute, t.second, force)) return false;
    return true;
}

static void handle_clock_sync() {
    if (clock_sync_settling &&
        millis() - clock_sync_therapy_end_ms >= CLOCK_POST_THERAPY_DELAY_MS)
        clock_sync_settling = false;

    const bool auto_enabled = Config::get().resmed_time;
    static bool was_auto_enabled = true;
    if (auto_enabled && !was_auto_enabled) Air10Clock::request_sync();
    was_auto_enabled = auto_enabled;

    const uint8_t requests = clock_sync_requests.exchange(0);
    if (requests) {
        if (clock_sync_ticket) {
            Arbiter::cancel_transaction(clock_sync_ticket);
            EdfRecorder::end_clock_write();
        }
        clock_sync_ticket = nullptr;
        clock_sync_manual = (requests & CLOCK_SYNC_MANUAL) ||
                            (clock_sync_pending && clock_sync_manual);
        clock_sync_pending = true;
        clock_sync_attempted = false;
        clock_sync_window_retries = 0;
    }
    if (!clock_sync_pending) return;
    const bool disabled = !auto_enabled && !clock_sync_manual;
    if (disabled || !AirSenseState::present_recently() || !WiFiSetup::time_synced() ||
        !AirSenseState::device_standby()) {
        if (clock_sync_ticket) {
            Arbiter::cancel_transaction(clock_sync_ticket);
            EdfRecorder::end_clock_write();
        }
        clock_sync_ticket = nullptr;
        if (disabled) clock_sync_pending = false;
        return;
    }
    if (clock_sync_settling) return;
    if (clock_sync_ticket) {
        if (Arbiter::transaction_done(clock_sync_ticket)) {
            if (finish_clock_sync()) clock_sync_pending = false;
            EdfRecorder::end_clock_write();
        }
        return;
    }
    if (clock_sync_attempted && millis() - clock_sync_attempt_ms < 30000) return;
    if (clock_sync_attempted) clock_sync_window_retries = 0;
    static const char *last_blocked = nullptr;
    const char *blocked = nullptr;
    if (!EdfRecorder::begin_clock_write(&blocked)) {
        if (blocked && blocked != last_blocked)
            Log::logf(CAT_TIME, LOG_DEBUG, "ResMed clock sync deferred: %s\n", blocked);
        last_blocked = blocked;
        return;
    }
    last_blocked = nullptr;
    clock_sync_attempt_ms = millis();
    clock_sync_attempted = true;
    push_time_to_resmed();
    if (!clock_sync_ticket) EdfRecorder::end_clock_write();
}

void Air10Clock::handle() {
    // An eligible write invalidates phase: do it before acquiring that phase.
    // A deferred/refused write must not prevent independent RTC observation.
    priority_work = (clock_sync_requests.load() || clock_sync_pending) &&
        (Config::get().resmed_time || clock_sync_manual ||
         (clock_sync_requests.load() & CLOCK_SYNC_MANUAL)) &&
        !clock_sync_settling && AirSenseState::device_standby() &&
        AirSenseState::present_recently() && WiFiSetup::time_synced() &&
        (!clock_sync_attempted || millis() - clock_sync_attempt_ms >= 30000);
    handle_clock_sync();
    handle_calendar_read();
    handle_phase_measurement();
    priority_work = priority_work.load() || clock_sync_ticket || calendar_step || phase_measuring;
}

bool Air10Clock::background_work_pending() { return priority_work.load(); }
