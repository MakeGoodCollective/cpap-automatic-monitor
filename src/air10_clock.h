#pragma once

#include <stdint.h>
#include <string.h>

namespace Air10Clock {

struct Calendar {
    int year, month, day, hour, minute, second;
};

// Native civil time mapped to ESP uptime, not to NTP. Uncertainty describes
// the measured edge bracket; it does not include unmeasured long-term drift.
struct PhaseAnchor {
    int64_t civil_ms = 0;
    uint32_t captured_ms = 0;
    uint32_t generation = 0;
    uint16_t uncertainty_ms = 0;

    int64_t native_at_ms(uint32_t now_ms) const {
        return civil_ms + int32_t(now_ms - captured_ms);
    }
};

// Queued UART read of DAC/TIC, bracketed by DAC across midnight.
bool read(Calendar &out, uint16_t timeout_ms = 0, uint32_t *captured_ms = nullptr);

// Health pass after confirming presence; status_time only reads the cache.
void poll_status();
void status_time(char (&out)[20]);
void invalidate();

// Copy a recent idle measurement for a session; check it against clock resets
// without expiring the frozen session anchor during therapy.
bool phase_anchor(PhaseAnchor &out);
bool phase_unchanged(const PhaseAnchor &anchor);

// Main-loop maintenance: idle phase measurement/retry and deferred NTP writes.
void handle();
// Owner-published priority work; routine STR yields, mandatory journals do not.
bool background_work_pending();
// Manual requests bypass autosync, not standby/STR or post-therapy deferral.
void request_sync(bool manual = false);
// Main-loop notification of confirmed therapy end; starts a 60-second hold.
void request_post_therapy_sync();
bool pull_time(bool force = false);

inline bool parse_calendar(const char *dac, const char *tic, Calendar &out) {
    if (!dac || !tic || strlen(dac) != 8 || strlen(tic) != 6) return false;
    int date = 0, clock = 0;
    for (unsigned i = 0; i < 8; i++) {
        if (dac[i] < '0' || dac[i] > '9') return false;
        date = date * 10 + dac[i] - '0';
    }
    for (unsigned i = 0; i < 6; i++) {
        if (tic[i] < '0' || tic[i] > '9') return false;
        clock = clock * 10 + tic[i] - '0';
    }
    Calendar value = {date % 10000, date / 10000 % 100, date / 1000000,
                      clock / 10000, clock / 100 % 100, clock % 100};
    if (!value.year || value.month < 1 || value.month > 12 ||
        value.day < 1 || value.hour > 23 || value.minute > 59 || value.second > 59)
        return false;
    static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int limit = days[value.month - 1];
    if (value.month == 2 && value.year % 4 == 0 &&
        (value.year % 100 != 0 || value.year % 400 == 0)) limit++;
    if (value.day > limit) return false;
    out = value;
    return true;
}

inline int32_t civil_epoch_day(int year, unsigned month, unsigned day) {
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
    const unsigned day_of_year =
        (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const unsigned day_of_era = year_of_era * 365 + year_of_era / 4 -
                                year_of_era / 100 + day_of_year;
    return era * 146097 + static_cast<int>(day_of_era) - 719468;
}

// Civil seconds, not UTC: the device calendar has no timezone information.
inline int64_t civil_seconds(const Calendar &value) {
    return int64_t(civil_epoch_day(value.year, value.month, value.day)) * 86400 +
           value.hour * 3600 + value.minute * 60 + value.second;
}

inline uint16_t therapy_day(int64_t civil) {
    return static_cast<uint16_t>((civil - 43200) / 86400);
}

inline uint32_t milliseconds_to_noon(int64_t civil) {
    const int64_t next = (int64_t(therapy_day(civil)) + 1) * 86400 + 43200;
    return static_cast<uint32_t>((next - civil) * 1000);
}

struct Anchor {
    bool native_valid = false;
    int64_t native_start = 0;
    uint32_t captured_ms = 0;

    int64_t native_at(uint32_t now_ms) const {
        return native_start + static_cast<uint32_t>(now_ms - captured_ms) / 1000;
    }
};

}
