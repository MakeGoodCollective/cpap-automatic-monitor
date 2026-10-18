#pragma once
#include <Arduino.h>
#include <Print.h>

#define LOG_MAX_OUTPUTS 4

typedef enum {
    LOG_ERROR = 0,
    LOG_WARN  = 1,
    LOG_INFO  = 2,
    LOG_DEBUG = 3,
} log_level_t;

typedef enum {
    CAT_GENERAL = 0,
    CAT_OXI,
    CAT_TCP,
    CAT_WIFI,
    CAT_OTA,
    CAT_WEB,
    CAT_ARB,
    CAT_HEALTH,
    CAT_EXPORT,
    CAT_EDF,
    CAT_STREAM,
    CAT_STORAGE,
    CAT_TIME,
    CAT_CONFIG,
    CAT_REPORT,
    CAT_COUNT
} log_cat_t;

namespace Log {
    // Accumulate enabled profiling scopes without allocation or log output.
    class ScopedMicros {
    public:
        ScopedMicros(uint64_t &elapsed, bool enabled)
            : elapsed_(enabled ? &elapsed : nullptr), started_(enabled ? micros() : 0) {}
        ~ScopedMicros() { if (elapsed_) *elapsed_ += uint32_t(micros() - started_); }
        ScopedMicros(const ScopedMicros &) = delete;
        ScopedMicros &operator=(const ScopedMicros &) = delete;

    private:
        uint64_t *elapsed_;
        uint32_t started_;
    };

    void init();
    void boot();
    const char *reset_reason_name();

    bool configure_syslog(bool enabled, const char *host, uint16_t port,
                          const char *hostname);
    void poll();

    void logf(log_cat_t cat, log_level_t lvl, const char *fmt, ...);

    void set_level(log_level_t lvl);
    log_level_t get_level();

    void set_cat_level(log_cat_t cat, log_level_t lvl);
    log_level_t get_cat_level(log_cat_t cat);

    const char *level_name(log_level_t lvl);
    const char *cat_name(log_cat_t cat);

    // Sinks run from poll(), must not wait for transport capacity.
    void add_output(Print *out);
    void remove_output(Print *out);
}
