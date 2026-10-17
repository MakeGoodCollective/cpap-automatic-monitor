#pragma once

#include <Arduino.h>
#include "clinical_settings.h"
#include "sleep_report.h"
#include <variant>

namespace ClinicalJobs {

using Handler = int (*)(const String &body, String &result);
enum class Kind : uint8_t { Read, Write, Report };
constexpr size_t MAX_BODY_SIZE = 2048;
void init(Handler handler);
void tick();  // Nonblocking result/cache cleanup from the main loop.
uint32_t revision();  // Changes after a completed result becomes available.
// Results outlive HTTP requests: 30 s unread, at least 2 s after completion
// once delivered. Callers must not resubmit writes after a retrieval failure.
bool submit(Kind kind, String &&body, uint32_t &id,
            SleepReport::Request report = {});

using Cursor = std::variant<ClinicalSettings::Cursor, SleepReport::Cursor>;

// Pins the immutable values and metadata until the HTTP response is destroyed.
class Result {
public:
    Result() = default;
    ~Result();
    Result(const Result &) = delete;
    Result &operator=(const Result &) = delete;
    Result(Result &&other) noexcept;
    bool available() const { return id_ != 0; }
    size_t length() const { return length_; }
    size_t read(Cursor &cursor, size_t offset, char *out, size_t capacity) const;
    void reset();

private:
    friend int poll(uint32_t id, Result &result);
    uint32_t id_ = 0;
    const char *data_ = nullptr;
    const ClinicalSettings::Snapshot *snapshot_ = nullptr;
    const SleepReport::Snapshot *report_ = nullptr;
    size_t length_ = 0;
};

int poll(uint32_t id, Result &result);  // 202 pending, 410 expired, 503 busy
// Job deadline minus reserve, independent of UART phase timeouts.
// Outside the worker, no job deadline applies (returns UINT16_MAX).
uint16_t remaining_ms(uint32_t reserve_ms = 0);

}
