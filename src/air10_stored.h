#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Air10Stored {

constexpr size_t MAX_SAMPLES = 10;

struct Value {
    bool present;
    uint8_t byte_count;
    uint8_t sample_count;
    int16_t samples[MAX_SAMPLES];
};

enum class ReadResult : uint8_t { Ok, Unsupported, Failed };
// One UART transaction; retry/cancellation policy belongs to the caller.
ReadResult read(const char *tag, uint16_t day, Value &out,
                uint16_t timeout_ms, bool background = false,
                uint16_t hard_timeout_ms = 0);

bool parse_value(const uint8_t *payload, size_t payload_len, Value &out);
bool parse_response(const uint8_t *payload, size_t payload_len,
                    const char *tag, uint16_t day, Value &out);
bool unsupported_response(const uint8_t *payload, size_t payload_len,
                          const char *tag, uint16_t day);
bool contains_minute(const Value &value, uint16_t expected,
                     uint16_t tolerance);
bool contains_interval(const Value &starts, const Value &ends,
                       uint16_t expected_start, uint16_t expected_end,
                       uint16_t tolerance);

}  // namespace Air10Stored
