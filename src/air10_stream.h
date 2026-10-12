#pragma once
#include <stddef.h>
#include <stdint.h>

namespace Air10Stream {

constexpr uint8_t FIELD_MAX = 8;
constexpr uint8_t SCHEMA_COUNT = 5;

struct Field {
    char name[4];
    uint8_t width;
};

struct Schema {
    char tag[4];
    Field fields[FIELD_MAX];
    uint8_t field_count;
    bool from_firmware;
};

struct Frame {
    uint8_t sequence;
    int32_t values[FIELD_MAX];
};

// Main-loop discovery once per device identity generation, without blocking RX.
void poll();

// Prioritize a consumer's schema; safe outside the main-loop discovery owner.
void request(const char *tag);
// A recorder snapshot is complete; an individual schema is usable as soon as
// resolved. Both reject stale identity generations.
bool snapshot(uint32_t generation, Schema (&out)[SCHEMA_COUNT]);
bool schema(const char *tag, Schema &out);

// Shared stock layouts and field decoding; no UART or cache mutation.
bool fallback_schema(Schema &schema, const char *tag, uint16_t mid, uint16_t vid);
bool schema_has_field(const Schema &schema, const char *name);
bool decode_frame(const uint8_t *payload, uint16_t len, const Schema &schema,
                  Frame &decoded);
bool decoded_value(const Schema &schema, const Frame &decoded,
                   const char *name, int16_t &value);
bool decoded_value32(const Schema &schema, const Frame &decoded,
                     const char *name, int32_t &value);

}  // namespace Air10Stream
