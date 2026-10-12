#include "air10_stream.h"

#include <limits.h>
#include <atomic>
#include <string.h>

#include "airsense_state.h"
#include "debug_log.h"
#include "hex_util.h"
#include "uart_arbiter.h"
#include "live_stream.h"

namespace Air10Stream {
namespace {

constexpr const char *SCHEMA_TAGS[SCHEMA_COUNT] = {"TCE", "APN", "CSN", "PBT", "BRH"};
constexpr uint32_t RETRY_MS = 5000;
constexpr uint16_t RESPONSE_SIZE = 160;

// Publish each bounded layout independently; recording takes a complete set.
Schema cached_schemas[SCHEMA_COUNT];
portMUX_TYPE cache_mux = portMUX_INITIALIZER_UNLOCKED;
uint32_t cache_generation = 0;
bool cache_ready = false;
uint8_t resolved_mask = 0;
std::atomic<uint8_t> requested_mask{0};
uint8_t next_schema = 0;
uart_transaction_t *query_ticket = nullptr;
uint32_t failed_at_ms[SCHEMA_COUNT] = {};
bool query_failed[SCHEMA_COUNT] = {};

static void reset_schema(Schema &schema, const char *tag) {
    memset(&schema, 0, sizeof(schema));
    memcpy(schema.tag, tag, 3);
}

static bool add_schema_field(Schema &schema, const char *name,
                             uint8_t width) {
    if (!name || strlen(name) != 3 || width == 0 ||
        schema.field_count >= FIELD_MAX) {
        return false;
    }
    Field &field = schema.fields[schema.field_count++];
    memcpy(field.name, name, 3);
    field.name[3] = 0;
    field.width = width;
    return true;
}

static bool set_schema(Schema &schema, const char *tag,
                       const char *definition) {
    reset_schema(schema, tag);
    if (!definition || !definition[0]) return false;

    char copy[96] = {};
    strncpy(copy, definition, sizeof(copy) - 1);
    char *save = nullptr;
    for (char *token = strtok_r(copy, " ", &save); token;
         token = strtok_r(nullptr, " ", &save)) {
        char *colon = strchr(token, ':');
        if (!colon || colon - token != 3) return false;
        *colon = 0;
        uint32_t width = 0;
        if (!aircannect::parse_hex(colon + 1, strlen(colon + 1), width) ||
            width == 0 || width > 8 ||
            !add_schema_field(schema, token, static_cast<uint8_t>(width))) {
            return false;
        }
    }
    return schema.field_count > 0;
}

static bool parse_schema(Schema &schema, const char *tag, const char *response) {
    char value[120] = {};
    const char *result = qframe_response_value(response);
    if (!result || strlen(result) >= sizeof(value)) return false;
    strcpy(value, result);

    char *save = nullptr;
    char *count_text = strtok_r(value, " ", &save);
    uint32_t count = 0;
    if (!count_text || !aircannect::parse_hex(count_text, strlen(count_text), count) ||
        count == 0 || count > FIELD_MAX) {
        return false;
    }

    reset_schema(schema, tag);
    for (uint32_t i = 0; i < count; i++) {
        char *token = strtok_r(nullptr, " ", &save);
        if (!token) return false;
        char *colon = strchr(token, ':');
        uint32_t width = 0;
        if (!colon || colon - token != 3) return false;
        *colon = 0;
        if (!aircannect::parse_hex(colon + 1, strlen(colon + 1), width) ||
            width == 0 || width > 8 ||
            !add_schema_field(schema, token, static_cast<uint8_t>(width))) {
            return false;
        }
    }
    schema.from_firmware = true;
    return strtok_r(nullptr, " ", &save) == nullptr;
}

static int stream_profile(uint16_t mid, uint16_t vid) {
    if (mid == 0x24) {
        switch (vid) {
            case 1: case 25: case 26: case 34: case 37: case 39: return 1;
            case 2: case 38: return 2;
            case 5: return 3;
            case 7: case 11: case 12: case 28: return 4;
            case 19: return 5;
            case 9: case 27: return 6;
            case 3: case 35: return 7;
            case 36: return 8;
            default: return 0;
        }
    }
    if (mid == 0x28) {
        switch (vid) {
            case 41: case 43: case 45: return 4;
            case 50: case 51: return 5;
            case 44: case 46: case 48: case 49: return 9;
            default: return 0;
        }
    }
    return 0;
}

}  // namespace

bool fallback_schema(Schema &schema, const char *tag, uint16_t mid, uint16_t vid) {
    const int profile = stream_profile(mid, vid);
    const char *definition = nullptr;
    if (!strcmp(tag, "TCE")) {
        definition = profile == 4 || profile == 6 || profile == 9
            ? "TCV:04 MKP:03 RFL:03 LYK:02" : "MKP:03 RFL:03 LYK:02";
    } else if (!strcmp(tag, "APN")) {
        definition = "AET:04 DUR:02";
    } else if (!strcmp(tag, "CSN") && profile != 3) {
        definition = profile == 4 || profile == 5 || profile == 6 || profile == 9
            ? "CSR:04" : "CET:05 CSR:04";
    } else if (!strcmp(tag, "PBT")) {
        if (profile == 7 || profile == 8) definition = "LKF:02 TIP:03 TEP:03";
        else if (profile == 5) definition = "MV5:02 TGT:02 RRR:02 LKF:02 TIP:03 TEP:03";
        else if (profile == 9) definition = "MV5:02 RRR:02 LKF:02 TIP:03 TEP:03 AAV:02";
        else if (profile) definition = "MV5:02 RRR:02 LKF:02 TIP:03 TEP:03";
    } else if (!strcmp(tag, "BRH")) {
        if (profile == 1 || profile == 5) definition = "TID:02 ATP:03";
        else if (profile == 2) definition = "TID:02";
        else if (profile == 3 || profile == 6) definition = "TID:02 ATP:03 INT:03 EXT:03";
        else if (profile == 4 || profile == 9) definition = "TID:02 INT:03 EXT:03";
        else if (profile == 8) definition = "ATP:03";
    }
    return set_schema(schema, tag, definition);
}

void request(const char *tag) {
    for (uint8_t i = 0; i < SCHEMA_COUNT; ++i)
        if (!strcmp(tag, SCHEMA_TAGS[i])) requested_mask.fetch_or(1u << i);
}

void poll() {
    const uint32_t generation = AirSenseState::identity_generation();
    if (generation != cache_generation) {
        if (query_ticket) Arbiter::cancel_transaction(query_ticket);
        query_ticket = nullptr;
        next_schema = 0;
        memset(query_failed, 0, sizeof(query_failed));
        portENTER_CRITICAL(&cache_mux);
        memset(cached_schemas, 0, sizeof(cached_schemas));
        cache_generation = generation;
        cache_ready = false;
        resolved_mask = 0;
        portEXIT_CRITICAL(&cache_mux);
    }
    if (cache_ready) return;

    const auto state = Arbiter::get_state();
    if (state != SYS_IDLE && state != SYS_THERAPY) {
        if (query_ticket) Arbiter::cancel_transaction(query_ticket);
        query_ticket = nullptr;
        return;
    }
    AirSenseState::Identity identity;
    if (!AirSenseState::identity(identity) || identity.generation != generation) return;
    if (!query_ticket) {
        int selected = -1;
        const uint8_t demand = requested_mask.load();
        for (uint8_t i = 0; i < SCHEMA_COUNT; ++i) {
            if ((resolved_mask & (1u << i)) ||
                (query_failed[i] && millis() - failed_at_ms[i] < RETRY_MS)) continue;
            if (selected < 0 || (demand & (1u << i))) selected = i;
            if (demand & (1u << i)) break;
        }
        if (selected < 0) return;
        next_schema = selected;
    }

    const char *tag = SCHEMA_TAGS[next_schema];
    Arbiter::VarReadTrace trace;
    uart_transaction_result_t result = {};
    char response[RESPONSE_SIZE] = {};
    uint16_t length = sizeof(response);
    bool ok = false;
    if (!query_ticket) {
        char command[16];
        snprintf(command, sizeof(command), "G C &%s", tag);
        const uart_send_window_t window = {0, 0, false, true};
        query_ticket = Arbiter::begin_cmd(command, CMD_SRC_INTERNAL, CMD_PRIO_LOW,
                                          sizeof(response), 500, window);
        if (query_ticket) return;
    } else {
        if (!Arbiter::transaction_done(query_ticket)) {
            if (!Arbiter::transaction_expired(query_ticket)) return;
            Arbiter::cancel_transaction(query_ticket);
            trace.outcome = "deadline";
        } else {
            ok = Arbiter::finish_cmd(query_ticket, response, &length, &result, &trace);
        }
        query_ticket = nullptr;
    }
    if (generation != AirSenseState::identity_generation()) return;

    Schema resolved = {};
    const char *value = qframe_response_value(response);
    const bool unsupported = result.terminal_type == QFRAME_TYPE_E &&
        length < sizeof(response) && value && !strcmp(value, "6009");
    if (unsupported) {
        // An absent optional stock stream is resolved, not a discovery failure.
        fallback_schema(resolved, tag, identity.mid, identity.vid);
    } else if (!ok || length >= sizeof(response) || !parse_schema(resolved, tag, response)) {
        Log::logf(CAT_STREAM, query_failed[next_schema] ? LOG_DEBUG : LOG_WARN,
                  "%s schema failed result=%s queue=%lums sent=%u wait=%lums response=%s\n",
                  tag, ok ? "invalid_schema" : trace.outcome,
                  (unsigned long)trace.queue_ms, unsigned(trace.sent),
                  (unsigned long)trace.wait_ms, response);
        failed_at_ms[next_schema] = millis();
        query_failed[next_schema] = true;
        return;
    }

    query_failed[next_schema] = false;
    portENTER_CRITICAL(&cache_mux);
    cached_schemas[next_schema] = resolved;
    resolved_mask |= 1u << next_schema;
    cache_ready = resolved_mask == (1u << SCHEMA_COUNT) - 1;
    portEXIT_CRITICAL(&cache_mux);
    Log::logf(CAT_STREAM, LOG_DEBUG, "%s schema from %s (%u fields)\n",
              tag, resolved.from_firmware ? "firmware" : "stock", resolved.field_count);
    LiveStream::resync(tag);
}

bool snapshot(uint32_t generation, Schema (&out)[SCHEMA_COUNT]) {
    portENTER_CRITICAL(&cache_mux);
    const bool ready = cache_ready && cache_generation == generation;
    if (ready) memcpy(out, cached_schemas, sizeof(cached_schemas));
    portEXIT_CRITICAL(&cache_mux);
    if (ready && generation == AirSenseState::identity_generation()) return true;
    memset(out, 0, sizeof(out));
    return false;
}

bool schema(const char *tag, Schema &out) {
    const uint32_t generation = AirSenseState::identity_generation();
    bool found = false;
    portENTER_CRITICAL(&cache_mux);
    if (cache_generation == generation) {
        for (const Schema &entry : cached_schemas) {
            if (entry.field_count && !strcmp(entry.tag, tag)) {
                out = entry;
                found = true;
                break;
            }
        }
    }
    portEXIT_CRITICAL(&cache_mux);
    if (found && generation == AirSenseState::identity_generation()) return true;
    out = {};
    return false;
}

bool schema_has_field(const Schema &schema, const char *name) {
    for (uint8_t i = 0; i < schema.field_count; i++)
        if (strcmp(schema.fields[i].name, name) == 0) return true;
    return false;
}

bool decode_frame(const uint8_t *payload, uint16_t len, const Schema &schema,
                         Frame &decoded) {
    size_t expected = 5;
    for (uint8_t i = 0; i < schema.field_count; i++)
        expected += schema.fields[i].width;
    if (len != expected || memcmp(payload, schema.tag, 3) != 0)
        return false;

    uint32_t sequence = 0;
    if (!aircannect::parse_hex(reinterpret_cast<const char *>(payload + 3),
                         2, sequence)) {
        return false;
    }
    decoded.sequence = static_cast<uint8_t>(sequence);

    size_t offset = 5;
    for (uint8_t i = 0; i < schema.field_count; i++) {
        const Field &field = schema.fields[i];
        uint32_t value = 0;
        if (!aircannect::parse_hex(
                reinterpret_cast<const char *>(payload + offset),
                field.width, value)) {
            return false;
        }
        if (strcmp(field.name, "RFL") == 0) {
            const uint8_t bits = field.width * 4;
            if (bits < 32 && (value & (1UL << (bits - 1))))
                value |= ~((1UL << bits) - 1);
        }
        decoded.values[i] = static_cast<int32_t>(value);
        offset += field.width;
    }
    return true;
}

bool decoded_value(const Schema &schema,
                          const Frame &decoded,
                          const char *name, int16_t &value) {
    for (uint8_t i = 0; i < schema.field_count; i++) {
        if (strcmp(schema.fields[i].name, name) != 0) continue;
        if (decoded.values[i] < INT16_MIN || decoded.values[i] > INT16_MAX)
            return false;
        value = static_cast<int16_t>(decoded.values[i]);
        return true;
    }
    return false;
}

bool decoded_value32(const Schema &schema,
                            const Frame &decoded,
                            const char *name, int32_t &value) {
    for (uint8_t i = 0; i < schema.field_count; i++) {
        if (strcmp(schema.fields[i].name, name) != 0) continue;
        value = decoded.values[i];
        return true;
    }
    return false;
}

}  // namespace Air10Stream
