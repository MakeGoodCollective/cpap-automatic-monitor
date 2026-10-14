#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>
#include <string_view>

#include <ArduinoJson.h>
#include "memory_manager.h"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define AIRCANNECT_JSON_UTIL_HAS_ARDUINO 1
#else
#define AIRCANNECT_JSON_UTIL_HAS_ARDUINO 0
#endif

namespace aircannect {

class LargeTextBuffer;

// WebUI historically omits non-whitespace control characters.
inline size_t json_escape_char(uint8_t c, char (&out)[6], bool omit_controls = false) {
    char code = 0;
    switch (c) {
        case '"': code = '"'; break;
        case '\\': code = '\\'; break;
        case '\n': code = 'n'; break;
        case '\r': code = 'r'; break;
        case '\t': code = 't'; break;
        case '\b': if (!omit_controls) code = 'b'; break;
        case '\f': if (!omit_controls) code = 'f'; break;
    }
    if (code) { out[0] = '\\'; out[1] = code; return 2; }
    if (c >= 0x20) { out[0] = c; return 1; }
    if (omit_controls) return 0;
    static const char hex[] = "0123456789ABCDEF";
    out[0] = '\\'; out[1] = 'u'; out[2] = '0'; out[3] = '0';
    out[4] = hex[c >> 4]; out[5] = hex[c & 15];
    return 6;
}

template <typename Out>
void json_escape_append(Out &out, const char *value, size_t len,
                         bool omit_controls = false) {
    if (!value) return;
    for (size_t i = 0; i < len; i++) {
        char encoded[6];
        size_t count = json_escape_char(static_cast<uint8_t>(value[i]),
                                        encoded, omit_controls);
        for (size_t j = 0; j < count; j++) out += encoded[j];
    }
}

class JsonAllocator : public ArduinoJson::Allocator {
public:
    explicit JsonAllocator(bool internal_fallback = true) : internal_fallback_(internal_fallback) {}
    void *allocate(size_t size) override { return Memory::alloc_large(size, internal_fallback_); }
    void deallocate(void *ptr) override { Memory::free(ptr); }
    void *reallocate(void *ptr, size_t size) override {
        return Memory::realloc_large(ptr, size, internal_fallback_);
    }
private:
    bool internal_fallback_;
};

bool json_variant_to_string(JsonVariantConst value, std::string &out);
bool json_variant_to_uint32(JsonVariantConst value, uint32_t &out);
void append_json_escaped(std::string &out, const char *value, size_t len);
void append_json_escaped(std::string &out, std::string_view value);

#if AIRCANNECT_JSON_UTIL_HAS_ARDUINO
bool json_variant_to_string(JsonVariantConst value, String &out);
void json_add_string(String &out, const char *key, const char *value, bool comma = true);
void json_add_bool(String &out, const char *key, bool value, bool comma = true);
void json_add_int(String &out, const char *key, long value, bool comma = true);
#endif

void append_json_escaped(LargeTextBuffer &out, const char *value, size_t len);
void append_json_float(LargeTextBuffer &out, float value);
void json_add_string(LargeTextBuffer &out, const char *key, const char *value, bool comma = true);
void json_add_string_view(LargeTextBuffer &out, const char *key, std::string_view value, bool comma = true);
void json_add_bool(LargeTextBuffer &out, const char *key, bool value, bool comma = true);
void json_add_int(LargeTextBuffer &out, const char *key, long value, bool comma = true);
void json_add_float(LargeTextBuffer &out, const char *key, float value, bool comma = true);
void json_add_uint64(LargeTextBuffer &out, const char *key, uint64_t value, bool comma = true);

}  // namespace aircannect
