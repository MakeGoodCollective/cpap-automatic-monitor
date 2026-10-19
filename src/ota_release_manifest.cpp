#include "ota_release_manifest.h"
#include "json_util.h"

#include <ArduinoJson.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

namespace OtaRelease {
namespace {

static constexpr uint32_t MANIFEST_SCHEMA = 1;
static constexpr char MANIFEST_PRODUCT[] = "airbridge";

struct ParsedVersion {
    uint32_t major = 0;
    uint32_t minor = 0;
    uint32_t patch = 0;
    const char *prerelease = nullptr;
    size_t prerelease_len = 0;
};

void set_error(char error[ERROR_MAX], const char *value) {
    snprintf(error, ERROR_MAX, "%s", value ? value : "manifest_invalid");
}

bool parse_number(const char *&cursor, const char *end, uint32_t &value) {
    if (cursor >= end || !isdigit((unsigned char)*cursor)) return false;

    uint64_t parsed = 0;
    while (cursor < end && isdigit((unsigned char)*cursor)) {
        parsed = parsed * 10u + (unsigned)(*cursor - '0');
        if (parsed > UINT32_MAX) return false;
        cursor++;
    }
    value = (uint32_t)parsed;
    return true;
}

bool valid_prerelease(const char *value, size_t len) {
    if (!value || len == 0) return false;
    bool identifier_has_char = false;
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)value[i];
        if (c == '.') {
            if (!identifier_has_char) return false;
            identifier_has_char = false;
        } else {
            if (!isalnum(c) && c != '-') return false;
            identifier_has_char = true;
        }
    }
    return identifier_has_char;
}

bool parse_semver(const char *value, size_t len, ParsedVersion &out) {
    out = {};
    if (!value || len == 0) return false;

    const char *cursor = value;
    const char *end = value + len;
    if (*cursor == 'v' || *cursor == 'V') cursor++;
    if (!parse_number(cursor, end, out.major) || cursor >= end ||
        *cursor++ != '.' || !parse_number(cursor, end, out.minor) ||
        cursor >= end || *cursor++ != '.' ||
        !parse_number(cursor, end, out.patch)) {
        return false;
    }

    if (cursor == end) return true;
    if (*cursor == '+') return ++cursor < end;
    if (*cursor++ != '-') return false;

    const char *prerelease = cursor;
    while (cursor < end && *cursor != '+') cursor++;
    size_t prerelease_len = (size_t)(cursor - prerelease);
    if (!valid_prerelease(prerelease, prerelease_len)) return false;
    if (cursor < end && ++cursor == end) return false;

    out.prerelease = prerelease;
    out.prerelease_len = prerelease_len;
    return true;
}

bool git_describe_suffix(const char *value, size_t len, size_t &base_len) {
    for (size_t i = len; i > 0; i--) {
        if (value[i - 1] != '-') continue;
        size_t cursor = i;
        size_t count_start = cursor;
        while (cursor < len && isdigit((unsigned char)value[cursor])) cursor++;
        if (cursor == count_start || cursor + 2 >= len ||
            value[cursor] != '-' || value[cursor + 1] != 'g') {
            continue;
        }
        cursor += 2;
        size_t hash_start = cursor;
        while (cursor < len && isxdigit((unsigned char)value[cursor])) cursor++;
        if (cursor == len && cursor > hash_start) {
            base_len = i - 1;
            return base_len > 0;
        }
    }
    return false;
}

bool parse_current(const char *value, ParsedVersion &out) {
    if (!value) return false;
    size_t len = strlen(value);
    static constexpr char DIRTY[] = "-dirty";
    if (len > sizeof(DIRTY) - 1 &&
        memcmp(value + len - (sizeof(DIRTY) - 1), DIRTY,
               sizeof(DIRTY) - 1) == 0) {
        len -= sizeof(DIRTY) - 1;
    }
    size_t base_len = 0;
    if (git_describe_suffix(value, len, base_len)) len = base_len;
    return parse_semver(value, len, out);
}

int compare_identifier(const char *left, size_t left_len,
                       const char *right, size_t right_len) {
    bool left_numeric = left_len > 0;
    bool right_numeric = right_len > 0;
    for (size_t i = 0; i < left_len; i++)
        left_numeric = left_numeric && isdigit((unsigned char)left[i]);
    for (size_t i = 0; i < right_len; i++)
        right_numeric = right_numeric && isdigit((unsigned char)right[i]);

    if (left_numeric != right_numeric) return left_numeric ? -1 : 1;
    if (left_numeric) {
        while (left_len > 1 && *left == '0') { left++; left_len--; }
        while (right_len > 1 && *right == '0') { right++; right_len--; }
        if (left_len != right_len) return left_len < right_len ? -1 : 1;
    }

    size_t common = left_len < right_len ? left_len : right_len;
    int compared = memcmp(left, right, common);
    if (compared != 0) return compared < 0 ? -1 : 1;
    if (left_len == right_len) return 0;
    return left_len < right_len ? -1 : 1;
}

int compare_prerelease(const ParsedVersion &left,
                       const ParsedVersion &right) {
    if (!left.prerelease && !right.prerelease) return 0;
    if (!left.prerelease) return 1;
    if (!right.prerelease) return -1;

    size_t left_offset = 0;
    size_t right_offset = 0;
    while (left_offset < left.prerelease_len &&
           right_offset < right.prerelease_len) {
        const char *left_value = left.prerelease + left_offset;
        const char *right_value = right.prerelease + right_offset;
        const char *left_dot = (const char *)memchr(
            left_value, '.', left.prerelease_len - left_offset);
        const char *right_dot = (const char *)memchr(
            right_value, '.', right.prerelease_len - right_offset);
        size_t left_len = left_dot ? (size_t)(left_dot - left_value)
                                   : left.prerelease_len - left_offset;
        size_t right_len = right_dot ? (size_t)(right_dot - right_value)
                                     : right.prerelease_len - right_offset;
        int compared = compare_identifier(left_value, left_len,
                                          right_value, right_len);
        if (compared != 0) return compared;
        left_offset += left_len + (left_dot ? 1 : 0);
        right_offset += right_len + (right_dot ? 1 : 0);
    }
    if (left_offset >= left.prerelease_len &&
        right_offset >= right.prerelease_len) return 0;
    return left_offset >= left.prerelease_len ? -1 : 1;
}

int compare_versions(const ParsedVersion &left, const ParsedVersion &right) {
    if (left.major != right.major) return left.major < right.major ? -1 : 1;
    if (left.minor != right.minor) return left.minor < right.minor ? -1 : 1;
    if (left.patch != right.patch) return left.patch < right.patch ? -1 : 1;
    return compare_prerelease(left, right);
}

bool valid_url_text(const char *url) {
    if (!url || !*url) return false;
    for (const unsigned char *p = (const unsigned char *)url; *p; p++)
        if (*p <= 0x20 || *p == 0x7f) return false;
    return true;
}

bool absolute_http_url(const char *url) {
    return url && (strncasecmp(url, "http://", 7) == 0 ||
                   strncasecmp(url, "https://", 8) == 0);
}

bool join_url(char *output, size_t capacity, const char *prefix,
              size_t prefix_len, const char *suffix) {
    size_t suffix_len = strlen(suffix);
    if (!output || capacity == 0 || prefix_len >= capacity ||
        suffix_len >= capacity - prefix_len) return false;
    memcpy(output, prefix, prefix_len);
    memcpy(output + prefix_len, suffix, suffix_len + 1);
    return true;
}

}  // namespace

bool parse_manifest(char *json, size_t json_len, const char *target,
                    Manifest &manifest, char error[ERROR_MAX]) {
    manifest = {};
    if (error) error[0] = '\0';
    if (!json || json_len == 0 || !target || !*target || !error) return false;

    aircannect::JsonAllocator allocator;
    JsonDocument filter(&allocator);
    filter["schema"] = true;
    filter["product"] = true;
    filter["version"] = true;
    filter["release_notes_url"] = true;
    for (const char *encoding : {"raw", "zlib"}) {
        JsonObject artifact_filter = filter["targets"][target][encoding].to<JsonObject>();
        artifact_filter["url"] = true;
        artifact_filter["size"] = true;
        artifact_filter["decoded_size"] = true;
    }
    if (filter.overflowed()) {
        set_error(error, "manifest_alloc_failed");
        return false;
    }
    JsonDocument document(&allocator);
    DeserializationError parse_error = deserializeJson(
        document, json, json_len, DeserializationOption::Filter(filter));
    if (parse_error) {
        set_error(error, parse_error == DeserializationError::NoMemory
                             ? "manifest_alloc_failed"
                             : "manifest_json_invalid");
        return false;
    }

    JsonObjectConst root = document.as<JsonObjectConst>();
    if (root.isNull() || root["schema"] != MANIFEST_SCHEMA) {
        set_error(error, "manifest_schema_unsupported");
        return false;
    }
    const char *product = root["product"].as<const char *>();
    if (!product || strcmp(product, MANIFEST_PRODUCT) != 0) {
        set_error(error, "manifest_product_mismatch");
        return false;
    }

    const char *version = root["version"].as<const char *>();
    ParsedVersion parsed_version;
    if (!version || strlen(version) >= sizeof(manifest.version) ||
        !parse_semver(version, strlen(version), parsed_version)) {
        set_error(error, "manifest_version_invalid");
        return false;
    }

    JsonObjectConst targets = root["targets"].as<JsonObjectConst>();
    JsonObjectConst target_object = targets[target].as<JsonObjectConst>();
    JsonObjectConst artifact = target_object["zlib"].as<JsonObjectConst>();
    const bool zlib = !artifact.isNull();
    if (!zlib) artifact = target_object["raw"].as<JsonObjectConst>();
    if (target_object.isNull() || artifact.isNull()) {
        set_error(error, "manifest_target_missing");
        return false;
    }

    const char *url = artifact["url"].as<const char *>();
    uint64_t size = artifact["size"].as<uint64_t>();
    uint64_t image_size = zlib ? artifact["decoded_size"].as<uint64_t>() : size;
    if (!url || !*url || strlen(url) >= sizeof(manifest.artifact.url) ||
        !artifact["size"].is<uint64_t>() || size == 0 || size > SIZE_MAX ||
        (zlib && !artifact["decoded_size"].is<uint64_t>()) ||
        image_size == 0 || image_size > SIZE_MAX) {
        set_error(error, "manifest_artifact_invalid");
        return false;
    }

    // Optional notes must not invalidate an otherwise usable firmware release.
    JsonString notes = root["release_notes_url"].as<JsonString>();
    const char *notes_url = notes.c_str();
    if (valid_url_text(notes_url) &&
        notes.size() < sizeof(manifest.release_notes_url) &&
        notes.size() == strlen(notes_url) &&
        !(notes_url[0] == '/' && notes_url[1] == '/')) {
        snprintf(manifest.release_notes_url, sizeof(manifest.release_notes_url),
                 "%s", notes_url);
    }

    snprintf(manifest.version, sizeof(manifest.version), "%s", version);
    snprintf(manifest.artifact.url, sizeof(manifest.artifact.url), "%s", url);
    manifest.artifact.size = (size_t)size;
    manifest.artifact.image_size = (size_t)image_size;
    manifest.artifact.zlib = zlib;
    return true;
}

bool is_newer(const char *current_version, const char *release_version,
              bool &newer) {
    newer = false;
    ParsedVersion current;
    ParsedVersion release;
    if (!parse_current(current_version, current) || !release_version ||
        !parse_semver(release_version, strlen(release_version), release)) {
        return false;
    }
    newer = compare_versions(release, current) > 0;
    return true;
}

bool resolve_artifact_url(const char *manifest_url, const char *artifact_url,
                          char *resolved_url, size_t resolved_capacity) {
    if (!valid_url_text(manifest_url) || !valid_url_text(artifact_url) ||
        !resolved_url || resolved_capacity == 0 ||
        !absolute_http_url(manifest_url)) return false;

    if (absolute_http_url(artifact_url)) {
        return join_url(resolved_url, resolved_capacity, artifact_url,
                        strlen(artifact_url), "");
    }
    if (artifact_url[0] == '/' && artifact_url[1] == '/') return false;

    const char *scheme = strstr(manifest_url, "://");
    const char *authority = scheme ? scheme + 3 : nullptr;
    if (!authority || !*authority) return false;
    const char *end = manifest_url + strlen(manifest_url);
    const char *query = strpbrk(authority, "?#");
    if (query) end = query;
    const char *path = (const char *)memchr(
        authority, '/', (size_t)(end - authority));

    if (artifact_url[0] == '/') {
        size_t origin_len = path ? (size_t)(path - manifest_url)
                                 : (size_t)(end - manifest_url);
        return join_url(resolved_url, resolved_capacity, manifest_url,
                        origin_len, artifact_url);
    }

    const char *directory_end = path;
    for (const char *cursor = path; cursor && cursor < end; cursor++)
        if (*cursor == '/') directory_end = cursor + 1;
    if (!directory_end) return false;
    return join_url(resolved_url, resolved_capacity, manifest_url,
                    (size_t)(directory_end - manifest_url), artifact_url);
}

}  // namespace OtaRelease
