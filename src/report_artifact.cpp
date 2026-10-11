#include "report_artifact.h"
#include "crc.h"
#include "sd_storage.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace ReportArtifact {
namespace {
using SdStorage::get_le16;
using SdStorage::get_le32;
using SdStorage::put_le16;
using SdStorage::put_le32;

uint64_t get64(const uint8_t *data) {
    return uint64_t(get_le32(data)) | uint64_t(get_le32(data + 4)) << 32;
}
void put64(uint8_t *data, uint64_t value) {
    put_le32(data, uint32_t(value));
    put_le32(data + 4, uint32_t(value >> 32));
}
double get_double(const uint8_t *data) {
    const uint64_t bits = get64(data);
    double value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}
void put_double(uint8_t *data, double value) {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
                  "artifact coefficients require IEEE754 binary64");
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put64(data, bits);
}

uint32_t metadata_crc(const uint8_t *data, size_t bytes) {
    const uint8_t zero[4] = {};
    uint32_t crc = crc32_ieee_update(crc32_ieee_initial(), data, 60);
    crc = crc32_ieee_update(crc, zero, sizeof(zero));
    return crc32_ieee_finish(crc32_ieee_update(crc, data + 64, bytes - 64));
}

bool valid_track(const Track &track) {
    if (!track.samples_per_record || !track.record_us ||
        track.record_us > 86400000000ull || track.start_ms < 0 ||
        track.start_ms > 9007199254740991ll / 1000 ||
        track.digital_min < INT16_MIN || track.digital_max > INT16_MAX ||
        track.digital_max <= track.digital_min ||
        !std::isfinite(track.physical_min) || !std::isfinite(track.physical_max) ||
        track.physical_max <= track.physical_min ||
        !memchr(track.unit, 0, sizeof(track.unit)) ||
        !memchr(track.label, 0, sizeof(track.label))) return false;
    // At most 24 h of samples, and at most 1 kHz, including absent/empty tracks.
    return track.record_us >= uint64_t(track.samples_per_record) * 1000 &&
        uint64_t(track.count) <= 86400000000ull * track.samples_per_record / track.record_us;
}

bool geometry(Metadata &out, bool initialize) {
    if (!out.track_count || out.track_count > MAX_TRACKS) return false;
    uint32_t header = HEADER_BYTES;
    for (size_t c = 0; c < out.track_count; ++c) {
        auto &track = out.tracks[c];
        if (!valid_track(track)) return false;
        for (size_t i = 0; i < c; ++i) if (out.tracks[i].id == track.id) return false;
        uint32_t count = track.count;
        uint16_t levels = 1;
        while (count > TILE_SAMPLES) {
            count = count / 4 + (count % 4 != 0);
            if (++levels > MAX_LEVELS) return false;
        }
        if (initialize) track.level_count = levels;
        else if (track.level_count != levels) return false;
        header += TRACK_BYTES + levels * LEVEL_BYTES;
    }
    if (initialize) out.header_bytes = header;
    else if (out.header_bytes != header) return false;
    uint64_t offset = header;
    for (size_t c = 0; c < out.track_count; ++c) {
        auto &track = out.tracks[c];
        uint32_t count = track.count, factor = 1;
        for (size_t l = 0; l < track.level_count; ++l) {
            const uint64_t bytes = uint64_t(tile_count(count)) *
                (l ? REDUCED_TILE_BYTES : RAW_TILE_BYTES);
            if (offset + bytes > UINT32_MAX) return false;
            auto &level = track.levels[l];
            if (initialize) level = {factor, count, uint32_t(offset), uint32_t(bytes)};
            else if (level.factor != factor || level.count != count ||
                     level.offset != offset || level.bytes != bytes) return false;
            offset += bytes;
            count = count / 4 + (count % 4 != 0);
            factor *= 4;
        }
    }
    if (initialize) out.bytes = offset;
    return out.bytes == offset;
}
}  // namespace

uint32_t tile_count(uint32_t positions) {
    return positions / TILE_SAMPLES + (positions % TILE_SAMPLES != 0);
}
uint32_t expected(const Track &track, const Level &level, uint32_t position) {
    const uint64_t begin = uint64_t(position) * level.factor;
    return begin < track.count ? std::min<uint64_t>(level.factor, track.count - begin) : 0;
}
bool layout(Metadata &metadata) { return geometry(metadata, true); }

bool encode(const Metadata &metadata, uint8_t *out, size_t capacity) {
    // Validate without copying a multi-kilobyte descriptor onto a task stack.
    if (!out || metadata.header_bytes < HEADER_BYTES ||
        metadata.header_bytes > MAX_METADATA_BYTES || capacity < metadata.header_bytes ||
        !metadata.track_count || metadata.track_count > MAX_TRACKS) return false;
    memset(out, 0, metadata.header_bytes);
    memcpy(out, "ABTILE01", 8);
    put_le16(out + 8, VERSION);
    put_le16(out + 10, metadata.header_bytes);
    put_le32(out + 12, metadata.bytes);
    put_le16(out + 16, metadata.track_count);
    put_le16(out + 18, TILE_SAMPLES);
    memcpy(out + 20, metadata.source, sizeof(metadata.source));
    size_t at = HEADER_BYTES;
    for (size_t c = 0; c < metadata.track_count; ++c) {
        const auto &track = metadata.tracks[c];
        if (!valid_track(track) || !track.level_count || track.level_count > MAX_LEVELS ||
            at + TRACK_BYTES + track.level_count * LEVEL_BYTES > metadata.header_bytes) return false;
        uint8_t *p = out + at;
        put_le16(p, track.id);
        put_le16(p + 2, track.level_count);
        put_le32(p + 4, track.count);
        put64(p + 8, track.start_ms);
        put64(p + 16, track.record_us);
        put_le32(p + 24, track.samples_per_record);
        put_le32(p + 28, uint32_t(track.digital_min));
        put_le32(p + 32, uint32_t(track.digital_max));
        put_double(p + 40, track.physical_min);
        put_double(p + 48, track.physical_max);
        memcpy(p + 56, track.unit, sizeof(track.unit));
        memcpy(p + 72, track.label, sizeof(track.label));
        at += TRACK_BYTES;
        for (size_t l = 0; l < track.level_count; ++l, at += LEVEL_BYTES) {
            const auto &level = track.levels[l];
            put_le32(out + at, level.factor);
            put_le32(out + at + 4, level.count);
            put_le32(out + at + 8, level.offset);
            put_le32(out + at + 12, level.bytes);
        }
    }
    if (at != metadata.header_bytes) return false;
    put_le32(out + 60, metadata_crc(out, at));
    return true;
}

bool decode(const uint8_t *data, size_t length, uint64_t file_bytes, Metadata &out) {
    out = {};
    if (!data || length < HEADER_BYTES || memcmp(data, "ABTILE01", 8) ||
        get_le16(data + 8) != VERSION || get_le16(data + 18) != TILE_SAMPLES ||
        get_le32(data + 12) != file_bytes) return false;
    const size_t bytes = get_le16(data + 10);
    if (bytes < HEADER_BYTES || bytes > MAX_METADATA_BYTES || bytes > length ||
        bytes > file_bytes || metadata_crc(data, bytes) != get_le32(data + 60)) return false;
    for (size_t i = 52; i < 60; ++i) if (data[i]) return false;
    out.track_count = get_le16(data + 16);
    out.header_bytes = bytes;
    out.bytes = file_bytes;
    memcpy(out.source, data + 20, sizeof(out.source));
    if (!out.track_count || out.track_count > MAX_TRACKS) return false;
    size_t at = HEADER_BYTES;
    for (size_t c = 0; c < out.track_count; ++c) {
        if (at + TRACK_BYTES > bytes) return false;
        auto &track = out.tracks[c];
        const uint8_t *p = data + at;
        track.id = get_le16(p);
        track.level_count = get_le16(p + 2);
        track.count = get_le32(p + 4);
        track.start_ms = int64_t(get64(p + 8));
        track.record_us = get64(p + 16);
        track.samples_per_record = get_le32(p + 24);
        track.digital_min = int32_t(get_le32(p + 28));
        track.digital_max = int32_t(get_le32(p + 32));
        track.physical_min = get_double(p + 40);
        track.physical_max = get_double(p + 48);
        memcpy(track.unit, p + 56, sizeof(track.unit));
        memcpy(track.label, p + 72, sizeof(track.label));
        if (get_le32(p + 36) || !track.level_count || track.level_count > MAX_LEVELS) return false;
        at += TRACK_BYTES;
        for (size_t l = 0; l < track.level_count; ++l, at += LEVEL_BYTES) {
            if (at + LEVEL_BYTES > bytes) return false;
            track.levels[l] = {get_le32(data + at), get_le32(data + at + 4),
                get_le32(data + at + 8), get_le32(data + at + 12)};
        }
    }
    return at == bytes && geometry(out, false);
}

void put_raw(uint8_t *tile, size_t position, int16_t raw, bool valid) {
    put_le16(tile + position * 2, uint16_t(raw));
    auto &bits = tile[TILE_SAMPLES * 2 + position / 8];
    const uint8_t mask = 1u << (position % 8);
    bits = valid ? bits | mask : bits & ~mask;
}
Cell raw_cell(const uint8_t *tile, size_t position) {
    const auto raw = int16_t(get_le16(tile + position * 2));
    return {raw, raw, uint32_t((tile[TILE_SAMPLES * 2 + position / 8] >> (position % 8)) & 1)};
}
void put_cell(uint8_t *tile, size_t position, const Cell &cell) {
    uint8_t *p = tile + position * 8;
    put_le16(p, uint16_t(cell.low));
    put_le16(p + 2, uint16_t(cell.high));
    put_le32(p + 4, cell.valid);
}
Cell get_cell(const uint8_t *tile, size_t position) {
    const uint8_t *p = tile + position * 8;
    return {int16_t(get_le16(p)), int16_t(get_le16(p + 2)), get_le32(p + 4)};
}
bool merge(Cell &out, const Cell &next) {
    if (!next.valid) return true;
    if (next.low > next.high || next.valid > UINT32_MAX - out.valid) return false;
    if (!out.valid) out = next;
    else {
        out.low = std::min(out.low, next.low);
        out.high = std::max(out.high, next.high);
        out.valid += next.valid;
    }
    return true;
}

}  // namespace ReportArtifact
