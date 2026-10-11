#pragma once

#include <stddef.h>
#include <stdint.h>

// Portable, immutable numeric publications. No filesystem, task or allocation
// ownership here; EdfReport owns their production and publication.
namespace ReportArtifact {

constexpr uint16_t VERSION = 1;
constexpr size_t TILE_SAMPLES = 512;
constexpr size_t RAW_TILE_BYTES = TILE_SAMPLES * 2 + TILE_SAMPLES / 8;
constexpr size_t REDUCED_TILE_BYTES = TILE_SAMPLES * 8;
constexpr size_t MAX_TRACKS = 16;
constexpr size_t MAX_LEVELS = 12;
constexpr size_t HEADER_BYTES = 64;
constexpr size_t TRACK_BYTES = 96;
constexpr size_t LEVEL_BYTES = 16;
constexpr size_t MAX_METADATA_BYTES = HEADER_BYTES + MAX_TRACKS *
    (TRACK_BYTES + MAX_LEVELS * LEVEL_BYTES);

struct Level {
    uint32_t factor = 0, count = 0, offset = 0, bytes = 0;
};

struct Track {
    uint16_t id = 0, level_count = 0;
    uint32_t count = 0;
    int64_t start_ms = 0;
    // Rational sample interval: record_us / samples_per_record. Do not round
    // the phase or force independent files onto a shared viewport grid.
    uint64_t record_us = 0;
    uint32_t samples_per_record = 0;
    int32_t digital_min = 0, digital_max = 0;
    double physical_min = 0, physical_max = 0;
    char unit[16] = {}, label[24] = {};
    Level levels[MAX_LEVELS];
};

struct Metadata {
    uint16_t track_count = 0;
    uint32_t header_bytes = 0, bytes = 0;
    uint8_t source[32] = {};
    Track tracks[MAX_TRACKS];
};

struct Cell {
    int16_t low = 0, high = 0;
    uint32_t valid = 0;
};

// Canonical track-major layout. Every level contains fixed-size tiles; unused
// positions in the last tile are zero, not additional samples or missing time.
bool layout(Metadata &metadata);
bool encode(const Metadata &metadata, uint8_t *out, size_t capacity);
bool decode(const uint8_t *data, size_t length, uint64_t file_bytes, Metadata &out);
uint32_t tile_count(uint32_t positions);
uint32_t expected(const Track &track, const Level &level, uint32_t position);

void put_raw(uint8_t *tile, size_t position, int16_t raw, bool valid);
Cell raw_cell(const uint8_t *tile, size_t position);
void put_cell(uint8_t *tile, size_t position, const Cell &cell);
Cell get_cell(const uint8_t *tile, size_t position);
bool merge(Cell &out, const Cell &next);

}  // namespace ReportArtifact
