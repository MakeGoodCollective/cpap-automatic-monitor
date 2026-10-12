#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Air10StrTimeline {

// Local history outlives EEPROM retention; never trim the only remaining copy.
constexpr uint32_t RECORD_LIMIT = UINT16_MAX;

struct Scan {
    uint16_t header_start_day = 0;
    uint16_t minimum_record_day = UINT16_MAX;
    uint16_t maximum_record_day = 0;
    uint32_t record_count = 0;
    uint32_t scanned_records = 0;
    bool continuous = false;
};

struct Plan {
    uint16_t start_day = 0;
    uint16_t end_day = 0;
    uint32_t record_count = 0;
    uint32_t incoming_index = 0;
};

struct Buffer {
    uint8_t *records = nullptr;
    uint8_t *present = nullptr;
    uint32_t capacity = 0;
};

struct BuildStats {
    uint32_t placed_records = 0;
    uint32_t replaced_records = 0;
    uint32_t discarded_records = 0;
    uint32_t filler_records = 0;
};

using FillerRenderer = bool (*)(uint16_t day, uint8_t *record,
                                size_t record_size);

bool begin(uint16_t header_start_day, uint32_t record_count, Scan &scan);
bool scan_record(Scan &scan, uint32_t record_index, uint16_t date_sample);
bool record_day(const Scan &scan, uint32_t record_index,
                uint16_t date_sample, uint16_t &day);
bool make_plan(const Scan &scan, uint16_t incoming_day, Plan &plan);
bool place_record(const Plan &plan, Buffer &buffer, uint16_t day,
                  const uint8_t *record, size_t record_size,
                  BuildStats &stats);
bool fill_missing(const Plan &plan, Buffer &buffer, size_t record_size,
                  FillerRenderer renderer, BuildStats &stats);

}  // namespace Air10StrTimeline
