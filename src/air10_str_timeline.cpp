#include "air10_str_timeline.h"

#include <string.h>

namespace Air10StrTimeline {

bool begin(uint16_t header_start_day, uint32_t record_count, Scan &scan) {
    scan = {};
    if (header_start_day == UINT16_MAX || record_count > RECORD_LIMIT ||
        (record_count &&
         static_cast<uint32_t>(header_start_day) + record_count - 1 >=
             UINT16_MAX)) {
        return false;
    }
    scan.header_start_day = header_start_day;
    scan.record_count = record_count;
    scan.continuous = true;
    return true;
}

bool record_day(const Scan &scan, uint32_t record_index,
                uint16_t date_sample, uint16_t &day) {
    if (record_index >= scan.record_count) return false;
    const uint32_t expected =
        static_cast<uint32_t>(scan.header_start_day) + record_index;
    if (expected >= UINT16_MAX) return false;
    day = date_sample == UINT16_MAX ? static_cast<uint16_t>(expected)
                                    : date_sample;
    return day != UINT16_MAX;
}

bool scan_record(Scan &scan, uint32_t record_index, uint16_t date_sample) {
    if (record_index != scan.scanned_records) return false;
    uint16_t day = 0;
    if (!record_day(scan, record_index, date_sample, day)) return false;

    if (scan.minimum_record_day == UINT16_MAX ||
        day < scan.minimum_record_day) {
        scan.minimum_record_day = day;
    }
    if (day > scan.maximum_record_day) scan.maximum_record_day = day;

    const uint16_t expected = static_cast<uint16_t>(
        static_cast<uint32_t>(scan.header_start_day) + record_index);
    if (date_sample == UINT16_MAX || day != expected)
        scan.continuous = false;
    scan.scanned_records++;
    return true;
}

bool make_plan(const Scan &scan, uint16_t incoming_day, Plan &plan) {
    plan = {};
    if (incoming_day == UINT16_MAX ||
        scan.scanned_records != scan.record_count) {
        return false;
    }

    uint32_t start_day = incoming_day;
    uint32_t end_day = incoming_day;
    if (scan.record_count) {
        const uint32_t expected_end =
            static_cast<uint32_t>(scan.header_start_day) +
            scan.record_count - 1;
        start_day = scan.header_start_day;
        end_day = expected_end;
        if (scan.minimum_record_day < start_day)
            start_day = scan.minimum_record_day;
        if (scan.maximum_record_day > end_day)
            end_day = scan.maximum_record_day;
        if (incoming_day < start_day) start_day = incoming_day;
        if (incoming_day > end_day) end_day = incoming_day;
    }
    if (end_day >= UINT16_MAX || end_day < start_day) return false;

    const uint32_t full_count = end_day - start_day + 1;
    if (full_count > RECORD_LIMIT) return false;
    if (incoming_day < start_day) return false;

    plan.start_day = static_cast<uint16_t>(start_day);
    plan.end_day = static_cast<uint16_t>(end_day);
    plan.record_count = end_day - start_day + 1;
    plan.incoming_index = incoming_day - start_day;
    return plan.record_count > 0 && plan.record_count <= RECORD_LIMIT;
}

bool place_record(const Plan &plan, Buffer &buffer, uint16_t day,
                  const uint8_t *record, size_t record_size,
                  BuildStats &stats) {
    if (!buffer.records || !buffer.present || !record || !record_size ||
        buffer.capacity < plan.record_count) {
        return false;
    }
    if (day < plan.start_day || day > plan.end_day) {
        stats.discarded_records++;
        return true;
    }

    const uint32_t index = day - plan.start_day;
    uint8_t *destination = buffer.records + index * record_size;
    if (buffer.present[index]) {
        stats.replaced_records++;
    } else {
        buffer.present[index] = 1;
        stats.placed_records++;
    }
    memcpy(destination, record, record_size);
    return true;
}

bool fill_missing(const Plan &plan, Buffer &buffer, size_t record_size,
                  FillerRenderer renderer, BuildStats &stats) {
    if (!buffer.records || !buffer.present || !record_size || !renderer ||
        buffer.capacity < plan.record_count) {
        return false;
    }
    for (uint32_t i = 0; i < plan.record_count; i++) {
        if (buffer.present[i]) continue;
        uint8_t *record = buffer.records + i * record_size;
        if (!renderer(static_cast<uint16_t>(plan.start_day + i), record,
                      record_size)) {
            return false;
        }
        buffer.present[i] = 1;
        stats.filler_records++;
    }
    return true;
}

}  // namespace Air10StrTimeline
