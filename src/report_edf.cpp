#include "report_edf.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "air10_clock.h"

namespace ReportEdf {
namespace {

bool read_at(const Reader &reader, uint64_t offset, uint8_t *out, size_t size) {
    return reader.read && offset <= reader.size && size <= reader.size - offset &&
           reader.read(reader.context, offset, out, size);
}

bool text_field(const uint8_t *bytes, size_t width, char *out, size_t capacity, bool unit = false) {
    while (width && bytes[width - 1] == ' ') width--;
    while (width && *bytes == ' ') { bytes++; width--; }
    if (width >= capacity) return false;
    for (size_t i = 0; i < width; i++)
        if (bytes[i] < 0x20 || bytes[i] == 0x7f || (!unit && bytes[i] > 0x7e)) return false;
    memcpy(out, bytes, width);
    out[width] = 0;
    return true;
}

bool number(const uint8_t *bytes, size_t width, double &out) {
    char text[40];
    if (!text_field(bytes, width, text, sizeof(text)) || !text[0]) return false;
    for (const char *p = text; *p; p++)
        if (!(*p >= '0' && *p <= '9') && *p != '+' && *p != '-' &&
            *p != '.' && *p != 'E' && *p != 'e') return false;
    char *end = nullptr;
    errno = 0;
    const double value = strtod(text, &end);
    if (errno || end == text || *end || !isfinite(value)) return false;
    out = value;
    return true;
}

bool integer(const uint8_t *bytes, size_t width, int64_t min, int64_t max,
             int64_t &out) {
    double value = 0;
    if (!number(bytes, width, value) || value < min || value > max ||
        floor(value) != value) return false;
    out = static_cast<int64_t>(value);
    return true;
}

bool start_time(const uint8_t *date, const uint8_t *time, int64_t &out) {
    if (date[2] != '.' || date[5] != '.' || time[2] != '.' || time[5] != '.')
        return false;
    const uint8_t positions[] = {0, 1, 3, 4, 6, 7};
    for (uint8_t i : positions)
        if (date[i] < '0' || date[i] > '9' || time[i] < '0' || time[i] > '9')
            return false;

    const int year = (date[6] - '0') * 10 + date[7] - '0';
    char dac[9], tic[7];
    snprintf(dac, sizeof(dac), "%.2s%.2s%04d", date, date + 3,
             year >= 85 ? 1900 + year : 2000 + year);
    snprintf(tic, sizeof(tic), "%.2s%.2s%.2s", time, time + 3, time + 6);
    Air10Clock::Calendar calendar;
    if (!Air10Clock::parse_calendar(dac, tic, calendar)) return false;
    out = Air10Clock::civil_seconds(calendar);
    return true;
}

bool same_scale(const Signal &a, const Signal &b) {
    return !strcmp(a.label, b.label) && !strcmp(a.unit, b.unit) &&
           a.digital_min == b.digital_min && a.digital_max == b.digital_max &&
           a.physical_min == b.physical_min && a.physical_max == b.physical_max;
}

const Signal *paired_pressure(const Header &header, const Signal &signal) {
    const int index = !strcmp(signal.label, "Flow.40ms")
        ? find_signal(header, "Press.40ms") : -1;
    return index >= 0 && header.signals[index].samples_per_record == signal.samples_per_record
        ? &header.signals[index] : nullptr;
}

Sample decode_sample(const Signal &signal, const uint8_t *bytes,
                     const Signal *pressure, const uint8_t *paired) {
    // Decode explicitly; no host endian/alignment dependency.
    const uint16_t bits = bytes[0] | (uint16_t(bytes[1]) << 8);
    Sample value;
    value.raw = static_cast<int16_t>(bits < 32768 ? bits : int32_t(bits) - 65536);
    value.valid = signal.valid(value.raw);
    if (pressure) {
        const uint16_t p = paired[0] | (uint16_t(paired[1]) << 8);
        value.valid &= pressure->valid(static_cast<int16_t>(p < 32768 ? p : int32_t(p) - 65536));
    }
    value.value = value.valid ? signal.physical(value.raw) : 0;
    return value;
}

bool record_has_valid(const Reader &reader, const Header &header, size_t signal,
                      uint32_t record, bool &valid) {
    Sample samples[32];
    const uint32_t count = header.signals[signal].samples_per_record;
    valid = false;
    for (uint32_t slot = 0; slot < count;) {
        const size_t take = count - slot > 32 ? 32 : count - slot;
        if (!read_samples(reader, header, signal, uint64_t(record) * count + slot,
                          take, samples)) return false;
        for (size_t i = 0; i < take; i++)
            if (samples[i].valid) { valid = true; return true; }
        slot += take;
    }
    return true;
}

}  // namespace

bool Signal::valid(int32_t raw) const {
    return !crc && !annotation && raw >= digital_min && raw <= digital_max;
}

double Signal::physical(int32_t raw) const {
    return physical_min + (static_cast<int32_t>(raw) - digital_min) *
           (physical_max - physical_min) / (digital_max - digital_min);
}

bool read_layout(const Reader &reader, Layout &out) {
    out = Layout{};
    uint8_t fixed[256];
    if (!read_at(reader, 0, fixed, sizeof(fixed))) return false;

    char version[9], reserved[45];
    int64_t bytes = 0, signals = 0, records = 0;
    if (!text_field(fixed, 8, version, sizeof(version)) || strcmp(version, "0") ||
        !text_field(fixed + 192, 44, reserved, sizeof(reserved)) ||
        (strcmp(reserved, "EDF") && strcmp(reserved, "EDF+D")) ||
        !integer(fixed + 184, 8, 256, 256 + MAX_STR_SIGNALS * 256, bytes) ||
        !integer(fixed + 252, 4, 1, MAX_STR_SIGNALS, signals) ||
        bytes != 256 + signals * 256 || uint64_t(bytes) > reader.size ||
        !integer(fixed + 236, 8, 0, UINT32_MAX, records) ||
        !number(fixed + 244, 8, out.record_seconds) || out.record_seconds < 0 ||
        !start_time(fixed + 168, fixed + 176, out.start_seconds)) return false;

    out.header_bytes = static_cast<uint32_t>(bytes);
    out.signal_count = static_cast<size_t>(signals);
    out.records = static_cast<uint32_t>(records);
    return true;
}

bool read_signals(const Reader &reader, Layout &out, Signal *signals,
                  size_t capacity, bool unsigned_ranges) {
    if (!signals || !out.signal_count || out.signal_count > MAX_STR_SIGNALS ||
        out.signal_count > capacity) return false;
    out.record_bytes = 0;
    for (size_t i = 0; i < out.signal_count; ++i) signals[i] = {};
    // Read whole descriptor columns, not one tiny SD callback per signal.
    uint8_t slab[MAX_SIGNALS * 16];
    const size_t columns[] = {0, 96, 104, 112, 120, 128, 216};
    for (size_t column : columns) {
        const size_t width = column == 0 ? 16 : 8;
        size_t loaded = 0;
        for (size_t i = 0; i < out.signal_count; i++) {
            if (i == loaded) {
                const size_t take = out.signal_count - i < MAX_SIGNALS ? out.signal_count - i : MAX_SIGNALS;
                if (!read_at(reader, 256 + out.signal_count * column + i * width, slab, take * width)) return false;
                loaded += take;
            }
            Signal &signal = signals[i];
            const uint8_t *field = slab + (i % MAX_SIGNALS) * width;
            int64_t value = 0;
            switch (column) {
                case 0:
                    if (!text_field(field, width, signal.label, sizeof(signal.label)) ||
                        !signal.label[0]) return false;
                    break;
                case 96:
                    if (!text_field(field, width, signal.unit, sizeof(signal.unit), true)) return false;
                    break;
                case 104:
                    if (!number(field, width, signal.physical_min)) return false;
                    break;
                case 112:
                    if (!number(field, width, signal.physical_max)) return false;
                    break;
                case 120:
                case 128:
                    if (!integer(field, width, INT16_MIN, unsigned_ranges ? UINT16_MAX : INT16_MAX, value)) return false;
                    if (column == 120) signal.digital_min = static_cast<int32_t>(value);
                    else signal.digital_max = static_cast<int32_t>(value);
                    break;
                case 216:
                    if (!integer(field, width, 1, 32768, value)) return false;
                    signal.samples_per_record = static_cast<uint32_t>(value);
                    break;
            }
        }
    }

    bool numeric = false;
    for (size_t i = 0; i < out.signal_count; i++) {
        Signal &signal = signals[i];
        if (signal.digital_min >= signal.digital_max ||
            (signal.digital_min < 0 && signal.digital_max > INT16_MAX) ||
            signal.physical_min == signal.physical_max ||
            signal.samples_per_record * 2 > 65536 - out.record_bytes) return false;
        for (size_t j = 0; j < i; j++)
            if (!strcmp(signal.label, signals[j].label)) return false;
        signal.record_offset = out.record_bytes;
        signal.crc = !strcmp(signal.label, "Crc16");
        signal.annotation = !strcmp(signal.label, "EDF Annotations");
        numeric |= !signal.crc && !signal.annotation;
        out.record_bytes += signal.samples_per_record * 2;
    }
    return (!numeric || out.record_seconds > 0) &&
           reader.size == out.header_bytes + uint64_t(out.records) * out.record_bytes;
}

bool read_header(const Reader &reader, Header &out) {
    out = Header{};
    return read_layout(reader, out) && read_signals(reader, out, out.signals, MAX_SIGNALS);
}

int find_signal(const Header &header, const char *label) {
    if (!label || header.signal_count > MAX_SIGNALS) return -1;
    for (size_t i = 0; i < header.signal_count; i++)
        if (!header.signals[i].crc && !strcmp(header.signals[i].label, label))
            return static_cast<int>(i);
    return -1;
}

uint64_t sample_count(const Header &header, size_t signal) {
    if (header.signal_count > MAX_SIGNALS || signal >= header.signal_count ||
        header.signals[signal].crc || header.signals[signal].annotation) return 0;
    return uint64_t(header.records) * header.signals[signal].samples_per_record;
}

bool read_samples(const Reader &reader, const Header &header, size_t index,
                  uint64_t first, size_t count, Sample *out) {
    if (header.signal_count > MAX_SIGNALS || index >= header.signal_count ||
        (!out && count)) return false;
    const Signal &signal = header.signals[index];
    const uint64_t total = sample_count(header, index);
    if (signal.crc || signal.annotation || !signal.samples_per_record ||
        first > total || count > total - first) return false;

    const Signal *pressure = paired_pressure(header, signal);

    uint8_t bytes[256], paired[256];
    size_t done = 0;
    while (done < count) {
        const uint64_t sample = first + done;
        const uint32_t slot = sample % signal.samples_per_record;
        size_t take = signal.samples_per_record - slot;
        if (take > count - done) take = count - done;
        if (take > sizeof(bytes) / 2) take = sizeof(bytes) / 2;
        const uint64_t base = header.header_bytes +
            (sample / signal.samples_per_record) * header.record_bytes + slot * 2;
        if (!read_at(reader, base + signal.record_offset, bytes, take * 2) ||
            (pressure && !read_at(reader, base + pressure->record_offset, paired, take * 2)))
            return false;

        for (size_t i = 0; i < take; i++)
            out[done + i] = decode_sample(signal, bytes + i * 2, pressure, paired + i * 2);
        done += take;
    }
    return true;
}

Sample RecordSamples::operator[](size_t slot) const {
    return decode_sample(*signal, data + slot * 2, pressure, paired ? paired + slot * 2 : nullptr);
}

bool record_samples(const Header &header, const uint8_t *record, size_t bytes,
                    size_t index, RecordSamples &out) {
    out = {};
    if (!record || bytes != header.record_bytes || header.signal_count > MAX_SIGNALS ||
        index >= header.signal_count) return false;
    const auto &signal = header.signals[index];
    const Signal *pressure = paired_pressure(header, signal);
    const Signal *specs[] = {&signal, pressure};
    for (const Signal *spec : specs) {
        if (!spec) continue;
        if (spec->crc || spec->annotation || !spec->samples_per_record ||
            spec->record_offset > bytes || spec->samples_per_record > (bytes - spec->record_offset) / 2)
            return false;
    }
    out = {&signal, pressure, record + signal.record_offset,
           pressure ? record + pressure->record_offset : nullptr};
    return true;
}

bool visit_valid_intervals(const Reader &reader, const Header &header,
                           size_t signal, IntervalSink sink, void *context) {
    if (!sink || header.signal_count > MAX_SIGNALS || signal >= header.signal_count ||
        header.signals[signal].crc || header.signals[signal].annotation ||
        !header.signals[signal].samples_per_record || header.record_seconds <= 0)
        return false;
    const uint64_t total = sample_count(header, signal);
    const double step = header.record_seconds / header.signals[signal].samples_per_record;
    Sample samples[32];
    uint64_t begin = 0;
    bool open = false;
    for (uint64_t first = 0; first < total;) {
        const size_t take = total - first > 32 ? 32 : static_cast<size_t>(total - first);
        if (!read_samples(reader, header, signal, first, take, samples)) return false;
        for (size_t i = 0; i < take; i++) {
            if (samples[i].valid && !open) { begin = first + i; open = true; }
            if (!samples[i].valid && open) {
                if (!sink(context, header.start_seconds + begin * step,
                          header.start_seconds + (first + i) * step)) return false;
                open = false;
            }
        }
        first += take;
    }
    return !open || sink(context, header.start_seconds + begin * step,
                         header.start_seconds + total * step);
}

bool recorded_bounds(const Reader &reader, const Header &header,
                     size_t signal, RecordedBounds &out) {
    out = RecordedBounds{};
    if (header.signal_count > MAX_SIGNALS || signal >= header.signal_count ||
        header.signals[signal].crc || header.signals[signal].annotation ||
        !header.signals[signal].samples_per_record || header.record_seconds <= 0)
        return false;
    uint32_t first = 0, end = header.records;
    bool valid = false;
    while (first < end) {
        if (!record_has_valid(reader, header, signal, first, valid)) return false;
        if (valid) break;
        first++;
    }
    if (first == end) return true;
    while (end > first + 1) {
        if (!record_has_valid(reader, header, signal, end - 1, valid)) return false;
        if (valid) break;
        end--;
    }
    out.first_record = first;
    out.record_count = end - first;
    out.begin_seconds = header.start_seconds + first * header.record_seconds;
    out.end_seconds = header.start_seconds + end * header.record_seconds;
    out.present = true;
    return true;
}

EventKind event_kind(const char *text) {
    if (!text) return EventKind::Unknown;
    if (!strcmp(text, "Obstructive Apnea")) return EventKind::OA;
    if (!strcmp(text, "Central Apnea")) return EventKind::CA;
    if (!strcmp(text, "Hypopnea")) return EventKind::H;
    if (!strcmp(text, "Apnea")) return EventKind::UA;
    if (!strcmp(text, "Arousal") || !strcmp(text, "RERA")) return EventKind::RERA;
    if (!strcmp(text, "CSR Start")) return EventKind::CsrStart;
    if (!strcmp(text, "CSR End")) return EventKind::CsrEnd;
    return EventKind::Unknown;
}

bool read_annotation(const Reader &reader, const Header &header,
                     uint32_t record, Annotation &out) {
    out = Annotation{};
    const int index = find_signal(header, "EDF Annotations");
    if (index < 0 || record >= header.records) return false;
    const Signal &signal = header.signals[index];
    const size_t size = signal.samples_per_record * 2;
    uint8_t bytes[256];
    if (!size || size > sizeof(bytes) ||
        !read_at(reader, header.header_bytes + uint64_t(record) * header.record_bytes +
                 signal.record_offset, bytes, size)) return false;

    bool found = false;
    size_t at = 0;
    while (at < size) {
        if (!bytes[at]) { at++; continue; }
        Annotation value;
        size_t begin = at;
        if (bytes[at] != '+' && bytes[at] != '-') return false;
        while (at < size && bytes[at] != 0x14 && bytes[at] != 0x15) at++;
        if (at == size || !number(bytes + begin, at - begin, value.onset_seconds)) return false;
        if (bytes[at] == 0x15) {
            begin = ++at;
            while (at < size && bytes[at] != 0x14) at++;
            if (at == size || !number(bytes + begin, at - begin, value.duration_seconds) ||
                value.duration_seconds < 0) return false;
            value.has_duration = true;
        }
        at++;
        while (at < size && bytes[at]) {
            begin = at;
            while (at < size && bytes[at] && bytes[at] != 0x14) at++;
            if (at == size || bytes[at] != 0x14) return false;
            if (at > begin) {
                if (found || !text_field(bytes + begin, at - begin, value.text, sizeof(value.text)))
                    return false;
                value.kind = event_kind(value.text);
                out = value;
                found = true;
            }
            at++;
        }
        if (at == size) return false;
        at++;
    }
    return found;
}

size_t histogram_bins(const Signal &signal) {
    if (signal.crc || signal.annotation || signal.digital_min < INT16_MIN ||
        signal.digital_max > INT16_MAX || signal.digital_min >= signal.digital_max ||
        !isfinite(signal.physical_min) || !isfinite(signal.physical_max) ||
        signal.physical_min == signal.physical_max) return 0;
    return static_cast<size_t>(signal.digital_max - signal.digital_min + 1);
}

bool init_histogram(const Signal &signal, uint64_t *bins, size_t capacity,
                    Histogram &out) {
    const size_t count = histogram_bins(signal);
    if (!bins || !count || capacity < count) return false;
    out = Histogram{};
    out.signal = signal;
    out.bins = bins;
    out.bin_count = count;
    memset(bins, 0, count * sizeof(*bins));
    return true;
}

bool add_sample(Histogram &histogram, int16_t raw, bool valid) {
    if (!histogram.bins || !histogram.bin_count) return false;
    if (!valid || !histogram.signal.valid(raw)) {
        if (histogram.missing == UINT64_MAX) return false;
        histogram.missing++;
        return true;
    }
    const size_t index = raw - histogram.signal.digital_min;
    if (index >= histogram.bin_count || histogram.count == UINT64_MAX) return false;
    histogram.bins[index]++;
    histogram.count++;
    histogram.sum += histogram.signal.physical(raw);
    return true;
}

bool merge_histogram(Histogram &out, const Histogram &other) {
    if (!out.bins || !other.bins || out.bins == other.bins || !out.bin_count ||
        out.bin_count != other.bin_count || !same_scale(out.signal, other.signal) ||
        other.count > UINT64_MAX - out.count || other.missing > UINT64_MAX - out.missing)
        return false;
    for (size_t i = 0; i < out.bin_count; i++)
        if (other.bins[i] > UINT64_MAX - out.bins[i]) return false;
    for (size_t i = 0; i < out.bin_count; i++) out.bins[i] += other.bins[i];
    out.count += other.count;
    out.missing += other.missing;
    out.sum += other.sum;
    return true;
}

bool percentile(const Histogram &histogram, unsigned percent, double &out) {
    if (!histogram.bins || !histogram.count || !histogram.bin_count ||
        !percent || percent > 100) return false;
    // Split before multiplying so even the uint64_t count limit is safe.
    uint64_t rank = (histogram.count / 100) * percent +
                    ((histogram.count % 100) * percent + 99) / 100;
    const bool descending = histogram.signal.physical_max < histogram.signal.physical_min;
    for (size_t n = 0; n < histogram.bin_count; n++) {
        const size_t i = descending ? histogram.bin_count - 1 - n : n;
        if (rank <= histogram.bins[i]) {
            out = histogram.signal.physical(static_cast<int16_t>(
                histogram.signal.digital_min + static_cast<int32_t>(i)));
            return true;
        }
        rank -= histogram.bins[i];
    }
    return false;
}

bool summarize(const Histogram &histogram, Statistics &out) {
    out = Statistics{};
    if (!histogram.bins || !histogram.bin_count) return false;
    out.count = histogram.count;
    out.missing = histogram.missing;
    out.sum = histogram.sum;
    if (!histogram.count) return true;
    size_t low = 0, high = histogram.bin_count - 1;
    while (low < histogram.bin_count && !histogram.bins[low]) low++;
    while (high && !histogram.bins[high]) high--;
    if (low > high) return false;
    const double a = histogram.signal.physical(static_cast<int16_t>(
        histogram.signal.digital_min + static_cast<int32_t>(low)));
    const double b = histogram.signal.physical(static_cast<int16_t>(
        histogram.signal.digital_min + static_cast<int32_t>(high)));
    out.min = a < b ? a : b;
    out.max = a > b ? a : b;
    out.mean = histogram.sum / histogram.count;
    out.present = percentile(histogram, 50, out.median) && percentile(histogram, 95, out.p95);
    return out.present;
}

}  // namespace ReportEdf
