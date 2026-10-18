#pragma once

#include <stddef.h>
#include <stdint.h>

namespace ReportEdf {

constexpr size_t MAX_SIGNALS = 32;
constexpr size_t MAX_STR_SIGNALS = 128;

struct Reader {
    // Exact offset read; the caller owns the immutable file and its I/O lease.
    bool (*read)(void *, uint64_t, uint8_t *, size_t) = nullptr;
    void *context = nullptr;
    uint64_t size = 0;
};

struct Signal {
    char label[17] = {};
    char unit[9] = {};
    double physical_min = 0, physical_max = 0;
    int32_t digital_min = 0, digital_max = 0;
    uint32_t samples_per_record = 0, record_offset = 0;
    bool annotation = false, crc = false;

    bool valid(int32_t raw) const;
    double physical(int32_t raw) const;
};

struct Layout {
    uint32_t header_bytes = 0, record_bytes = 0, records = 0;
    double record_seconds = 0;
    // Native EDF wall-time in epoch-like seconds, NOT UTC or host-local time.
    int64_t start_seconds = 0;
    size_t signal_count = 0;
};

struct Header : Layout {
    Signal signals[MAX_SIGNALS];
};

struct Sample {
    int16_t raw = 0;
    double value = 0;
    bool valid = false;
};

// Borrowed view into one complete raw record. No I/O or decoded sample array;
// valid until the caller reuses the record buffer. Uses read_samples' validity.
struct RecordSamples {
    const Signal *signal = nullptr, *pressure = nullptr;
    const uint8_t *data = nullptr, *paired = nullptr;
    Sample operator[](size_t slot) const;  // slot < signal->samples_per_record
};

enum class EventKind : uint8_t {
    Unknown, OA, CA, H, UA, RERA, CsrStart, CsrEnd,
};

struct Annotation {
    double onset_seconds = 0, duration_seconds = 0;
    bool has_duration = false;
    EventKind kind = EventKind::Unknown;
    char text[64] = {};
};

struct Histogram {
    Signal signal;
    uint64_t *bins = nullptr;
    size_t bin_count = 0;
    uint64_t count = 0, missing = 0;
    double sum = 0;
};

struct Statistics {
    uint64_t count = 0, missing = 0;
    double sum = 0, min = 0, max = 0, mean = 0, median = 0, p95 = 0;
    bool present = false;
};

struct RecordedBounds {
    uint32_t first_record = 0, record_count = 0;
    double begin_seconds = 0, end_seconds = 0;
    bool present = false;
};

struct RecordScanPosition {
    enum class Phase : uint8_t { Leading, Trailing, Done, Empty };
    uint32_t ordinal = 0, record = 0;
    uint32_t first = 0, slot = 0, valid_mask = 0;
    Phase phase = Phase::Leading;
    bool paired_pending = false;
};

// Our finalized BRP/PLD/SAD/EVE/CSL only: no CRC/SHA checks or recovery.
// Reject incomplete/mismatched file sizes and unfinished record counts (-1).
// Output is usable only on success; zero-record headers are valid.
bool read_header(const Reader &reader, Header &out);
// Two-phase header read for STR: allocate the actual signal count separately,
// without enlarging every detail Header. Only read_signals validates file size.
bool read_layout(const Reader &reader, Layout &out);
bool read_signals(const Reader &reader, Layout &layout, Signal *signals,
                  size_t capacity, bool unsigned_ranges = false);
int find_signal(const Header &header, const char *label);
uint64_t sample_count(const Header &header, size_t signal);

// No sample shifting. Outside digital bounds is missing. Flow raw -1 remains
// valid unless paired Press.40ms on the same sample axis is invalid. Without
// that channel, old recorder flow gaps cannot be distinguished from valid -1.
// Failure may leave a partial output; callers must discard that range.
bool read_samples(const Reader &reader, const Header &header, size_t signal,
                  uint64_t first, size_t count, Sample *out);

bool record_samples(const Header &header, const uint8_t *record, size_t bytes,
                    size_t signal, RecordSamples &out);

// Contiguous valid sample slots on the unshifted file axis. Zero is valid if
// inside digital bounds. Sink receives native absolute [begin, end) seconds.
using IntervalSink = bool (*)(void *, double, double);
bool visit_valid_intervals(const Reader &reader, const Header &header,
                           size_t signal, IntervalSink sink, void *context);
// Selected numeric channel only: trim wholly invalid leading/trailing records,
// retain any edge record with a valid sample (including zero), and do not trim
// interior gaps. This is distinct from per-sample validity coverage above.
// Optional position resumes after failed reads, including within a record.
// ordinal counts completed records in traversal order, including the tail.
// Reset position to {} when the source, header or selected signal changes.
// A completed position returns the same bounds without reading again.
bool recorded_bounds(const Reader &reader, const Header &header,
                     size_t signal, RecordedBounds &out, RecordScanPosition *position = nullptr);

// Skip timekeeping TALs, retain the one event-bearing TAL in our records.
// EVE onset is publication/end: display its band at end-duration..end without
// changing these raw fields. CSL start/end remain independent annotations.
bool read_annotation(const Reader &reader, const Header &header,
                     uint32_t record, Annotation &out);
EventKind event_kind(const char *text);

// One exact bin per digital value. Caller supplies/owns the bins; no allocation.
size_t histogram_bins(const Signal &signal);
bool init_histogram(const Signal &signal, uint64_t *bins, size_t capacity,
                    Histogram &out);
bool add_sample(Histogram &histogram, int16_t raw, bool valid = true);
bool merge_histogram(Histogram &out, const Histogram &other);
// Nearest rank: ceil(count * percent / 100), percent in 1..100; no interpolation.
bool percentile(const Histogram &histogram, unsigned percent, double &out);
// Empty initialized histograms succeed with present=false, never invented zeros.
bool summarize(const Histogram &histogram, Statistics &out);

}  // namespace ReportEdf
