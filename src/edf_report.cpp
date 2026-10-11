#include "edf_report.h"
#include "board.h"
#include "sd_storage.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <utility>

namespace EdfReport {

bool Request::operator==(const Request &other) const {
    return action == other.action && view == other.view && day == other.day &&
        period == other.period && from_ms == other.from_ms && to_ms == other.to_ms &&
        px == other.px && revision == other.revision &&
        !memcmp(excluded, other.excluded, sizeof(excluded));
}

bool valid(const Request &request) {
    if (request.action > Action::History || request.view > View::Period ||
        request.day == UINT16_MAX) return false;
    if (request.action == Action::History && request.view != View::Day) return false;
    if (request.view == View::Period && request.period != 1 && request.period != 7 &&
        request.period != 30 && request.period != 90 && request.period != 180 &&
        request.period != 365) return false;
    for (uint8_t excluded : request.excluded)
        if (excluded && (request.action != Action::Series || !request.revision)) return false;
    return request.action != Action::Series ||
        (request.from_ms > 0 && request.to_ms > request.from_ms &&
         request.to_ms - request.from_ms <= 366ll * 86400000 &&
         request.px && request.px <= 1600);
}

const char *state_name(State state) {
    switch (state) {
        case State::Ready: return "ready";
        case State::WaitingStorage: return "waiting_storage";
        case State::Building: return "building";
        case State::Blocked: return "blocked";
        default: return "error";
    }
}

size_t Result::read(size_t offset, char *out, size_t capacity) const {
    if (!data_ || offset >= length_ || !out) return 0;
    const size_t count = std::min(capacity, length_ - offset);
    memcpy(out, data_ + offset, count);
    return count;
}

Result::~Result() { reset(); }
Result::Result(Result &&other) noexcept
    : id_(std::exchange(other.id_, 0)), data_(std::exchange(other.data_, nullptr)),
      length_(std::exchange(other.length_, 0)) {}

}  // namespace EdfReport

#if AB_STORAGE_HAS_SDCARD
#include "air10_clock.h"
#include "air10_edf.h"
#include "air10_str_timeline.h"
#include "airsense_state.h"
#include "edf_catalog.h"
#include "debug_log.h"
#include "memory_manager.h"
#include "large_text_buffer.h"
#include "hex_util.h"
#include "report_edf.h"
#include "settings_defs.h"
#include <Arduino.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>
#include <cstdarg>
#include <cstdio>
#include <new>

namespace EdfReport {
namespace {
constexpr uint32_t STATS_CACHE_VERSION = 7;
constexpr uint32_t SERIES_CACHE_VERSION = 7;
constexpr size_t MAX_BINS = 16384;
constexpr size_t MAX_EVENTS = 4096;
constexpr size_t MAX_DISK_EVENTS = 65536;
constexpr size_t MAX_BODY = 512 * 1024;
constexpr size_t INTERNAL_BODY = 16 * 1024;
constexpr size_t INTERNAL_WORK = 8 * 1024;
constexpr uint64_t MAX_SD_CACHE = 128ull * 1024 * 1024;
constexpr uint32_t RETAIN_MS = 6 * 60 * 60 * 1000;
constexpr uint32_t DELIVERY_MS = 30000;
constexpr size_t SLOT_COUNT = 2;
constexpr size_t RETAINED_SLOTS = 32;
constexpr size_t CHART_CHANNEL_COUNT = 11;
constexpr double LEAK_THRESHOLD = 24;
constexpr int64_t LEVEL_MS[] = {2000, 10000, 60000, 600000, 3600000};
constexpr size_t LEVEL_COUNT = sizeof(LEVEL_MS) / sizeof(LEVEL_MS[0]);
constexpr int64_t FINE_BLOCK_MS = 60000;
constexpr size_t FINE_BLOCK_CELLS = FINE_BLOCK_MS / LEVEL_MS[0];
constexpr const char *CACHE_DIR = "/airbridge/report";
constexpr const char *SUFFIXES[] = {"BRP", "PLD", "SAD", "EVE", "CSL"};

struct ChannelSpec {
    const char *id, *label, *unit, *signal;
    uint8_t file;
};
constexpr ChannelSpec CHANNELS[] = {
    {"Press", "Pressure", "cmH2O", "Press.2s", 1},
    {"EprPress", "Expiratory pressure", "cmH2O", "EprPress.2s", 1},
    {"MaskPress", "Mask pressure", "cmH2O", "MaskPress.2s", 1},
    {"Leak", "Leak", "L/min", "Leak.2s", 1},
    {"MinVent", "Minute ventilation", "L/min", "MinVent.2s", 1},
    {"RespRate", "Respiratory rate", "bpm", "RespRate.2s", 1},
    {"TidVol", "Tidal volume", "mL", "TidVol.2s", 1},
    {"SpO2", "SpO2", "%", "SpO2.1s", 2},
    {"Pulse", "Pulse", "bpm", "Pulse.1s", 2},
    {"FlowLim", "Flow limitation", "", "FlowLim.2s", 1},
    {"Snore", "Snore", "", "Snore.2s", 1},
    {"TgtVent", "Target ventilation", "L/min", "TgtVent.2s", 1},
    {"Ti", "Inspiratory time", "s", "Ti.2s", 1},
    {"IERatio", "I:E", "%", "IERatio.2s", 1},
    {"AlvMinVent", "Alveolar ventilation", "L/min", "AlvMinVent.2s", 1},
};
constexpr size_t CHANNEL_COUNT = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

// Sidecars are identity inputs, not recomputed source-data checksums.
struct SourceKey {
    uint64_t sizes[5] = {};
    uint8_t sidecars[5][8] = {};
    uint8_t present = 0, checksums = 0;

    bool operator==(const SourceKey &other) const {
        return present == other.present && checksums == other.checksums &&
            !memcmp(sizes, other.sizes, sizeof(sizes)) &&
            !memcmp(sidecars, other.sidecars, sizeof(sidecars));
    }
};
struct IndexedSession {
    EdfCatalog::Entry entry = {};
    SourceKey key;
    uint16_t day = 0;
    int64_t start_ms = 0, end_ms = 0;
    int64_t event_from_ms = 0, event_to_ms = 0;
    bool bounds_known = false;
};
struct Event {
    int64_t time_ms = 0, duration_ms = 0;
    ReportEdf::EventKind kind = ReportEdf::EventKind::Unknown;
};
struct CachedChannel {
    ReportEdf::Signal signal;
    uint64_t count = 0, missing = 0;
    double sum = 0, coverage_seconds = 0;
    uint32_t bins = 0;
};
struct CacheHeader {
    uint32_t version = STATS_CACHE_VERSION, header_bytes = sizeof(CacheHeader), bytes = 0;
    char magic[8] = {'A','B','R','P','T','0','1',0};
    SourceKey key;
    int64_t start_ms = 0, end_ms = 0;
    int64_t chart_start_ms = 0, chart_end_ms = 0;
    int64_t event_from_ms = 0, event_to_ms = 0;
    double used_seconds = 0;
    uint64_t counts[5] = {};
    CachedChannel channels[CHANNEL_COUNT];
    uint32_t events = 0;
    uint16_t day = 0;
    bool recorded = false, partial = false, events_truncated = false, events_known = false;
    bool usage_known = false;
    double csr_seconds = 0, leak_high_seconds = 0;
    bool csr_known = false;
};
struct SessionData {
    CacheHeader header;
    uint64_t *bins = nullptr;
    Event *events = nullptr;
    ~SessionData() {
        aircannect::Memory::free(bins);
        aircannect::Memory::free(events);
    }
};

struct DailyValues {
    uint64_t count[4] = {}, events = 0;
    double sum[4] = {}, seconds = 0;
    bool present = false, ready = true, usage = true, event_count = true;

    double value(size_t metric) const {
        if (!present || !ready) return NAN;
        if (!metric) return usage && seconds > 0 ? seconds : NAN;
        if (metric == 1) return usage && event_count && seconds > 0 ? events * 3600.0 / seconds : NAN;
        return count[metric - 2] ? sum[metric - 2] / count[metric - 2] : NAN;
    }
};
struct CachedDay {
    DailyValues values;
    uint32_t updated = 0;
    uint16_t day = 0;
};
// Two annual selections; lazy growth, no per-day SD files or internal bulk fallback.
constexpr size_t MAX_CACHED_DAYS = 2 * 366;
CachedDay *daily_cache = nullptr;
size_t daily_capacity = 0;
uint32_t daily_update = 0;

void invalidate_day(uint16_t day = 0) {
    if (!day) {
        aircannect::Memory::free(daily_cache);
        daily_cache = nullptr;
        daily_capacity = 0;
        return;
    }
    for (size_t i = 0; i < daily_capacity; ++i)
        if (daily_cache[i].day == day) daily_cache[i].day = 0;
}
bool cached_day(uint16_t day, DailyValues &out, bool build) {
    if (!day) return false;
    for (size_t i = 0; i < daily_capacity; ++i) {
        auto &entry = daily_cache[i];
        if (entry.day != day || (build && !entry.values.ready)) continue;
        out = entry.values;
        entry.updated = ++daily_update;
        return true;
    }
    return false;
}
void remember_day(uint16_t day, const DailyValues &values) {
    if (!day) return;
    size_t slot = daily_capacity;
    for (size_t i = 0; i < daily_capacity; ++i) {
        if (daily_cache[i].day == day) { slot = i; break; }
        if (!daily_cache[i].day && slot == daily_capacity) slot = i;
    }
    if (slot == daily_capacity && daily_capacity < MAX_CACHED_DAYS) {
        const size_t capacity = std::min(MAX_CACHED_DAYS, std::max(size_t(16), daily_capacity * 2));
        auto *grown = static_cast<CachedDay *>(aircannect::Memory::realloc_large(
            daily_cache, capacity * sizeof(CachedDay), false));
        if (grown) {
            daily_cache = grown;
            for (size_t i = daily_capacity; i < capacity; ++i) new (&daily_cache[i]) CachedDay;
            daily_capacity = capacity;
        }
    }
    if (!daily_capacity) return; // Cache allocation failure does not fail a report.
    if (slot == daily_capacity) {
        slot = 0;
        for (size_t i = 1; i < daily_capacity; ++i)
            if (int32_t(daily_cache[i].updated - daily_cache[slot].updated) < 0) slot = i;
    }
    daily_cache[slot] = {values, ++daily_update, day};
}

struct Payload : aircannect::LargeTextBuffer {
    Payload() : LargeTextBuffer(MAX_BODY, INTERNAL_BODY) {}
    bool format_error = false;
    bool good() const { return !format_error && !overflowed(); }
    bool print(const char *format, ...) {
        char text[256];
        va_list args;
        va_start(args, format);
        const int count = vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        if (count < 0 || size_t(count) >= sizeof(text)) { format_error = true; return false; }
        return append(text, size_t(count));
    }
};

enum class JobState : uint8_t { Free, Queued, Running, Complete };
struct Job {
    Request request;
    JobState state = JobState::Free;
    uint32_t id = 0, completed_ms = 0, data_revision = 0, cache_generation = 0;
    uint16_t readers = 0;
    uint16_t consumers = 1, deliveries = 0;
    bool delivered = false, cancelled = false, shared = false;
    int code = 0;
    const char *error = nullptr;
    mutable const char *issue = nullptr;
    mutable char issue_path[96] = {};
    Payload body;
};

bool note_issue(const Job &job, const char *reason, const char *path = "") {
    if (!job.issue) {
        job.issue = reason;
        snprintf(job.issue_path, sizeof(job.issue_path), "%s", path);
    }
    return false;
}

void log_result(const Job &job) {
    const bool expected = job.code == 409 || job.code == 410;
    const char *reason = expected ? job.error : job.issue ? job.issue : job.error;
    Log::logf(CAT_REPORT, expected ? LOG_DEBUG : job.code >= 500 ? LOG_ERROR :
        job.issue ? LOG_WARN : LOG_DEBUG,
        "EDF job=%lu %s day=%u code=%d %s path=%s\n",
        (unsigned long)job.id, job.request.action == Action::Days ? "days" :
        job.request.action == Action::Summary ? "summary" : job.request.action == Action::History ? "history" : "series",
        unsigned(job.request.day), job.code, reason ? reason : "complete",
        expected || !job.issue_path[0] ? "-" : job.issue_path);
}

struct JobTiming {
    bool enabled;
    uint32_t id, started_us;
    Action action;
    uint32_t index_us = 0, numeric_us = 0, events_us = 0, encode_us = 0;

    explicit JobTiming(const Job *job)
        : enabled(job && Log::get_cat_level(CAT_REPORT) >= LOG_DEBUG),
          id(job ? job->id : 0), started_us(stamp()),
          action(job ? job->request.action : Action::Days) {}

    uint32_t stamp() const { return enabled ? micros() : 0; }

    void log(int code, size_t bytes) const {
        if (!enabled) return;
        Log::logf(CAT_REPORT, LOG_DEBUG,
            "report job=%lu action=%s code=%d bytes=%u ms total=%lu "
            "idx=%lu num=%lu evt=%lu enc=%lu\n",
            static_cast<unsigned long>(id), action == Action::Days ? "days" :
                action == Action::Summary ? "summary" : action == Action::History ? "history" : "series", code,
            static_cast<unsigned>(bytes), static_cast<unsigned long>((stamp() - started_us) / 1000),
            static_cast<unsigned long>(index_us / 1000), static_cast<unsigned long>(numeric_us / 1000),
            static_cast<unsigned long>(events_us / 1000), static_cast<unsigned long>(encode_us / 1000));
    }
};

struct JobPool {
    Job fallback[SLOT_COUNT];
    Job *data = fallback;
    size_t count = SLOT_COUNT;
    Job *begin() { return data; }
    Job *end() { return data + count; }
} jobs;
SemaphoreHandle_t mutex = nullptr, wake = nullptr;
TaskHandle_t worker_task = nullptr;
portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
Status status;
uint32_t next_id = 1, data_revision = 1;
IndexedSession *index = nullptr;
size_t index_count = 0;
uint32_t catalog_generation = 0, file_revision = 0;
uint32_t invalidation_generation = 0, indexed_invalidation = 0;
bool index_ready = false;
bool cache_usage_known = false;
uint64_t cache_bytes = 0;
uint32_t cache_generation = 0;
enum class IndexFailure : uint8_t { Error, WaitingCatalog, Preempted };
IndexFailure index_failure = IndexFailure::Error;
const char *index_error = nullptr;
size_t index_error_entry = SIZE_MAX;
bool prune_cache(const SdStorage::Session &session, uint64_t needed = 0,
                 const char *keep = nullptr);
void invalidate_str();

bool retire_cache(const SdStorage::Session &session, const IndexedSession &entry) {
    bool removed = false;
    const bool success = session.run([&](fs::FS &fs) {
        for (const char *suffix : {"stats", "series", "stats.part", "series.part"}) {
            char path[96];
            snprintf(path, sizeof(path), "%s/%s/%s.%s", CACHE_DIR,
                     entry.entry.therapy_day, entry.entry.file_prefix, suffix);
            if (fs.exists(path)) {
                if (!fs.remove(path)) return false;
                removed = true;
            }
        }
        return true;
    });
    if (removed) cache_usage_known = false;
    return success;
}

void publish(State state, const char *error = "") {
    portENTER_CRITICAL(&status_mux);
    if (status.state != state || strcmp(status.error, error)) {
        status.state = state;
        snprintf(status.error, sizeof(status.error), "%s", error);
        ++status.revision;
    }
    portEXIT_CRITICAL(&status_mux);
}

bool reusable(const Job &job) {
    return job.state == JobState::Free || (job.state == JobState::Complete &&
        !job.readers && (job.cancelled || uint32_t(millis() - job.completed_ms) >=
            (job.code == 200 && !job.issue ? RETAIN_MS : DELIVERY_MS)));
}
void reset_job(Job &job) {
    job.~Job();
    new (&job) Job;
}

// Caller holds mutex. Undelivered results get the existing delivery grace;
// active HTTP leases are never candidates, even under memory pressure.
bool evictable(const Job &job) {
    return job.state == JobState::Complete && !job.readers &&
        (job.deliveries >= job.consumers || uint32_t(millis() - job.completed_ms) >= DELIVERY_MS);
}
bool reclaim_result() {
    // Called by Memory in any allocating task, including error serialization
    // with this mutex already held. Do not wait or touch SD here.
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
    Job *oldest = nullptr;
    for (auto &job : jobs) {
        if (evictable(job) && job.body.capacity() &&
            (!oldest || int32_t(job.completed_ms - oldest->completed_ms) < 0)) oldest = &job;
    }
    if (oldest) reset_job(*oldest);
    xSemaphoreGive(mutex);
    return oldest != nullptr;
}
bool aborted(const Job *job, const SdStorage::Session &session) {
    return (job && __atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) ||
        !AirSenseState::local_background_allowed() || !session.valid();
}

void source_path(const IndexedSession &entry, uint8_t file, const char *ext,
                 char (&out)[96]) {
    snprintf(out, sizeof(out), "/DATALOG/%s/%s_%s.%s", entry.entry.therapy_day,
             entry.entry.file_prefix, SUFFIXES[file], ext);
}
void cache_path(const IndexedSession &entry, const char *ext, char (&out)[96]) {
    snprintf(out, sizeof(out), "%s/%s/%s.%s", CACHE_DIR,
             entry.entry.therapy_day, entry.entry.file_prefix, ext);
}

bool catalog_day(const char *text, uint16_t &out) {
    if (strlen(text) != 8) return false;
    int parts[3] = {};
    for (size_t i = 0; i < 8; ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
        const size_t part = i < 4 ? 0 : i < 6 ? 1 : 2;
        parts[part] = parts[part] * 10 + text[i] - '0';
    }
    char dac[9];
    snprintf(dac, sizeof(dac), "%02d%02d%04d", parts[2], parts[1], parts[0]);
    Air10Clock::Calendar calendar;
    if (!Air10Clock::parse_calendar(dac, "120000", calendar)) return false;
    const int32_t day = Air10Clock::civil_epoch_day(parts[0], parts[1], parts[2]);
    if (day <= 0 || day >= UINT16_MAX) return false;
    out = static_cast<uint16_t>(day);
    return true;
}

bool fingerprint(const SdStorage::Session &session, IndexedSession &entry) {
    entry.key = {};
    return session.run([&](fs::FS &fs) {
        for (uint8_t i = 0; i < 5; ++i) {
            char path[96];
            source_path(entry, i, "edf", path);
            fs::File file = fs.open(path, FILE_READ);
            if (file && !file.isDirectory()) {
                entry.key.sizes[i] = file.size();
                entry.key.present |= 1u << i;
            }
            file.close();
            source_path(entry, i, "crc", path);
            file = fs.open(path, FILE_READ);
            if (file && file.size() == 8 &&
                file.read(entry.key.sidecars[i], 8) == 8) entry.key.checksums |= 1u << i;
            file.close();
        }
        return true;
    });
}

bool refresh_index(const SdStorage::Session &session) {
    index_failure = IndexFailure::Error;
    index_error = "catalog_status";
    index_error_entry = SIZE_MAX;
    EdfCatalog::Changes changes;
    bool incremental = false;
    if (!session.run([&](fs::FS &) {
        incremental = EdfCatalog::changes_since(catalog_generation, changes);
        return changes.catalog.ready;
    })) {
        if (!session.valid() || !AirSenseState::local_background_allowed()) index_failure = IndexFailure::Preempted;
        else if (!changes.catalog.ready) index_failure = IndexFailure::WaitingCatalog;
        return false;
    }
    const uint32_t files = SdStorage::files_revision();
    const uint32_t invalidation = __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE);
    const bool invalidated = invalidation != indexed_invalidation;
    if (index_ready && !invalidated && changes.catalog.generation == catalog_generation && files == file_revision)
        return true;
    const bool full = !index_ready || invalidated || !incremental || files != file_revision ||
        changes.catalog.entries < index_count;
    if (full) { invalidate_day(); invalidate_str(); }
    const size_t old_count = index_count;
    index_error = "session_limit";
    if (changes.catalog.entries > MAX_SESSION_COUNT) return false;
    if (changes.catalog.entries != index_count) {
        const size_t bytes = changes.catalog.entries * sizeof(IndexedSession);
        if (!bytes) {
            aircannect::Memory::free(index);
            index = nullptr;
        } else {
            void *grown = aircannect::Memory::realloc_large(index, bytes, bytes <= INTERNAL_WORK);
            if (!grown) { index_error = "index_alloc"; return false; }
            index = static_cast<IndexedSession *>(grown);
        }
        if (changes.catalog.entries > index_count)
            for (size_t i = index_count; i < changes.catalog.entries; ++i)
                new (&index[i]) IndexedSession;
        index_count = changes.catalog.entries;
    }
    bool detail_changed = !index_ready || changes.catalog.entries < old_count;
    bool summary_changed = false;
    const size_t count = full ? index_count : changes.count;
    for (size_t n = 0; n < count; ++n) {
        if (aborted(nullptr, session)) {
            index_failure = IndexFailure::Preempted;
            index_ready = false;
            return false;
        }
        const size_t i = full ? n : changes.indexes[n];
        index_error_entry = i;
        index_error = "catalog_index";
        if (i >= index_count) return false;
        IndexedSession next;
        index_error = "catalog_read";
        if (!session.run([&](fs::FS &) { return EdfCatalog::read(i, next.entry); })) return false;
        index_error = "catalog_day";
        if (!catalog_day(next.entry.therapy_day, next.day)) return false;
        // STR/Identification readiness is not a detail-data revision.
        summary_changed |= index[i].entry.str_revision != next.entry.str_revision ||
            ((index[i].entry.flags ^ next.entry.flags) & EdfCatalog::ENTRY_STR_READY);
        const bool changed = strcmp(index[i].entry.file_prefix, next.entry.file_prefix) ||
            strcmp(index[i].entry.therapy_day, next.entry.therapy_day) ||
            ((index[i].entry.flags ^ next.entry.flags) & EdfCatalog::ENTRY_LIVE_COMPLETE);
        if (full || changed) {
            index_error = "source_fingerprint";
            if (!fingerprint(session, next)) { index_ready = false; return false; }
            const bool key_changed = !(index[i].key == next.key);
            if (changed || key_changed) {
                if (index[i].day) invalidate_day(index[i].day);
                invalidate_day(next.day);
            }
            index_error = "cache_retire";
            if (index[i].entry.file_prefix[0] && index[i].key.present && (changed || key_changed) &&
                !retire_cache(session, index[i])) { index_ready = false; return false; }
            detail_changed |= changed || key_changed;
            if (!changed && !key_changed) {
                next.start_ms = index[i].start_ms;
                next.end_ms = index[i].end_ms;
                next.event_from_ms = index[i].event_from_ms;
                next.event_to_ms = index[i].event_to_ms;
                next.bounds_known = index[i].bounds_known;
            }
        } else {
            next.key = index[i].key;
            next.start_ms = index[i].start_ms;
            next.end_ms = index[i].end_ms;
            next.event_from_ms = index[i].event_from_ms;
            next.event_to_ms = index[i].event_to_ms;
            next.bounds_known = index[i].bounds_known;
        }
        index[i] = next;
    }
    if (summary_changed) invalidate_str();
    catalog_generation = changes.catalog.generation;
    file_revision = files;
    indexed_invalidation = invalidation;
    index_ready = true;
    if (full) {
        cache_usage_known = false;
        if (!prune_cache(session)) { index_ready = false; index_error = "cache_prune"; return false; }
    }
    if ((detail_changed || summary_changed) && !invalidated) __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
    portENTER_CRITICAL(&status_mux);
    status.catalog_generation = catalog_generation;
    status.files_revision = file_revision;
    status.sessions = index_count;
    status.data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (detail_changed || summary_changed || invalidated) ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

struct Input {
    SdStorage::Reader file;
    ReportEdf::Reader reader;
    ReportEdf::Header *header = nullptr;
    uint8_t *record = nullptr;
    size_t record_capacity = 0;
    uint64_t record_offset = UINT64_MAX;
    char path[96] = {};
    const SdStorage::Session &session;
    const Job &job;
    Input(const SdStorage::Session &session, const Job &job) : session(session), job(job) {
        void *memory = aircannect::Memory::alloc_large(sizeof(ReportEdf::Header),
                                                      sizeof(ReportEdf::Header) <= INTERNAL_WORK);
        if (memory) header = new (memory) ReportEdf::Header;
    }
    ~Input() {
        aircannect::Memory::free(header);
        aircannect::Memory::free(record);
    }
    bool open(const SdStorage::Session &session, const IndexedSession &entry, uint8_t kind) {
        char source[96];
        source_path(entry, kind, "edf", source);
        if (!open_file(source)) return false;
        if (!ReportEdf::read_header(reader, *header)) return note_issue(job, "source_header", path);
        return prepare_record();
    }
    bool open_file(const char *source) {
        if (!header) return note_issue(job, "header_alloc");
        record_offset = UINT64_MAX;
        *header = {};
        snprintf(path, sizeof(path), "%s", source);
        if (!file.open(session, path)) return note_issue(job, "source_open", path);
        reader = {[](void *ctx, uint64_t offset, uint8_t *out, size_t count) {
            auto &input = *static_cast<Input *>(ctx);
            if (aborted(&input.job, input.session)) return false;
            auto &file = input.file;
            const auto &header = *input.header;
            if (input.record && header.record_bytes && offset >= header.header_bytes &&
                offset + count <= header.header_bytes + uint64_t(header.records) * header.record_bytes) {
                const uint64_t base = header.header_bytes +
                    ((offset - header.header_bytes) / header.record_bytes) * header.record_bytes;
                if (offset + count <= base + header.record_bytes) {
                    if (!input.load_record((base - header.header_bytes) / header.record_bytes)) return false;
                    memcpy(out, input.record + (offset - base), count);
                    return true;
                }
            }
            if (!file.seek(offset)) return note_issue(input.job, "source_seek", input.path);
            size_t done = 0;
            while (done < count) {
                const size_t take = std::min(count - done, SdStorage::READ_CHUNK_BYTES);
                if (file.read(out + done, take) != take)
                    return note_issue(input.job, "source_read", input.path);
                done += take;
            }
            return true;
        }, this, file.size()};
        return true;
    }
    bool prepare_record() {
        if (header->record_bytes > record_capacity) {
            // One actual record, at most the reader's validated 64 KiB limit.
            void *grown = aircannect::Memory::realloc_large(record, header->record_bytes,
                                                           header->record_bytes <= INTERNAL_WORK);
            if (grown) {
                record = static_cast<uint8_t *>(grown);
                record_capacity = header->record_bytes;
            }
        }
        if (record_capacity < header->record_bytes) return note_issue(job, "record_alloc", path);
        return true;
    }

    bool load_record(uint32_t index) {
        if (aborted(&job, session) || index >= header->records || !record ||
            record_capacity < header->record_bytes) return false;
        const uint64_t base = header->header_bytes + uint64_t(index) * header->record_bytes;
        if (base == record_offset) return true;
        record_offset = UINT64_MAX;
        if (!file.seek(base)) return note_issue(job, "source_seek", path);
        for (size_t at = 0; at < header->record_bytes;) {
            if (aborted(&job, session)) return false;
            const size_t take = std::min<size_t>(header->record_bytes - at, SdStorage::READ_CHUNK_BYTES);
            if (file.read(record + at, take) != take) return note_issue(job, "source_read", path);
            at += take;
        }
        record_offset = base;
        return true;
    }

    template <typename Sink>
    bool scan_records(const int *signals, size_t count, uint32_t first, uint32_t end, Sink consume) {
        if (first > end || end > header->records) return false;
        bool present = false;
        for (size_t c = 0; c < count; ++c) present |= signals[c] >= 0;
        if (!present) return true;
        for (uint32_t n = first; n < end; ++n) {
            if (!load_record(n)) return false;
            for (size_t c = 0; c < count; ++c) {
                if (signals[c] < 0) continue;
                ReportEdf::RecordSamples samples;
                if (aborted(&job, session) ||
                    !ReportEdf::record_samples(*header, record, header->record_bytes, signals[c], samples) ||
                    !consume(c, uint64_t(n) * samples.signal->samples_per_record, samples)) return false;
            }
        }
        return true;
    }

    template <typename Sink>
    bool scan_window(const int *signals, size_t count, int64_t from, int64_t to, Sink consume) {
        const int64_t origin = header->start_seconds * 1000;
        const double record_ms = header->record_seconds * 1000;
        const uint32_t first = std::min<double>(header->records,
            std::max<double>(0, std::floor((from - origin) / record_ms)));
        const uint32_t last = std::min<double>(header->records,
            std::max<double>(0, std::ceil((to - origin) / record_ms)));
        return scan_records(signals, count, first, last,
            [&](size_t c, uint64_t base, const ReportEdf::RecordSamples &samples) {
                const double step = record_ms / samples.signal->samples_per_record;
                const uint64_t begin = std::max<uint64_t>(base,
                    std::max<double>(0, std::floor((from - origin) / step)));
                const uint64_t end = std::min<uint64_t>(base + samples.signal->samples_per_record,
                    std::max<double>(0, std::ceil((to - origin) / step)));
                return consume(c, base, begin, end, samples);
            });
    }
};

struct StrLayoutCache {
    ReportEdf::Layout layout;
    ReportEdf::Signal *signals = nullptr;
    uint64_t bytes = 0;

    void reset() {
        aircannect::Memory::free(signals);
        signals = nullptr;
        layout = {};
        bytes = 0;
    }
    bool load(const ReportEdf::Reader &reader) {
        if (signals && bytes == reader.size) return true;
        reset();
        if (!ReportEdf::read_layout(reader, layout) || layout.record_seconds != 86400 ||
            layout.records > Air10StrTimeline::RECORD_LIMIT) return false;
        const size_t size = layout.signal_count * sizeof(*signals);
        signals = static_cast<ReportEdf::Signal *>(aircannect::Memory::alloc_large(size, size <= INTERNAL_WORK));
        if (!signals || !ReportEdf::read_signals(reader, layout, signals, layout.signal_count, true) ||
            layout.record_bytes > 4096) { reset(); return false; }
        for (size_t i = 0; i < layout.signal_count; ++i) {
            const auto &date = signals[i];
            if (strcmp(date.label, "Date")) continue;
            if (date.samples_per_record == 1 && date.digital_min == 0 &&
                date.physical_min == 0 && date.physical_max == date.digital_max) {
                bytes = reader.size;
                return true;
            }
            break;
        }
        reset();
        return false;
    }
} str_layout;

void invalidate_str() { str_layout.reset(); }

struct HistoricalStr {
    Input input;
    const ReportEdf::Signal *signals = nullptr;
    bool present = false;

    HistoricalStr(const SdStorage::Session &session, const Job &job) : input(session, job) {}

    const ReportEdf::Signal *find(const char *label) const {
        if (!signals || !input.header) return nullptr;
        for (size_t i = 0; i < input.header->signal_count; ++i)
            if (!strcmp(signals[i].label, label)) return &signals[i];
        return nullptr;
    }
    double value(const char *label, double scale = 1) const {
        const auto *signal = find(label);
        if (!present || !signal || signal->samples_per_record != 1) return NAN;
        const uint8_t *sample = input.record + signal->record_offset;
        const uint16_t raw = sample[0] | uint16_t(sample[1]) << 8;
        if (raw == UINT16_MAX) return NAN;
        const int32_t digital = signal->digital_min < 0 ? int16_t(raw) : int32_t(raw);
        return signal->valid(digital) ? signal->physical(digital) * scale : NAN;
    }
    bool load(uint16_t day) {
        if (!input.open_file("/STR.edf") || !str_layout.load(input.reader)) return false;
        auto &header = *input.header;
        static_cast<ReportEdf::Layout &>(header) = str_layout.layout;
        signals = str_layout.signals;
        if (!input.prepare_record()) return false;
        const auto *date = find("Date");
        const int64_t guessed = int64_t(day) - header.start_seconds / 86400;
        auto read = [&](uint32_t record) {
            uint8_t bytes[2];
            return input.reader.read(input.reader.context,
                header.header_bytes + uint64_t(record) * header.record_bytes + date->record_offset,
                bytes, sizeof(bytes));
        };
        // The current timeline is continuous. Older compact files are located
        // by their Date samples, never by substituting a neighbouring record.
        if (guessed >= 0 && guessed < header.records) {
            if (!read(uint32_t(guessed))) return false;
            present = true;
            if (value("Date") == day) {
                present = std::isfinite(value("Mode")) || std::isfinite(value("Duration"));
                input.file.close(); return true;
            }
        }
        for (uint32_t record = 0; record < header.records; ++record) {
            if (!read(record)) return false;
            present = true;
            if (value("Date") == day) {
                present = std::isfinite(value("Mode")) || std::isfinite(value("Duration"));
                input.file.close(); return true;
            }
        }
        present = false;
        input.file.close();
        return true;
    }
    void settings_json(Payload &body, uint16_t day) const {
        body.print(",\"settings\":{\"day\":%u,\"available\":%s,\"fields\":[", day, present ? "true" : "false");
        bool comma = false;
        auto append = [&](const char *tag) {
            const auto *def = var_lookup(tag);
            if (!def) return;
            const auto &schema = Air10Edf::str_schema();
            for (size_t i = 0; i < schema.signal_count; ++i) {
                if (strcmp(Air10Edf::str_signal_tag(i), tag)) continue;
                const auto &spec = schema.signals[i];
                const auto *signal = find(spec.label);
                const double saved = value(spec.label);
                if (!std::isfinite(saved) || !signal || strcmp(signal->unit, spec.dimension)) return;
                char text[48];
                if (def->type != SET_ENUM || saved < 0 || saved > INT32_MAX || saved != std::floor(saved) ||
                    !setting_enum_label(def, int(saved), text, sizeof(text)))
                    snprintf(text, sizeof(text), "%.8g", saved);
                // Text and labels come only from the static Air10 catalog or
                // numeric formatting, never live metadata or file strings.
                body.print("%s{\"tag\":\"%s\",\"label\":\"%s\",\"value\":\"%s\",\"unit\":\"%s\"}",
                           comma ? "," : "", tag, def->label, text, spec.dimension);
                comma = true;
                return;
            }
        };
        append("MOP");
        const double mode = value("Mode");
        if (std::isfinite(mode) && mode >= 0 && mode < MODE_COUNT && mode == std::floor(mode)) {
            for (const char *tag : MODE_LAYOUT[int(mode)]) { if (!tag) break; append(tag); }
            for (const char *tag : COMFORT_LAYOUT[int(mode)]) { if (!tag) break; append(tag); }
        }
        for (const char *tag : EPR_VARS) { if (!tag) break; append(tag); }
        append("SST");
        for (const char *tag : ACCESSORY_VARS) { if (!tag) break; append(tag); }
        body.print("]}");
    }
    double statistic(size_t channel, const char *suffix) const {
        const char *base = channel == 0 ? "TgtIPAP" : channel == 1 ? "TgtEPAP" : CHANNELS[channel].id;
        char label[32];
        snprintf(label, sizeof(label), "%s.%s", base, suffix);
        const auto *signal = find(label);
        if (!signal) return NAN;
        double scale = 1;
        if (channel == 3 && !strcmp(signal->unit, "L/s")) scale = 60;
        else if (channel == 6 && !strcmp(signal->unit, "L")) scale = 1000;
        else if (channel == 12 && !strcmp(signal->unit, "seconds")) scale = 1;
        else if (strcmp(signal->unit, CHANNELS[channel].unit)) return NAN;
        return value(label, scale);
    }
};

size_t total_bins(const CacheHeader &header) {
    size_t count = 0;
    for (const auto &channel : header.channels) count += channel.bins;
    return count;
}
bool allocate_data(SessionData &out, bool events = true) {
    const size_t count = total_bins(out.header);
    if (count > MAX_BINS || out.header.events > MAX_DISK_EVENTS) return false;
    if (count) {
        const size_t bytes = count * sizeof(uint64_t);
        out.bins = static_cast<uint64_t *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (!out.bins) return false;
        memset(out.bins, 0, bytes);
    }
    if (events && out.header.events) {
        const size_t bytes = std::min<size_t>(out.header.events, MAX_EVENTS) * sizeof(Event);
        out.events = static_cast<Event *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (!out.events) return false;
    }
    return true;
}

bool sane_header(const CacheHeader &h, const IndexedSession &entry, uint64_t bytes) {
    if (h.version != STATS_CACHE_VERSION || h.header_bytes != sizeof(h) ||
        memcmp(h.magic, "ABRPT01", 8) || !(h.key == entry.key) || h.day != entry.day ||
        h.events > MAX_DISK_EVENTS || total_bins(h) > MAX_BINS ||
        h.start_ms < 0 || h.end_ms < h.start_ms ||
        h.chart_start_ms < 0 || h.chart_end_ms < h.chart_start_ms ||
        h.event_from_ms < 0 || h.event_to_ms < h.event_from_ms ||
        !std::isfinite(h.used_seconds) || h.used_seconds < 0 || h.used_seconds > 86400 ||
        !std::isfinite(h.csr_seconds) || h.csr_seconds < 0 || h.csr_seconds > h.used_seconds ||
        !std::isfinite(h.leak_high_seconds) || h.leak_high_seconds < 0 || h.leak_high_seconds > 86400 ||
        bytes != sizeof(h) + total_bins(h) * sizeof(uint64_t) + h.events * sizeof(Event) ||
        h.bytes != bytes) return false;
    for (const auto &channel : h.channels) {
        if (!channel.bins) continue;
        if (!memchr(channel.signal.label, 0, sizeof(channel.signal.label)) ||
            !memchr(channel.signal.unit, 0, sizeof(channel.signal.unit)) ||
            ReportEdf::histogram_bins(channel.signal) != channel.bins ||
            !std::isfinite(channel.sum) || !std::isfinite(channel.coverage_seconds) ||
            channel.coverage_seconds < 0 || channel.coverage_seconds > 86400)
            return false;
    }
    return true;
}

bool load_cache(const SdStorage::Session &session, const IndexedSession &entry,
                SessionData &out) {
    char path[96];
    cache_path(entry, "stats", path);
    SdStorage::Reader reader;
    if (!reader.open(session, path) || reader.read(reinterpret_cast<uint8_t *>(&out.header),
        sizeof(out.header)) != sizeof(out.header) || !sane_header(out.header, entry, reader.size()) ||
        !allocate_data(out, false)) return false;
    const size_t bin_bytes = total_bins(out.header) * sizeof(uint64_t);
    for (size_t offset = 0; offset < bin_bytes;) {
        const size_t take = std::min(bin_bytes - offset, SdStorage::READ_CHUNK_BYTES);
        if (reader.read(reinterpret_cast<uint8_t *>(out.bins) + offset, take) != take) return false;
        offset += take;
    }
    return true;
}

bool read_event(Input &input, uint32_t record, Event &event) {
    ReportEdf::Annotation annotation;
    if (!ReportEdf::read_annotation(input.reader, *input.header, record, annotation))
        return note_issue(input.job, "annotation_record", input.path);
    event.time_ms = llround((input.header->start_seconds + annotation.onset_seconds) * 1000);
    event.duration_ms = annotation.has_duration ? llround(annotation.duration_seconds * 1000) : 0;
    event.kind = annotation.kind;
    return true;
}

struct PartGuard {
    const SdStorage::Session &session;
    const char *path;
    SdStorage::Writer writer;
    bool complete = false;
    ~PartGuard() {
        writer.close();
        if (complete) return;
        cache_usage_known = false;
        (void)session.run([&](fs::FS &fs) { return !fs.exists(path) || fs.remove(path); });
    }
};

bool start_part(PartGuard &, const uint8_t *, size_t);
bool finish_part(PartGuard &, const char *, uint64_t);

bool store_cache(const SdStorage::Session &session, const Job &job, const IndexedSession &entry,
                 SessionData &data) {
    if (data.header.events > MAX_DISK_EVENTS) return false;
    char path[96], part[96];
    cache_path(entry, "stats", path);
    cache_path(entry, "stats.part", part);
    data.header.bytes = sizeof(data.header) + total_bins(data.header) * sizeof(uint64_t) +
        data.header.events * sizeof(Event);
    if (!prune_cache(session, data.header.bytes, path)) return false;
    PartGuard part_guard{session, part};
    if (!start_part(part_guard, reinterpret_cast<const uint8_t *>(&data.header), sizeof(data.header)) ||
        !part_guard.writer.write(reinterpret_cast<const uint8_t *>(data.bins),
                                 total_bins(data.header) * sizeof(uint64_t))) return false;
    if (data.header.events <= MAX_EVENTS) {
        if (!part_guard.writer.write(reinterpret_cast<const uint8_t *>(data.events),
                                     data.header.events * sizeof(Event))) return false;
    } else {
        // Persist all events with a bounded block, not just the in-RAM prefix.
        Input input(session, job);
        Event block[32];
        uint32_t events = 0;
        size_t used = 0;
        for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
            if (!(entry.key.present & (1u << kind))) continue;
            if (!input.open(session, entry, kind)) return false;
            for (uint32_t i = 0; i < input.header->records; ++i) {
                Event event;
                if (!read_event(input, i, event)) return false;
                if (event.kind == ReportEdf::EventKind::Unknown) continue;
                block[used++] = event;
                ++events;
                if (used == 32) {
                    if (!part_guard.writer.write(reinterpret_cast<uint8_t *>(block), sizeof(block))) return false;
                    used = 0;
                }
            }
            input.file.close();
        }
        if (events != data.header.events || !part_guard.writer.write(
            reinterpret_cast<uint8_t *>(block), used * sizeof(Event))) return false;
    }
    part_guard.complete = finish_part(part_guard, path, data.header.bytes);
    if (part_guard.complete) {
        invalidate_day(entry.day);
        __atomic_add_fetch(&cache_generation, 1, __ATOMIC_RELEASE);
    }
    return part_guard.complete;
}

ReportEdf::Histogram histogram(const CachedChannel &channel, uint64_t *bins) {
    ReportEdf::Histogram out;
    out.signal = channel.signal;
    out.bins = bins;
    out.bin_count = channel.bins;
    out.count = channel.count;
    out.missing = channel.missing;
    out.sum = channel.sum;
    return out;
}

bool prepare_channels(Input &input, uint8_t file, SessionData &data) {
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        if (CHANNELS[c].file != file) continue;
        const int signal = ReportEdf::find_signal(*input.header, CHANNELS[c].signal);
        if (signal < 0) continue;
        auto &channel = data.header.channels[c];
        channel.signal = input.header->signals[signal];
        channel.bins = ReportEdf::histogram_bins(channel.signal);
        // Merge normalized physical scales while preserving exact digital bins.
        double scale = 1;
        if (c == 3 && !strcmp(channel.signal.unit, "L/s")) scale = 60;
        if (c == 6 && !strcmp(channel.signal.unit, "L")) scale = 1000;
        channel.signal.physical_min *= scale;
        channel.signal.physical_max *= scale;
        snprintf(channel.signal.unit, sizeof(channel.signal.unit), "%s", CHANNELS[c].unit);
        if (!channel.bins || total_bins(data.header) > MAX_BINS) return false;
    }
    return true;
}

bool scan_channels(Input &input, uint8_t file, SessionData &data) {
    int signals[CHANNEL_COUNT];
    size_t offsets[CHANNEL_COUNT], offset = 0;
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        offsets[c] = offset;
        offset += data.header.channels[c].bins;
        signals[c] = -1;
        if (CHANNELS[c].file != file || !data.header.channels[c].bins) continue;
        signals[c] = ReportEdf::find_signal(*input.header, CHANNELS[c].signal);
        if (signals[c] < 0) return false;
    }
    if (!input.scan_records(signals, CHANNEL_COUNT, 0, input.header->records,
        [&](size_t c, uint64_t, const ReportEdf::RecordSamples &samples) {
            auto &channel = data.header.channels[c];
            auto hist = histogram(channel, data.bins + offsets[c]);
            const uint32_t count = samples.signal->samples_per_record;
            for (uint32_t slot = 0; slot < count; ++slot) {
                const auto sample = samples[slot];
                if (!ReportEdf::add_sample(hist, sample.raw, sample.valid)) return false;
                if (c == 3 && sample.valid && channel.signal.physical(sample.raw) > LEAK_THRESHOLD)
                    data.header.leak_high_seconds += input.header->record_seconds / count;
            }
            channel.count = hist.count;
            channel.missing = hist.missing;
            channel.sum = hist.sum;
            return true;
        })) return false;
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        if (signals[c] >= 0) {
            auto &channel = data.header.channels[c];
            channel.coverage_seconds = channel.count * input.header->record_seconds /
                channel.signal.samples_per_record;
        }
    }
    return true;
}

int count_index(ReportEdf::EventKind kind) {
    switch (kind) {
        case ReportEdf::EventKind::OA: return 0;
        case ReportEdf::EventKind::CA: return 1;
        case ReportEdf::EventKind::UA: return 2;
        case ReportEdf::EventKind::H: return 3;
        case ReportEdf::EventKind::RERA: return 4;
        default: return -1;
    }
}
const char *event_name(ReportEdf::EventKind kind) {
    switch (kind) {
        case ReportEdf::EventKind::OA: return "oa";
        case ReportEdf::EventKind::CA: return "ca";
        case ReportEdf::EventKind::UA: return "ua";
        case ReportEdf::EventKind::H: return "h";
        case ReportEdf::EventKind::RERA: return "rera";
        case ReportEdf::EventKind::CsrStart: return "csr_start";
        case ReportEdf::EventKind::CsrEnd: return "csr_end";
        default: return "unknown";
    }
}

bool numeric_extent(const ReportEdf::Header &header, int64_t &start, int64_t &end) {
    start = header.start_seconds * 1000;
    const double duration = header.records * header.record_seconds;
    if (duration < 0 || duration > 86400 || start < 0) return false;
    end = start + llround(duration * 1000);
    return true;
}

struct CsrPairing {
    Event start;
    bool active = false, unmatched = false;

    template <typename Sink>
    void consume(const Event &event, Sink emit) {
        if (event.kind == ReportEdf::EventKind::CsrStart) {
            if (active) { emit(start, false); unmatched = true; }
            start = event;
            active = true;
        } else if (event.kind == ReportEdf::EventKind::CsrEnd) {
            if (active && event.time_ms > start.time_ms) {
                Event interval = event;
                interval.duration_ms = event.time_ms - start.time_ms;
                emit(interval, true);
            } else {
                if (active) emit(start, false);
                emit(event, false);
                unmatched = true;
            }
            active = false;
        } else emit(event, false);
    }
};

bool build_session(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, SessionData &out) {
    out.header.key = entry.key;
    out.header.day = entry.day;
    out.header.partial = entry.key.present != 31 || entry.key.checksums != 31;
    Input input(session, job);
    uint8_t readable_numeric = 0;
    // BRP contributes its header extent only. Its samples and overview cache
    // remain lazy, independent of the PLD/SAD summary statistics.
    for (uint8_t kind : {uint8_t(0), uint8_t(1), uint8_t(2)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            out.header.partial = true;
            input.file.close();
            continue;
        }
        if (kind && !prepare_channels(input, kind, out)) return false;
        readable_numeric |= 1u << kind;
        bool chart_present = !kind && ReportEdf::find_signal(*input.header, "Flow.40ms") >= 0;
        for (size_t c = 0; c < CHANNEL_COUNT; ++c)
            chart_present |= CHANNELS[c].file == kind && out.header.channels[c].bins;
        int64_t start = 0, end = 0;
        if (chart_present && numeric_extent(*input.header, start, end) && end > start) {
            if (!out.header.chart_end_ms) out.header.chart_start_ms = start;
            else out.header.chart_start_ms = std::min(out.header.chart_start_ms, start);
            out.header.chart_end_ms = std::max(out.header.chart_end_ms, end);
        }
        if (kind == 1) {
            out.header.usage_known = input.header->records == 0;
            int coverage = -1;
            for (const char *name : {"Press.2s", "EprPress.2s", "MaskPress.2s"}) {
                coverage = ReportEdf::find_signal(*input.header, name);
                if (coverage >= 0) break;
            }
            if (coverage >= 0) {
                ReportEdf::RecordedBounds bounds;
                if (!ReportEdf::recorded_bounds(input.reader, *input.header, coverage, bounds))
                    return false;
                out.header.recorded = bounds.present;
                out.header.usage_known |= bounds.present;
                if (bounds.present) {
                    out.header.start_ms = llround(bounds.begin_seconds * 1000);
                    out.header.end_ms = llround(bounds.end_seconds * 1000);
                    out.header.used_seconds = bounds.end_seconds - bounds.begin_seconds;
                }
            }
        }
        input.file.close();
    }
    // Cap event retention, but count every supported event independently.
    uint32_t event_capacity = 0;
    for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            out.header.partial = true;
            input.file.close();
            continue;
        }
        if (kind == 3) out.header.events_known = true;
        event_capacity += std::min<uint32_t>(input.header->records, MAX_EVENTS - event_capacity);
        input.file.close();
    }
    out.header.events = event_capacity;
    if (!allocate_data(out)) return false;
    for (uint8_t kind : {uint8_t(1), uint8_t(2)}) {
        if (!(readable_numeric & (1u << kind))) continue;
        if (!input.open(session, entry, kind) || !scan_channels(input, kind, out))
            return false;
        input.file.close();
    }
    uint32_t events = 0;
    CsrPairing csr;
    for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            if (kind == 3) out.header.events_known = false;
            out.header.partial = true;
            input.file.close();
            continue;
        }
        if (kind == 4) out.header.csr_known = true;
        for (uint32_t i = 0; i < input.header->records; ++i) {
            if (aborted(&job, session)) return false;
            Event event;
            if (!read_event(input, i, event)) {
                if (aborted(&job, session)) return false;
                if (kind == 3) out.header.events_known = false;
                if (kind == 4) out.header.csr_known = false;
                out.header.partial = true;
                break;
            }
            if (event.kind == ReportEdf::EventKind::Unknown) continue;
            if (kind == 4) csr.consume(event, [&](const Event &interval, bool paired) {
                if (!paired || !out.header.recorded) return;
                const int64_t begin = std::max(out.header.start_ms, interval.time_ms - interval.duration_ms);
                const int64_t end = std::min(out.header.end_ms, interval.time_ms);
                if (end > begin) out.header.csr_seconds += (end - begin) / 1000.0;
            });
            const int64_t begin = event.time_ms - event.duration_ms;
            if (!events) {
                out.header.event_from_ms = begin;
                out.header.event_to_ms = event.time_ms;
            } else {
                out.header.event_from_ms = std::min(out.header.event_from_ms, begin);
                out.header.event_to_ms = std::max(out.header.event_to_ms, event.time_ms);
            }
            const int count = count_index(event.kind);
            if (kind == 3 && count >= 0 && out.header.recorded &&
                event.time_ms >= out.header.start_ms && event.time_ms < out.header.end_ms)
                ++out.header.counts[count];
            if (events < event_capacity) out.events[events] = event;
            else out.header.events_truncated = true;
            ++events;
        }
        input.file.close();
    }
    out.header.events = events;
    out.header.csr_known &= !csr.active && !csr.unmatched && out.header.usage_known;
    if (events) std::sort(out.events, out.events + std::min<uint32_t>(events, event_capacity), [](const Event &a, const Event &b) {
        return a.time_ms < b.time_ms;
    });
    return true;
}

bool session_data(const SdStorage::Session &session, const Job &job,
                  const IndexedSession &entry, SessionData &out) {
    if (load_cache(session, entry, out)) {
        if (out.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
        return true;
    }
    out.~SessionData();
    new (&out) SessionData;
    if (!build_session(session, job, entry, out))
        return note_issue(job, "session_build", entry.entry.file_prefix);
    if (out.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
    // The cache is derived and replaceable; failed persistence does not discard
    // useful parsed data or modify any source EDF/sidecar.
    if (!store_cache(session, job, entry, out) && !aborted(&job, session))
        note_issue(job, "cache_write", entry.entry.file_prefix);
    return !aborted(&job, session);
}

uint16_t last_day(const Request &request) {
    if (request.day) return request.day;
    uint16_t day = 0;
    for (size_t i = 0; i < index_count; ++i)
        if ((index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE) &&
            (index[i].key.present || (index[i].entry.flags & EdfCatalog::ENTRY_STR_READY)))
            day = std::max(day, index[i].day);
    return day;
}
uint16_t first_day(const Request &request, uint16_t last) {
    const uint16_t days = request.view == View::Day ? 1 : request.period;
    return last >= days ? last - days + 1 : 0;
}
bool selected(const IndexedSession &entry, uint16_t first, uint16_t last) {
    return entry.day >= first && entry.day <= last &&
        (entry.entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE);
}

void start_json(Payload &body, uint32_t publication, uint32_t dataset) {
    body.print("{\"source\":\"edf\",\"revision\":%u,\"data_revision\":%u,",
               publication, dataset);
}
void append_event(Payload &body, const Event &event, bool &comma, const char *type = nullptr) {
    body.print("%s{\"time_ms\":%lld,\"duration_ms\":%lld,\"type\":\"%s\"}",
               comma ? "," : "", static_cast<long long>(event.time_ms),
               static_cast<long long>(event.duration_ms), type ? type : event_name(event.kind));
    comma = true;
}

bool days_json(Job &job) {
    const size_t bytes = index_count * sizeof(uint16_t);
    auto *days = bytes ? static_cast<uint16_t *>(aircannect::Memory::alloc_large(bytes,
                                                       bytes <= INTERNAL_WORK)) : nullptr;
    if (bytes && !days) return false;
    size_t count = 0;
    bool partial = false;
    for (size_t i = 0; i < index_count; ++i) {
        if (!(index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE)) continue;
        if ((index[i].key.present & 7) || (index[i].entry.flags & EdfCatalog::ENTRY_STR_READY)) days[count++] = index[i].day;
        if (index[i].key.present != 31 || index[i].key.checksums != 31) partial = true;
    }
    if (count) std::sort(days, days + count, std::greater<uint16_t>());
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"partial\":%s,\"days\":[", partial ? "true" : "false");
    for (size_t n = 0; n < count;) {
        const uint16_t day = days[n];
        size_t end = n + 1;
        while (end < count && days[end] == day) ++end;
        job.body.print("%s{\"day\":%u,\"session_count\":%u}", n ? "," : "", day,
                       static_cast<unsigned>(end - n));
        n = end;
    }
    job.body.print("]}");
    aircannect::Memory::free(days);
    return job.body.good();
}

struct Aggregate {
    ReportEdf::Histogram *channels = nullptr;
    double coverage[CHANNEL_COUNT] = {};
    bool incompatible[CHANNEL_COUNT] = {};
    Aggregate() {
        const size_t bytes = CHANNEL_COUNT * sizeof(*channels);
        channels = static_cast<ReportEdf::Histogram *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (channels) for (size_t c = 0; c < CHANNEL_COUNT; ++c) new (&channels[c]) ReportEdf::Histogram;
    }
    ~Aggregate() {
        if (channels) for (size_t c = 0; c < CHANNEL_COUNT; ++c) aircannect::Memory::free(channels[c].bins);
        aircannect::Memory::free(channels);
    }
    bool merge(const SessionData &data, bool &partial) {
        size_t offset = 0;
        for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
            const auto &cached = data.header.channels[c];
            if (cached.bins && !incompatible[c]) {
                auto next = histogram(cached, data.bins + offset);
                if (!channels[c].bins) {
                    const size_t bytes = cached.bins * sizeof(uint64_t);
                    auto *bins = static_cast<uint64_t *>(aircannect::Memory::alloc_large(bytes,
                                                                   bytes <= INTERNAL_WORK));
                    if (!bins || !ReportEdf::init_histogram(cached.signal, bins,
                                                           cached.bins, channels[c])) {
                        aircannect::Memory::free(bins);
                        return false;
                    }
                }
                if (!ReportEdf::merge_histogram(channels[c], next)) {
                    incompatible[c] = true;
                    partial = true;
                } else coverage[c] += cached.coverage_seconds;
            }
            offset += cached.bins;
        }
        return true;
    }
};

bool history_json(const SdStorage::Session &session, Job &job, bool build);

bool summary_json(const SdStorage::Session &session, Job &job) {
    Aggregate aggregate;
    if (!aggregate.channels) return note_issue(job, "aggregate_alloc");
    const uint16_t last = last_day(job.request), first = first_day(job.request, last);
    bool partial = false, session_comma = false;
    bool usage_known = true, events_known = true, csr_known = true;
    uint32_t sessions = 0;
    uint64_t counts[5] = {};
    double seconds = 0, csr_seconds = 0, leak_high_seconds = 0;
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"view\":\"%s\",\"day\":%u,\"first_day\":%u,\"last_day\":%u,\"sessions\":[",
                   job.request.view == View::Day ? "day" : "period", last, first, last);
    for (size_t i = 0; i < index_count; ++i) {
        auto &entry = index[i];
        if (!selected(entry, first, last)) continue;
        if (aborted(&job, session)) return false;
        if (!(entry.key.present & 7)) {
            note_issue(job, "numeric_files_missing", entry.entry.file_prefix);
            partial = true;
            usage_known = events_known = csr_known = false;
            continue;
        }
        SessionData data;
        if (!session_data(session, job, entry, data)) {
            if (aborted(&job, session)) return false;
            partial = true;
            usage_known = events_known = csr_known = false;
            continue;
        }
        partial |= data.header.partial;
        usage_known &= data.header.usage_known;
        events_known &= data.header.usage_known;
        csr_known &= data.header.csr_known;
        csr_seconds += data.header.csr_seconds;
        leak_high_seconds += data.header.leak_high_seconds;
        entry.bounds_known = data.header.chart_end_ms > data.header.chart_start_ms;
        entry.start_ms = data.header.chart_start_ms;
        entry.end_ms = data.header.chart_end_ms;
        entry.event_from_ms = data.header.event_from_ms;
        entry.event_to_ms = data.header.event_to_ms;
        if (!aggregate.merge(data, partial)) return false;
        if (data.header.recorded) {
            seconds += data.header.used_seconds;
            events_known &= data.header.events_known;
            for (size_t c = 0; c < 5; ++c) counts[c] += data.header.counts[c];
        }
        job.body.print("%s{\"id\":%u,\"day\":%u,\"prefix\":\"%s\",\"start_ms\":",
                       session_comma ? "," : "", static_cast<unsigned>(i), entry.day, entry.entry.file_prefix);
        if (entry.bounds_known)
            job.body.print("%lld,\"end_ms\":%lld}", static_cast<long long>(entry.start_ms),
                           static_cast<long long>(entry.end_ms));
        else job.body.print("null,\"end_ms\":null}");
        session_comma = true;
        ++sessions;
    }
    HistoricalStr str(session, job);
    bool str_expected = false;
    if (job.request.view == View::Day)
        for (size_t i = 0; i < index_count; ++i)
            str_expected |= index[i].day == last && (index[i].entry.flags & EdfCatalog::ENTRY_STR_READY);
    if (str_expected && !str.load(last)) {
        if (aborted(&job, session)) return false;
        str.present = false;
        partial = true;
        note_issue(job, "str_read", "/STR.edf");
    }
    job.body.print("],\"present\":%s,\"partial\":%s,\"used_seconds\":",
                   sessions || str.present ? "true" : "false", partial ? "true" : "false");
    if (usage_known && seconds > 0) job.body.print("%.10g", seconds);
    else job.body.print("null");
    job.body.print(",\"rdi\":");
    if (seconds > 0 && events_known) job.body.print("%.10g", (counts[0] + counts[1] + counts[2] + counts[3] + counts[4]) * 3600.0 / seconds);
    else job.body.print("null");
    job.body.print(",\"csr_seconds\":");
    if (csr_known && seconds > 0) job.body.print("%.10g", csr_seconds);
    else job.body.print("null");
    job.body.print(",\"leak_threshold\":%.10g,\"leak_high_seconds\":", LEAK_THRESHOLD);
    if (!aggregate.incompatible[3] && aggregate.coverage[3] > 0) job.body.print("%.10g", leak_high_seconds);
    else job.body.print("null");
    job.body.print(",\"session_count\":%u,\"ahi\":", sessions);
    if (seconds > 0 && events_known) job.body.print("%.10g", (counts[0] + counts[1] + counts[2] + counts[3]) * 3600.0 / seconds);
    else job.body.print("null");
    job.body.print(",\"events_known\":%s,", events_known && seconds > 0 ? "true" : "false");
    if (events_known && seconds > 0) job.body.print("\"counts\":{\"oa\":%llu,\"ca\":%llu,\"ua\":%llu,\"h\":%llu,\"rera\":%llu},",
                   static_cast<unsigned long long>(counts[0]), static_cast<unsigned long long>(counts[1]),
                   static_cast<unsigned long long>(counts[2]), static_cast<unsigned long long>(counts[3]),
                   static_cast<unsigned long long>(counts[4]));
    else job.body.print("\"counts\":{\"oa\":null,\"ca\":null,\"ua\":null,\"h\":null,\"rera\":null},");
    job.body.print("\"channels\":[");
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        ReportEdf::Statistics stats;
        const bool present = !aggregate.incompatible[c] && aggregate.channels[c].bins &&
            ReportEdf::summarize(aggregate.channels[c], stats) && stats.present;
        job.body.print("%s{\"id\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\",\"count\":%llu,",
                       c ? "," : "", CHANNELS[c].id, CHANNELS[c].label, CHANNELS[c].unit,
                       static_cast<unsigned long long>(present ? stats.count : 0));
        job.body.print("\"coverage_seconds\":%.10g,", present ? aggregate.coverage[c] : 0);
        const double values[] = {present ? stats.min : NAN, present ? stats.mean : NAN,
            present ? stats.median : str.statistic(c, "50"),
            present ? stats.p95 : str.statistic(c, "95"), present ? stats.max : str.statistic(c, "Max")};
        constexpr const char *keys[] = {"min", "mean", "median", "p95", "max"};
        bool str_value = false;
        for (size_t n = 0; n < 5; ++n) {
            job.body.print("\"%s\":", keys[n]);
            if (std::isfinite(values[n])) { job.body.print("%.10g,", values[n]); str_value |= !present; }
            else job.body.print("null,");
        }
        job.body.print("\"source\":\"%s\"}", present ? "edf" : str_value ? "str" : "");
    }
    job.body.print("]");
    if (job.request.view == View::Day) str.settings_json(job.body, last);
    if (job.request.view == View::Day && !history_json(session, job, false)) return false;
    job.body.print("}");
    return job.body.good();
}

struct Cell {
    float low = INFINITY, high = -INFINITY;
    uint32_t covered_ms = 0;
};
struct ReducedLevel {
    int64_t start_ms = 0;
    uint32_t count = 0;
    uint64_t offset = 0;
};
struct ReducedChannel {
    int64_t end_ms = 0;
    ReducedLevel levels[LEVEL_COUNT];
};
struct ReducedHeader {
    char magic[8] = {'A','B','S','E','R','0','1',0};
    uint32_t version = SERIES_CACHE_VERSION, header_bytes = sizeof(ReducedHeader);
    uint64_t bytes = 0;
    SourceKey key;
    uint32_t fine = 0;
    uint32_t fine_stride[3] = {};
    ReducedChannel channels[CHART_CHANNEL_COUNT + 1];
};

uint8_t chart_file(size_t c) { return c ? CHANNELS[c - 1].file : 0; }
const char *chart_signal(size_t c) { return c ? CHANNELS[c - 1].signal : "Flow.40ms"; }
double chart_scale(size_t c, const ReportEdf::Signal &signal) {
    if ((c == 0 || c == 4) && !strcmp(signal.unit, "L/s")) return 60;
    if (c == 7 && !strcmp(signal.unit, "L")) return 1000;
    return 1;
}

void chart_signals(const ReportEdf::Header &header, uint8_t kind,
                   int (&signals)[CHART_CHANNEL_COUNT + 1]) {
    for (size_t c = 0; c <= CHART_CHANNEL_COUNT; ++c)
        signals[c] = chart_file(c) == kind ? ReportEdf::find_signal(header, chart_signal(c)) : -1;
}

bool start_part(PartGuard &part, const uint8_t *header, size_t bytes) {
    if (!part.session.run([&](fs::FS &fs) {
        if ((!fs.exists("/airbridge") && !fs.mkdir("/airbridge")) ||
            (!fs.exists(CACHE_DIR) && !fs.mkdir(CACHE_DIR))) return false;
        char directory[96];
        snprintf(directory, sizeof(directory), "%s", part.path);
        *strrchr(directory, '/') = 0;
        if (!fs.exists(directory) && !fs.mkdir(directory)) return false;
        return !fs.exists(part.path) || fs.remove(part.path);
    })) return false;
    return part.writer.open(part.session, part.path) && part.writer.write(header, bytes);
}

bool finish_part(PartGuard &part, const char *path, uint64_t bytes) {
    if (!part.writer.finish(bytes)) return false;
    uint64_t old_bytes = 0;
    const bool success = part.session.run([&](fs::FS &fs) {
        fs::File old = fs.open(path, FILE_READ);
        if (old) old_bytes = old.size();
        old.close();
        return (!fs.exists(path) || fs.remove(path)) && fs.rename(part.path, path);
    });
    if (success && cache_usage_known) cache_bytes = cache_bytes - std::min(cache_bytes, old_bytes) + bytes;
    else cache_usage_known = false;
    return success;
}

bool cache_name(const char *input, const char *&base) {
    base = strrchr(input, '/');
    base = base ? base + 1 : input;
    if (strlen(base) < 21 || strlen(base) > 32) return false;
    for (size_t n = 0; n < 15; ++n)
        if (n == 8 ? base[n] != '_' : base[n] < '0' || base[n] > '9') return false;
    if (strcmp(base + 15, ".stats") && strcmp(base + 15, ".series") &&
        strcmp(base + 15, ".stats.part") && strcmp(base + 15, ".series.part")) return false;
    return true;
}

bool migrate_cache(const SdStorage::Session &session) {
    SdStorage::Reader directory;
    if (!directory.open(session, CACHE_DIR, true))
        return session.run([&](fs::FS &fs) { return !fs.exists(CACHE_DIR); });
    bool end = false;
    SdStorage::Reader::Entry file;
    while (!end) {
        if (!directory.next(file, end)) return false;
        const char *base = nullptr;
        if (end || file.directory || !cache_name(file.name, base) || strstr(base, ".part")) continue;
        for (size_t i = 0; i < index_count; ++i) {
            const auto &entry = index[i];
            if (strncmp(base, entry.entry.file_prefix, 15)) continue;
            char old[96], path[96], parent[96];
            snprintf(old, sizeof(old), "%s/%s", CACHE_DIR, base);
            cache_path(entry, base + 16, path);
            snprintf(parent, sizeof(parent), "%s/%s", CACHE_DIR, entry.entry.therapy_day);
            // A rename is restart-safe. Never overwrite an independently existing copy.
            if (!session.run([&](fs::FS &fs) {
                return fs.exists(path) || ((!fs.exists(parent) ? fs.mkdir(parent) : true) && fs.rename(old, path));
            })) return false;
            break;
        }
    }
    return true;
}

bool prune_cache(const SdStorage::Session &session, uint64_t needed, const char *keep) {
    if (needed > MAX_SD_CACHE) return false;
    if (cache_usage_known && cache_bytes + needed <= MAX_SD_CACHE) return true;
    if (!migrate_cache(session)) return false;
    while (true) {
        SdStorage::Reader directory;
        if (!directory.open(session, CACHE_DIR, true)) {
            bool missing = false;
            if (!session.run([&](fs::FS &fs) { missing = !fs.exists(CACHE_DIR); return true; })) return false;
            if (missing) { cache_bytes = 0; cache_usage_known = true; return true; }
            return false;
        }
        uint64_t total = 0, oldest_bytes = 0;
        char oldest[96] = {};
        auto visit = [&](const char *parent, const SdStorage::Reader::Entry &file) {
            char path[96];
            const char *base = nullptr;
            if (!cache_name(file.name, base)) return true;
            snprintf(path, sizeof(path), "%s/%s", parent, base);
            bool rebuildable = false;
            for (size_t i = 0; i < index_count; ++i)
                if (index[i].key.present == 31 && !strncmp(base, index[i].entry.file_prefix, 15)) {
                    rebuildable = true;
                    break;
                }
            if (strstr(base, ".part"))
                return session.run([&](fs::FS &fs) { return fs.remove(path); });
            total += file.size;
            if (rebuildable && (!keep || strcmp(path, keep)) && (!oldest[0] || strcmp(path, oldest) < 0)) {
                snprintf(oldest, sizeof(oldest), "%s", path);
                oldest_bytes = file.size;
            }
            return true;
        };
        bool end = false;
        SdStorage::Reader::Entry file;
        while (!end) {
            if (!directory.next(file, end)) return false;
            if (end) break;
            if (!file.directory) {
                if (!visit(CACHE_DIR, file)) return false;
                continue;
            }
            const char *name = strrchr(file.name, '/');
            name = name ? name + 1 : file.name;
            uint16_t day = 0;
            if (!catalog_day(name, day)) continue;
            char parent[96];
            snprintf(parent, sizeof(parent), "%s/%s", CACHE_DIR, name);
            SdStorage::Reader children;
            if (!children.open(session, parent, true)) return false;
            bool last = false;
            while (!last) {
                if (!children.next(file, last)) return false;
                if (!last && !file.directory && !visit(parent, file)) return false;
            }
        }
        directory.close();
        cache_bytes = total;
        cache_usage_known = true;
        if (total + needed <= MAX_SD_CACHE) return true;
        if (!oldest[0] || !session.run([&](fs::FS &fs) { return fs.remove(oldest); })) return false;
        cache_bytes -= oldest_bytes;
        // Retire oldest cache entries only. Missing cache is rebuilt from EDF.
        if (cache_bytes + needed <= MAX_SD_CACHE) return true;
    }
}

bool reduced_layout(ReducedHeader &out, bool initialize) {
    if (out.fine > 1) return false;
    uint64_t offset = sizeof(out);
    for (const auto &channel : out.channels) {
        if (channel.end_ms < 0) return false;
        for (size_t n = 0; n < LEVEL_COUNT; ++n) {
            const auto &level = channel.levels[n];
            const uint64_t limit = n ? 86400000 / LEVEL_MS[n] + 2
                                     : (86400000 / FINE_BLOCK_MS + 2) * FINE_BLOCK_CELLS;
            if ((!n && !out.fine && level.count) || level.count > limit ||
                level.start_ms < 0 || level.start_ms % LEVEL_MS[n]) return false;
        }
    }
    // Fine data is minute-major within each source file. Coarser levels keep
    // their contiguous channel-major layout, independent of recording length.
    for (uint8_t kind = 0; kind < 3; ++kind) {
        uint32_t stride = 0, count = 0;
        int64_t start = 0;
        for (size_t c = 0; c <= CHART_CHANNEL_COUNT; ++c) {
            if (chart_file(c) != kind) continue;
            auto &level = out.channels[c].levels[0];
            if (!level.count) {
                if (level.offset) return false;
                continue;
            }
            if (level.count % FINE_BLOCK_CELLS || level.start_ms % FINE_BLOCK_MS ||
                (stride && (level.count != count || level.start_ms != start))) return false;
            count = level.count;
            start = level.start_ms;
            const uint64_t channel_offset = offset + uint64_t(stride) * sizeof(Cell);
            if (initialize) level.offset = channel_offset;
            else if (level.offset != channel_offset) return false;
            stride += FINE_BLOCK_CELLS;
        }
        if (initialize) out.fine_stride[kind] = stride;
        else if (out.fine_stride[kind] != stride) return false;
        offset += uint64_t(count / FINE_BLOCK_CELLS) * stride * sizeof(Cell);
    }
    for (auto &channel : out.channels) {
        for (size_t n = 1; n < LEVEL_COUNT; ++n) {
            auto &level = channel.levels[n];
            if (initialize) level.offset = offset;
            else if (level.offset != offset) return false;
            offset += uint64_t(level.count) * sizeof(Cell);
        }
    }
    if (initialize) out.bytes = offset;
    return offset == out.bytes;
}

bool reduced_header(const SdStorage::Session &session, const IndexedSession &entry,
                    ReducedHeader &out) {
    char path[96];
    cache_path(entry, "series", path);
    SdStorage::Reader reader;
    return reader.open(session, path) && reader.read(reinterpret_cast<uint8_t *>(&out), sizeof(out)) == sizeof(out) &&
        !memcmp(out.magic, "ABSER01", 8) && out.version == SERIES_CACHE_VERSION &&
        out.header_bytes == sizeof(out) && out.key == entry.key && out.bytes == reader.size() &&
        reduced_layout(out, false);
}

void cell_sample(Cell *cells, uint32_t count, int64_t start_ms, int64_t step_ms,
                 int64_t begin, int64_t end, double value, bool valid) {
    if (!valid || end <= start_ms) return;
    int64_t cell = std::max<int64_t>(0, (begin - start_ms) / step_ms);
    while (cell < count && start_ms + cell * step_ms < end) {
        const int64_t left = std::max(begin, start_ms + cell * step_ms);
        const int64_t right = std::min(end, start_ms + (cell + 1) * step_ms);
        if (right > left) {
            cells[cell].low = std::min(cells[cell].low, static_cast<float>(value));
            cells[cell].high = std::max(cells[cell].high, static_cast<float>(value));
            cells[cell].covered_ms += right - left;
        }
        ++cell;
    }
}

bool build_reduced(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, bool fine, ReducedHeader &out) {
    out = {};
    out.key = entry.key;
    out.fine = fine;
    Input input(session, job);
    for (uint8_t kind = 0; kind < 3; ++kind) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) return false;
        int64_t start = 0, end = 0;
        if (!numeric_extent(*input.header, start, end)) return false;
        for (size_t c = 0; c <= CHART_CHANNEL_COUNT; ++c) {
            if (chart_file(c) != kind || ReportEdf::find_signal(*input.header, chart_signal(c)) < 0)
                continue;
            auto &channel = out.channels[c];
            channel.end_ms = end;
            for (size_t n = 0; n < LEVEL_COUNT; ++n) {
                if (!n && !fine) continue;
                auto &level = channel.levels[n];
                const int64_t block = n ? LEVEL_MS[n] : FINE_BLOCK_MS;
                level.start_ms = start / block * block;
                level.count = end > start ? ((end - level.start_ms + block - 1) / block) * (block / LEVEL_MS[n]) : 0;
            }
        }
        input.file.close();
    }
    if (!reduced_layout(out, true)) return false;
    char path[96], part[96];
    cache_path(entry, "series", path);
    cache_path(entry, "series.part", part);
    if (!prune_cache(session, out.bytes, path)) return false;
    PartGuard part_guard{session, part};
    if (!start_part(part_guard, reinterpret_cast<const uint8_t *>(&out), sizeof(out))) return false;
    // Keep only the coarse levels (under 1.5 MB for 24 h x 12 channels).
    // Fine cells are appended one minute at a time, not retained for the day.
    const uint64_t coarse_offset = out.channels[0].levels[1].offset;
    const size_t bytes = out.bytes - coarse_offset;
    std::unique_ptr<Cell, decltype(&aircannect::Memory::free)> cells(
        bytes ? static_cast<Cell *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK)) : nullptr,
        aircannect::Memory::free);
    if (bytes && !cells) return false;
    for (size_t i = 0; i < bytes / sizeof(Cell); ++i) new (cells.get() + i) Cell;
    for (uint8_t kind = 0; kind < 3; ++kind) {
        if (!(entry.key.present & (1u << kind))) continue;
        const size_t stride = out.fine_stride[kind];
        if (!input.open(session, entry, kind)) return false;
        int signals[CHART_CHANNEL_COUNT + 1];
        chart_signals(*input.header, kind, signals);
        size_t fine_offsets[CHART_CHANNEL_COUNT + 1], fine_count = 0;
        for (size_t c = 0; c <= CHART_CHANNEL_COUNT; ++c) {
            fine_offsets[c] = fine_count;
            if (signals[c] >= 0) fine_count += FINE_BLOCK_CELLS;
        }
        if (fine && fine_count != stride && input.header->records) return false;
        const size_t fine_bytes = stride * sizeof(Cell);
        std::unique_ptr<Cell, decltype(&aircannect::Memory::free)> fine_cells(
            fine_bytes ? static_cast<Cell *>(aircannect::Memory::alloc_large(fine_bytes, fine_bytes <= INTERNAL_WORK)) : nullptr,
            aircannect::Memory::free);
        if (fine_bytes && !fine_cells) return false;
        int64_t start = 0, end = 0;
        if (!numeric_extent(*input.header, start, end)) return false;
        const int64_t origin = input.header->start_seconds * 1000;
        for (int64_t block = start / FINE_BLOCK_MS * FINE_BLOCK_MS; block < end; block += FINE_BLOCK_MS) {
            for (size_t i = 0; i < stride; ++i) new (fine_cells.get() + i) Cell;
            if (!input.scan_window(signals, CHART_CHANNEL_COUNT + 1, block, block + FINE_BLOCK_MS,
                [&](size_t c, uint64_t base, uint64_t first, uint64_t last, const ReportEdf::RecordSamples &samples) {
                    const auto &channel = out.channels[c];
                    const double step = input.header->record_seconds * 1000 / samples.signal->samples_per_record;
                    const double scale = chart_scale(c, *samples.signal);
                    for (uint64_t i = first; i < last; ++i) {
                        const auto sample = samples[i - base];
                        const int64_t begin = std::max<int64_t>(block, origin + llround(i * step));
                        const int64_t end = std::min<int64_t>(block + FINE_BLOCK_MS, origin + llround((i + 1) * step));
                        if (fine)
                            cell_sample(fine_cells.get() + fine_offsets[c], FINE_BLOCK_CELLS, block, LEVEL_MS[0],
                                        begin, end, sample.value * scale, sample.valid);
                        for (size_t n = 1; n < LEVEL_COUNT; ++n) {
                            const auto &level = channel.levels[n];
                            cell_sample(cells.get() + (level.offset - coarse_offset) / sizeof(Cell),
                                        level.count, level.start_ms, LEVEL_MS[n], begin, end,
                                        sample.value * scale, sample.valid);
                        }
                    }
                    return true;
                })) return false;
            if (!part_guard.writer.write(reinterpret_cast<const uint8_t *>(fine_cells.get()), fine_bytes)) return false;
        }
        input.file.close();
    }
    if (!part_guard.writer.write(reinterpret_cast<const uint8_t *>(cells.get()), bytes)) return false;
    part_guard.complete = finish_part(part_guard, path, out.bytes);
    return part_guard.complete;
}

struct Bucket {
    float low = INFINITY, high = -INFINITY;
    uint64_t covered_ms = 0;
};
struct Series {
    Bucket *buckets = nullptr;
    uint16_t px = 0;
    int64_t from_ms = 0, to_ms = 0, step_ms = 0, level_ms = 0;
    size_t level_index = 0;
    bool partial = false;
    explicit Series(const Request &request) {
        from_ms = request.from_ms;
        to_ms = request.to_ms;
        step_ms = (to_ms - from_ms + request.px - 1) / request.px;
        if (step_ms >= LEVEL_MS[0]) {
            for (size_t n = 0; n < LEVEL_COUNT; ++n)
                if (LEVEL_MS[n] <= step_ms) level_index = n;
            level_ms = LEVEL_MS[level_index];
            from_ms = from_ms / level_ms * level_ms;
            step_ms = ((request.to_ms - from_ms + request.px - 1) / request.px + level_ms - 1) /
                level_ms * level_ms;
            px = (request.to_ms - from_ms + step_ms - 1) / step_ms;
            to_ms = from_ms + px * step_ms;
        } else px = (to_ms - from_ms + step_ms - 1) / step_ms;
        const size_t count = px * (CHART_CHANNEL_COUNT + 1), bytes = count * sizeof(Bucket);
        buckets = static_cast<Bucket *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (buckets) for (size_t i = 0; i < count; ++i) new (&buckets[i]) Bucket;
    }
    ~Series() { aircannect::Memory::free(buckets); }
    void add(size_t c, int64_t begin, int64_t end, float low, float high, uint64_t covered) {
        if (!covered || end <= from_ms || begin >= to_ms) return;
        const int64_t cell = std::max<int64_t>(0, (begin - from_ms) / step_ms);
        if (cell >= px) return;
        Bucket &bucket = buckets[c * px + cell];
        bucket.low = std::min(bucket.low, low);
        bucket.high = std::max(bucket.high, high);
        bucket.covered_ms += covered;
    }
};

void series_channel_json(Payload &out, const Series &series, size_t channel) {
    auto format_bucket = [&](size_t at, char *text) -> size_t {
        const auto &bucket = series.buckets[channel * series.px + at];
        if (!bucket.covered_ms || !std::isfinite(bucket.low) || !std::isfinite(bucket.high))
            return 0;
        const uint64_t expected = std::min<int64_t>(series.step_ms, series.to_ms -
            (series.from_ms + int64_t(at) * series.step_ms));
        return snprintf(text, 64, "%.7g,%.7g,%llu", bucket.low, bucket.high,
                        static_cast<unsigned long long>(std::min(bucket.covered_ms, expected)));
    };
    auto visit_runs = [&](auto &&visit) {
        char value[64], next[64];
        const auto *buckets = series.buckets + channel * series.px;
        size_t begin = 0, length = format_bucket(0, value);
        for (size_t at = 1; at <= series.px; ++at) {
            if (at < series.px) {
                const auto &bucket = buckets[at], &previous = buckets[at - 1];
                const uint64_t expected = std::min<int64_t>(series.step_ms, series.to_ms -
                    (series.from_ms + int64_t(at) * series.step_ms));
                // The preceding bucket cannot be the clipped tail. Bitwise
                // float equality also distinguishes +0 from -0.
                if (std::min(bucket.covered_ms, expected) ==
                    std::min(previous.covered_ms, static_cast<uint64_t>(series.step_ms)) &&
                    !memcmp(&bucket.low, &previous.low, sizeof(bucket.low)) &&
                    !memcmp(&bucket.high, &previous.high, sizeof(bucket.high)))
                    continue;
            }
            const size_t next_length = at < series.px ? format_bucket(at, next) : 0;
            // Compare wire values, including tail-clipped coverage and the
            // existing seven-significant-digit representation, not raw floats.
            if (at < series.px && length == next_length && !memcmp(value, next, length))
                continue;
            visit(at - begin, value, length);
            begin = at;
            length = next_length;
            if (length) memcpy(value, next, length + 1);
        }
    };

    size_t dense_bytes = sizeof("\"buckets\":[]") - 1 + series.px - 1;
    size_t run_bytes = sizeof("\"runs\":[]") - 1, run_count = 0;
    visit_runs([&](size_t count, const char *, size_t length) {
        dense_bytes += count * (length ? length + 2 : 4);
        run_bytes += snprintf(nullptr, 0, "%zu", count) + 2 +
            (length ? length + 1 : 0) + (run_count++ ? 1 : 0);
    });
    const bool compact = run_bytes < dense_bytes;
    out.print("\"%s\":[", compact ? "runs" : "buckets");
    bool comma = false;
    visit_runs([&](size_t count, const char *value, size_t length) {
        if (compact) {
            if (length) out.print("%s[%zu,%s]", comma ? "," : "", count, value);
            else out.print("%s[%zu]", comma ? "," : "", count);
            comma = true;
        } else {
            for (size_t i = 0; i < count; ++i) {
                if (length) out.print("%s[%s]", comma ? "," : "", value);
                else out.print("%snull", comma ? "," : "");
                comma = true;
            }
        }
    });
    out.print("]");
}

bool cached_series(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, Series &series, ReducedHeader &header) {
    const bool fine = !series.level_index;
    if (!reduced_header(session, entry, header) || (fine && !header.fine))
        if (!build_reduced(session, job, entry, fine, header))
            return note_issue(job, "series_cache_build", entry.entry.file_prefix);
    char path[96];
    cache_path(entry, "series", path);
    SdStorage::Reader reader;
    if (!reader.open(session, path)) return false;
    auto add = [&](size_t c, int64_t begin, const Cell &cell) {
        if (cell.covered_ms > series.level_ms ||
            (cell.covered_ms && (!std::isfinite(cell.low) ||
             !std::isfinite(cell.high) || cell.low > cell.high))) return false;
        series.add(c, begin, begin + series.level_ms, cell.low, cell.high, cell.covered_ms);
        return true;
    };
    if (!series.level_index) {
        const size_t stride = *std::max_element(header.fine_stride, header.fine_stride + 3);
        const size_t bytes = stride * sizeof(Cell);
        std::unique_ptr<Cell, decltype(&aircannect::Memory::free)> cells(
            bytes ? static_cast<Cell *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK)) : nullptr,
            aircannect::Memory::free);
        if (bytes && !cells) return false;
        for (uint8_t kind = 0; kind < 3; ++kind) {
            if (!header.fine_stride[kind]) continue;
            size_t first_channel = 0;
            while (chart_file(first_channel) != kind || !header.channels[first_channel].levels[0].count)
                ++first_channel;
            const auto &level = header.channels[first_channel].levels[0];
            const uint32_t first = std::min<int64_t>(level.count / FINE_BLOCK_CELLS,
                std::max<int64_t>(0, (series.from_ms - level.start_ms) / FINE_BLOCK_MS));
            const uint32_t last = std::min<int64_t>(level.count / FINE_BLOCK_CELLS,
                std::max<int64_t>(0, (series.to_ms - level.start_ms + FINE_BLOCK_MS - 1) / FINE_BLOCK_MS));
            const size_t block_bytes = header.fine_stride[kind] * sizeof(Cell);
            if (!reader.seek(level.offset + uint64_t(first) * block_bytes)) return false;
            for (uint32_t n = first; n < last; ++n) {
                if (aborted(&job, session) || reader.read(reinterpret_cast<uint8_t *>(cells.get()),
                    block_bytes) != block_bytes) return false;
                size_t at = 0;
                const int64_t block = level.start_ms + int64_t(n) * FINE_BLOCK_MS;
                for (size_t c = 0; c <= CHART_CHANNEL_COUNT; ++c) {
                    if (chart_file(c) != kind || !header.channels[c].levels[0].count) continue;
                    for (size_t i = 0; i < FINE_BLOCK_CELLS; ++i) {
                        const int64_t begin = block + int64_t(i) * series.level_ms;
                        if (begin >= series.from_ms && begin < series.to_ms &&
                            !add(c, begin, cells.get()[at + i])) return false;
                    }
                    at += FINE_BLOCK_CELLS;
                }
            }
        }
        return true;
    }
    Cell cells[64];
    for (size_t c = 0; c <= CHART_CHANNEL_COUNT; ++c) {
        const auto &channel = header.channels[c];
        const auto &level = channel.levels[series.level_index];
        const uint32_t count = level.count;
        const int64_t start = level.start_ms;
        const uint64_t offset = level.offset;
        if (!count || channel.end_ms <= series.from_ms || start >= series.to_ms) continue;
        const uint32_t first = std::max<int64_t>(0, (series.from_ms - start) / series.level_ms);
        const uint32_t last = std::min<int64_t>(count, (series.to_ms - start) / series.level_ms);
        if (!reader.seek(offset + first * sizeof(Cell))) return false;
        for (uint32_t n = first; n < last;) {
            const size_t take = std::min<uint32_t>(64, last - n);
            if (aborted(&job, session) || reader.read(reinterpret_cast<uint8_t *>(cells),
                take * sizeof(Cell)) != take * sizeof(Cell)) return false;
            for (size_t i = 0; i < take; ++i) {
                const int64_t begin = start + (n + i) * series.level_ms;
                if (!add(c, begin, cells[i])) return false;
            }
            n += take;
        }
    }
    return true;
}

bool window_series(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, Series &series) {
    Input input(session, job);
    for (uint8_t kind = 0; kind < 3; ++kind) {
        if (!(entry.key.present & (1u << kind))) { series.partial = true; continue; }
        if (!input.open(session, entry, kind)) return false;
        int signals[CHART_CHANNEL_COUNT + 1];
        chart_signals(*input.header, kind, signals);
        const int64_t origin = input.header->start_seconds * 1000;
        const double record_ms = input.header->record_seconds * 1000;
        if (!input.scan_window(signals, CHART_CHANNEL_COUNT + 1, series.from_ms, series.to_ms,
            [&](size_t c, uint64_t base, uint64_t begin_sample, uint64_t end_sample,
                const ReportEdf::RecordSamples &samples) {
                const auto &spec = *samples.signal;
                const double step = record_ms / spec.samples_per_record;
                const double scale = chart_scale(c, spec);
                for (uint64_t n = begin_sample; n < end_sample; ++n) {
                    const auto sample = samples[n - base];
                    if (!sample.valid) continue;
                    int64_t begin = std::max<int64_t>(series.from_ms, origin + llround(n * step));
                    const int64_t end = std::min<int64_t>(series.to_ms, origin + llround((n + 1) * step));
                    while (begin < end) {
                        const int64_t cell = (begin - series.from_ms) / series.step_ms;
                        const int64_t right = std::min(end, series.from_ms + (cell + 1) * series.step_ms);
                        const float value = sample.value * scale;
                        series.add(c, begin, right, value, value, right - begin);
                        begin = right;
                    }
                }
                return true;
            })) return false;
        input.file.close();
    }
    return true;
}

bool cached_summary_header(const SdStorage::Session &session, const IndexedSession &entry,
                           CacheHeader &header) {
    char path[96];
    cache_path(entry, "stats", path);
    SdStorage::Reader reader;
    return reader.open(session, path) && reader.read(reinterpret_cast<uint8_t *>(&header),
        sizeof(header)) == sizeof(header) && sane_header(header, entry, reader.size());
}
bool build_summary_header(const SdStorage::Session &session, const Job &job,
                          const IndexedSession &entry, CacheHeader &header) {
    SessionData data;
    if (!session_data(session, job, entry, data)) return false;
    header = data.header;
    return true;
}

bool history_json(const SdStorage::Session &session, Job &job, bool build) {
    constexpr size_t DAYS = 14, METRICS = 6;
    constexpr size_t channels[] = {0, 3, 6, 4};
    constexpr const char *names[] = {"usage", "ahi", "pressure", "leak", "vt", "mv"};
    struct Work { DailyValues days[DAYS]; bool cached[DAYS] = {}; CacheHeader header; };
    void *memory = aircannect::Memory::alloc_large(sizeof(Work), sizeof(Work) <= INTERNAL_WORK);
    if (!memory) return false;
    std::unique_ptr<Work, decltype(&aircannect::Memory::free)> work(new (memory) Work, aircannect::Memory::free);
    const uint16_t last = last_day(job.request), first = last > DAYS - 1 ? last - DAYS + 1 : 0;
    bool complete = true;
    for (size_t d = 0; d < DAYS; ++d) {
        work->cached[d] = cached_day(first + d, work->days[d], build);
        complete &= work->days[d].ready;
    }
    for (size_t i = 0; i < index_count; ++i) {
        const auto &entry = index[i];
        if (!selected(entry, first, last) || entry.day - first >= DAYS) continue;
        if (work->cached[entry.day - first]) continue;
        if (aborted(&job, session)) return false;
        auto &day = work->days[entry.day - first];
        day.present = true;
        auto &header = work->header;
        if (!cached_summary_header(session, entry, header) &&
            (!build || !build_summary_header(session, job, entry, header))) {
            day.ready = complete = false;
            continue;
        }
        day.usage &= header.usage_known;
        day.event_count &= header.events_known || !header.recorded;
        day.seconds += header.used_seconds;
        for (size_t c = 0; c < 4; ++c) {
            day.events += header.counts[c];
            day.count[c] += header.channels[channels[c]].count;
            day.sum[c] += header.channels[channels[c]].sum;
        }
    }
    if (aborted(&job, session)) return false;
    for (size_t d = 0; d < DAYS; ++d)
        if (!work->cached[d]) remember_day(first + d, work->days[d]);
    job.body.print(",\"history\":{\"first_day\":%u,\"last_day\":%u,\"complete\":%s,\"metrics\":[",
                   first, last, complete ? "true" : "false");
    for (size_t m = 0; m < METRICS; ++m) {
        double sum = 0;
        unsigned count = 0;
        for (size_t d = DAYS - 8; d < DAYS - 1; ++d) {
            const double value = work->days[d].value(m);
            if (std::isfinite(value)) { sum += value; ++count; }
        }
        const double current = work->days[DAYS - 1].value(m);
        job.body.print("%s{\"id\":\"%s\",\"baseline_count\":%u,\"baseline\":", m ? "," : "", names[m], count);
        if (count) job.body.print("%.10g", sum / count); else job.body.print("null");
        job.body.print(",\"delta\":");
        if (count && std::isfinite(current)) job.body.print("%.10g", current - sum / count);
        else job.body.print("null");
        job.body.print(",\"points\":[");
        for (size_t d = 0; d < DAYS; ++d) {
            const double value = work->days[d].value(m);
            if (d) job.body.print(",");
            if (std::isfinite(value)) job.body.print("%.10g", value); else job.body.print("null");
        }
        job.body.print("]}");
    }
    job.body.print("]}");
    return job.body.good();
}

bool window_events(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, const CacheHeader &header,
                   int64_t from, int64_t to, Payload &out, bool &comma, bool &truncated) {
    if (truncated) return true;
    auto append = [&](const Event &event, const char *type = nullptr) {
        const bool intersects = type ?
            event.time_ms > from && event.time_ms - event.duration_ms < to :
            event.duration_ms > 0 ?
            event.time_ms >= from && event.time_ms - event.duration_ms < to :
            event.time_ms >= from && event.time_ms < to;
        if (event.kind == ReportEdf::EventKind::Unknown || !intersects)
            return;
        if (out.length() > MAX_BODY / 4 - 256) { truncated = true; return; }
        append_event(out, event, comma, type);
    };
    // Pair before viewport filtering, retaining only one pending start per
    // session. A repeated start leaves the preceding boundary unmatched.
    CsrPairing csr;
    auto consume = [&](const Event &event) {
        csr.consume(event, [&](const Event &value, bool paired) { append(value, paired ? "csr" : nullptr); });
    };
    char path[96];
    cache_path(entry, "stats", path);
    SdStorage::Reader reader;
    if (reader.open(session, path) && reader.size() == header.bytes &&
        reader.seek(sizeof(header) + total_bins(header) * sizeof(uint64_t))) {
        Event block[32];
        for (uint32_t n = 0; n < header.events;) {
            const size_t take = std::min<uint32_t>(32, header.events - n);
            if (aborted(&job, session) || reader.read(reinterpret_cast<uint8_t *>(block),
                take * sizeof(Event)) != take * sizeof(Event)) return false;
            for (size_t i = 0; i < take; ++i) consume(block[i]);
            if (truncated) return out.good();
            n += take;
        }
        if (csr.active) append(csr.start);
        return out.good();
    }
    reader.close();
    // Failed derived-cache persistence is not permission to truncate a late
    // viewport. Read the small annotation sources, never unrelated histograms.
    Input input(session, job);
    for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) return false;
        for (uint32_t i = 0; i < input.header->records; ++i) {
            Event event;
            if (!read_event(input, i, event)) return false;
            consume(event);
            if (truncated) return out.good();
        }
        input.file.close();
    }
    if (csr.active) append(csr.start);
    return out.good();
}

bool series_json(const SdStorage::Session &session, Job &job, JobTiming &timing) {
    uint32_t started = timing.stamp();
    // Inlining cache branches must not reserve their large headers on the raw
    // zoom stack, which also needs room for EDF parsing and libc formatting.
    struct Work { CacheHeader summary; ReducedHeader reduced; };
    void *memory = aircannect::Memory::alloc_large(sizeof(Work), sizeof(Work) <= INTERNAL_WORK);
    if (!memory) return note_issue(job, "series_work_alloc");
    std::unique_ptr<Work, decltype(&aircannect::Memory::free)> work(new (memory) Work, aircannect::Memory::free);
    Series series(job.request);
    timing.numeric_us += timing.stamp() - started;
    if (!series.buckets) return false;
    Payload events;
    bool event_comma = false, events_truncated = false;
    const uint16_t last = last_day(job.request), first = first_day(job.request, last);
    for (size_t i = 0; i < index_count; ++i) {
        auto &entry = index[i];
        if (job.request.excludes(i) || !selected(entry, first, last)) continue;
        const int64_t noon = int64_t(entry.day) * 86400000 + 43200000;
        if (noon + 86400000 <= series.from_ms || noon >= series.to_ms ||
            (entry.bounds_known && (entry.end_ms <= series.from_ms || entry.start_ms >= series.to_ms) &&
             (entry.event_to_ms < series.from_ms || entry.event_from_ms >= series.to_ms))) continue;
        if (aborted(&job, session)) return false;
        if (!(entry.key.present & 7)) { series.partial = true; continue; }
        started = timing.stamp();
        auto &header = work->summary;
        if (!cached_summary_header(session, entry, header) && !build_summary_header(session, job, entry, header)) {
            timing.numeric_us += timing.stamp() - started;
            if (aborted(&job, session)) return false;
            series.partial = true;
            continue;
        }
        series.partial |= header.partial;
        entry.bounds_known = header.chart_end_ms > header.chart_start_ms;
        entry.start_ms = header.chart_start_ms;
        entry.end_ms = header.chart_end_ms;
        entry.event_from_ms = header.event_from_ms;
        entry.event_to_ms = header.event_to_ms;
        // Non-overlapping sessions need neither BRP cache construction nor a
        // source window. Their summaries remain reusable for later requests.
        const bool numeric_overlap = !entry.bounds_known ||
            (entry.end_ms > series.from_ms && entry.start_ms < series.to_ms);
        const bool success = !numeric_overlap || (series.level_ms ? cached_series(session, job, entry, series, work->reduced)
                                                                  : window_series(session, job, entry, series));
        timing.numeric_us += timing.stamp() - started;
        if (!success) {
            if (aborted(&job, session)) return false;
            series.partial = true;
        }
        started = timing.stamp();
        if (!window_events(session, job, entry, header, series.from_ms, series.to_ms,
                           events, event_comma, events_truncated)) series.partial = true;
        timing.events_us += timing.stamp() - started;
    }
    started = timing.stamp();
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"excluded\":\"");
    size_t mask_bytes = sizeof(job.request.excluded);
    while (mask_bytes && !job.request.excluded[mask_bytes - 1]) --mask_bytes;
    for (size_t i = 0; i < mask_bytes; ++i) {
        const char pair[] = {
            aircannect::hex_digit(job.request.excluded[i] >> 4, aircannect::HexCase::Lower),
            aircannect::hex_digit(job.request.excluded[i], aircannect::HexCase::Lower)};
        job.body.append(pair, sizeof(pair));
    }
    job.body.print("\",\"day\":%u,\"from_ms\":%lld,\"to_ms\":%lld,\"bucket_ms\":%lld,\"px\":%u,",
                   last, static_cast<long long>(series.from_ms), static_cast<long long>(series.to_ms),
                   static_cast<long long>(series.step_ms), series.px);
    job.body.print("\"requested_from_ms\":%lld,\"requested_to_ms\":%lld,\"partial\":%s,\"channels\":[",
                   static_cast<long long>(job.request.from_ms), static_cast<long long>(job.request.to_ms),
                   series.partial ? "true" : "false");
    for (size_t c = 0; c <= CHART_CHANNEL_COUNT; ++c) {
        job.body.print("%s{\"id\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\",", c ? "," : "",
                       c ? CHANNELS[c - 1].id : "Flow", c ? CHANNELS[c - 1].label : "Flow",
                       c ? CHANNELS[c - 1].unit : "L/min");
        series_channel_json(job.body, series, c);
        job.body.print("}");
    }
    job.body.print("],\"events_truncated\":%s,\"events\":[", events_truncated ? "true" : "false");
    if (events.length()) job.body.append(events.c_str(), events.length());
    job.body.print("]}");
    timing.encode_us += timing.stamp() - started;
    return job.body.good() && events.good();
}

void error_body(Job &job, int code, const char *error) {
    job.body.~Payload();
    new (&job.body) Payload;
    job.code = code;
    job.error = error;
    job.body.print("{\"source\":\"edf\",\"error\":\"%s\"}", error);
}

bool dirty() {
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    return !index_ready || __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE) != indexed_invalidation ||
        catalog.generation != catalog_generation ||
        SdStorage::files_revision() != file_revision;
}

void defer_job(Job *job) {
    if (!job) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) reset_job(*job);
    else job->state = JobState::Queued;
    xSemaphoreGive(mutex);
}

bool work_once() {
    static const char *reported_index_error = nullptr;
    Job *job = nullptr;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto &candidate : jobs) {
        if (candidate.state != JobState::Free && reusable(candidate)) reset_job(candidate);
        if (candidate.state == JobState::Queued &&
            (!job || int32_t(candidate.id - job->id) < 0)) job = &candidate;
    }
    if (job) job->state = JobState::Running;
    xSemaphoreGive(mutex);
    if (!AirSenseState::local_background_allowed()) {
        defer_job(job); publish(State::Blocked, "not_idle"); return false;
    }
    if (!SdStorage::local_access_allowed()) { defer_job(job); publish(State::Blocked, "usb_storage"); return false; }
    if (!SdStorage::mounted()) { defer_job(job); publish(State::Blocked, "no_sd"); return false; }
    if (!job && !dirty()) { publish(State::Ready); return false; }
    SdStorage::Session session;
    if (!session.begin()) { defer_job(job); publish(State::WaitingStorage, "storage_busy"); return false; }
    JobTiming timing(job);
    publish(State::Building);
    const uint32_t index_started = timing.stamp();
    const bool indexed = refresh_index(session);
    timing.index_us = timing.stamp() - index_started;
    if (!indexed) {
        const bool preempted = !session.valid() || !AirSenseState::local_background_allowed();
        session.end();
        if (preempted || index_failure == IndexFailure::Preempted || index_failure == IndexFailure::WaitingCatalog) {
            defer_job(job);
            publish(preempted ? State::Blocked : State::WaitingStorage,
                    preempted ? "not_idle" : "catalog_not_ready");
        } else {
            Log::logf(CAT_REPORT, reported_index_error == index_error ? LOG_DEBUG : LOG_WARN,
                "EDF index failed: %s entry=%ld\n", index_error,
                index_error_entry == SIZE_MAX ? -1L : long(index_error_entry));
            reported_index_error = index_error;
            if (job) {
                xSemaphoreTake(mutex, portMAX_DELAY);
                job->data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
                error_body(*job, job->cancelled ? 410 : 503, job->cancelled ? "cancelled" : "index_unavailable");
                job->state = JobState::Complete;
                job->completed_ms = millis();
                const int code = job->code;
                const size_t bytes = job->body.length();
                xSemaphoreGive(mutex);
                portENTER_CRITICAL(&status_mux);
                ++status.revision;
                portEXIT_CRITICAL(&status_mux);
                timing.log(code, bytes);
            }
            publish(State::Error, "index_unavailable");
            return job != nullptr;
        }
        return false;
    }
    if (reported_index_error) {
        Log::logf(CAT_REPORT, LOG_INFO, "EDF index recovered\n");
        reported_index_error = nullptr;
    }
    if (!job) {
        session.end();
        publish(State::Ready);
        return false;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    job->state = JobState::Running;
    job->data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    job->code = 0;
    job->error = job->issue = nullptr;
    job->issue_path[0] = 0;
    xSemaphoreGive(mutex);
    bool success = false;
    if (__atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) error_body(*job, 410, "cancelled");
    else if (job->request.revision && job->request.revision != job->data_revision)
        error_body(*job, 409, "stale_revision");
    else {
        if (job->request.action == Action::History) {
            start_json(job->body, revision() + 1, job->data_revision);
            job->body.print("\"day\":%u", last_day(job->request));
            success = history_json(session, *job, true);
            job->body.print("}");
        } else success = job->request.action == Action::Days ? days_json(*job) :
            job->request.action == Action::Summary ? summary_json(session, *job) : series_json(session, *job, timing);
        if (success) job->code = 200;
    }
    const bool interrupted = !AirSenseState::local_background_allowed() || !session.valid();
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const bool changed = catalog.generation != catalog_generation || SdStorage::files_revision() != file_revision ||
        job->data_revision != __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    session.end();
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) error_body(*job, 410, "cancelled");
    else if (interrupted || changed) {
        job->body.~Payload();
        new (&job->body) Payload;
        job->state = JobState::Queued;
        job->code = 0;
        xSemaphoreGive(mutex);
        publish(interrupted ? State::Blocked : State::Building, interrupted ? "not_idle" : "");
        return false;
    } else if (!success && !job->code) error_body(*job, 503, "report_build_failed");
    job->state = JobState::Complete;
    job->completed_ms = millis();
    job->cache_generation = __atomic_load_n(&cache_generation, __ATOMIC_ACQUIRE);
    const int code = job->code;
    const size_t bytes = job->body.length();
    log_result(*job);
    xSemaphoreGive(mutex);
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    publish(State::Ready);
    timing.log(code, bytes);
    return true;
}

void worker(void *) {
    while (true) {
        if (work_once()) continue;
        Status publication;
        get_status(publication);
        xSemaphoreTake(wake, publication.state == State::WaitingStorage || publication.state == State::Blocked
            ? pdMS_TO_TICKS(1000) : portMAX_DELAY);
    }
}

}  // namespace

void init() {
    if (mutex) return;
    const auto memory = aircannect::Memory::status();
    if (memory.psram_available) {
        auto *storage = static_cast<Job *>(aircannect::Memory::alloc_large(RETAINED_SLOTS * sizeof(Job), false));
        if (storage) {
            for (size_t i = 0; i < RETAINED_SLOTS; ++i) new (&storage[i]) Job;
            jobs.data = storage;
            jobs.count = RETAINED_SLOTS;
        }
    }
    mutex = xSemaphoreCreateMutex();
    wake = xSemaphoreCreateBinary();
    portENTER_CRITICAL(&status_mux);
    status.supported = true;
    status.data_revision = data_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    if (!mutex || !wake) {
        Log::logf(CAT_REPORT, LOG_ERROR, "EDF worker synchronization allocation failed\n");
        publish(State::Error, "worker_unavailable");
        return;
    }
    aircannect::Memory::set_reclaimer(reclaim_result);
    BaseType_t created = pdFAIL;
    if (aircannect::Memory::psram_available())
        created = xTaskCreatePinnedToCoreWithCaps(worker, "edf_report", 8192, nullptr, 1,
            &worker_task, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (created != pdPASS)
        created = xTaskCreatePinnedToCoreWithCaps(worker, "edf_report", 8192, nullptr, 1,
            &worker_task, 0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (created != pdPASS) {
        Log::logf(CAT_REPORT, LOG_ERROR, "EDF worker task allocation failed\n");
        worker_task = nullptr;
        publish(State::Error, "worker_unavailable");
    }
    tick();
}

void tick() {
    static uint32_t observed_catalog = 0, observed_files = 0;
    const bool local = SdStorage::local_access_allowed();
    const bool mounted = SdStorage::mounted();
    const bool available = local && mounted && worker_task && AirSenseState::local_background_allowed();
    const uint32_t catalog = EdfCatalog::revision(), files = SdStorage::files_revision();
    bool changed = false, invalidated = false;
    portENTER_CRITICAL(&status_mux);
    // A deliberate USB unmount pauses reports, not a request to rebuild metadata.
    if (local && status.mounted != mounted) {
        status.mounted = mounted;
        invalidated = true;
        changed = true;
    }
    if (status.available != available) { status.available = available; changed = true; }
    if (observed_files != files) { observed_files = files; invalidated = true; changed = true; }
    if (invalidated) {
        status.data_revision = __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
        __atomic_add_fetch(&invalidation_generation, 1, __ATOMIC_RELEASE);
    }
    if (changed) ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    if (!local) publish(State::Blocked, "usb_storage");
    else if (!mounted) publish(State::Blocked, "no_sd");
    else if (!available && worker_task) publish(State::Blocked, "not_idle");
    if (wake && (changed || observed_catalog != catalog)) {
        observed_catalog = catalog;
        xSemaphoreGive(wake);
    }
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return;
    for (auto &job : jobs) if (job.state != JobState::Free && reusable(job)) reset_job(job);
    xSemaphoreGive(mutex);
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = status;
    portEXIT_CRITICAL(&status_mux);
}
uint32_t revision() {
    portENTER_CRITICAL(&status_mux);
    const uint32_t out = status.revision;
    portEXIT_CRITICAL(&status_mux);
    return out;
}

bool submit(const Request &request, uint32_t &id, const char **error) {
    auto reject = [&](const char *reason) { if (error) *error = reason; return false; };
    if (!valid(request)) return reject("request_invalid");
    Status publication;
    get_status(publication);
    if (!worker_task) return reject("worker_unavailable");
    if (!publication.mounted) return reject("no_sd");
    if (!publication.available) return reject("not_idle");
    if (request.revision && request.revision != publication.data_revision) return reject("stale_revision");
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return reject("busy");
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const bool current_index = catalog.ready &&
        catalog.generation == publication.catalog_generation &&
        SdStorage::files_revision() == publication.files_revision;
    Job *slot = nullptr, *delivered = nullptr;
    size_t active = 0;
    for (auto &job : jobs) {
        if (current_index && !job.cancelled && !job.issue && job.state == JobState::Complete && job.code == 200 &&
            job.data_revision == publication.data_revision && job.request == request &&
            ((request.action != Action::Summary && request.action != Action::History) ||
             job.cache_generation == __atomic_load_n(&cache_generation, __ATOMIC_ACQUIRE)) &&
            uint32_t(millis() - job.completed_ms) < RETAIN_MS) {
            if (job.consumers == UINT16_MAX) { xSemaphoreGive(mutex); return reject("busy"); }
            ++job.consumers;
            job.shared = true;
            job.completed_ms = millis();
            id = job.id;
            xSemaphoreGive(mutex);
            portENTER_CRITICAL(&status_mux);
            ++status.revision;
            portEXIT_CRITICAL(&status_mux);
            return true;
        }
        if (!job.cancelled && (job.state == JobState::Queued || job.state == JobState::Running) &&
            job.request == request) {
            if (job.consumers == UINT16_MAX) { xSemaphoreGive(mutex); return reject("busy"); }
            ++job.consumers;
            job.shared = true;
            id = job.id;
            xSemaphoreGive(mutex);
            return true;
        }
        if (job.state == JobState::Queued || job.state == JobState::Running) ++active;
        if (reusable(job)) slot = &job;
        if (job.state == JobState::Complete && !job.readers && job.data_revision != publication.data_revision)
            slot = &job;
        if (evictable(job) && (!delivered ||
            int32_t(job.completed_ms - delivered->completed_ms) < 0)) delivered = &job;
    }
    // Retention is opportunistic: routine days->summary->series and repeated
    // zooms can reclaim delivered/unpinned slots without a browser delay timer.
    if (!slot) slot = delivered;
    if (!slot || active >= SLOT_COUNT) { xSemaphoreGive(mutex); return reject("busy"); }
    reset_job(*slot);
    slot->request = request;
    slot->id = next_id++;
    if (!next_id) next_id = 1;
    slot->state = JobState::Queued;
    id = slot->id;
    xSemaphoreGive(mutex);
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    xSemaphoreGive(wake);
    return true;
}

void cancel(uint32_t id) {
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return;
    for (auto &job : jobs) {
        if (job.id != id || job.shared || job.readers) continue;
        __atomic_store_n(&job.cancelled, true, __ATOMIC_RELEASE);
        if (job.state == JobState::Queued || job.state == JobState::Complete) reset_job(job);
        break;
    }
    xSemaphoreGive(mutex);
    if (wake) xSemaphoreGive(wake);
}

int poll(uint32_t id, Result &out) {
    out.reset();
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return 503;
    int code = 410;
    for (auto &job : jobs) {
        if (job.id != id || job.state == JobState::Free) continue;
        if (job.state != JobState::Complete) { code = 202; break; }
        Status publication;
        get_status(publication);
        if (job.code == 200 && (!SdStorage::local_access_allowed() || !SdStorage::mounted() || !publication.mounted ||
            job.data_revision != publication.data_revision ||
            SdStorage::files_revision() != publication.files_revision)) { code = 409; break; }
        EdfCatalog::Status catalog;
        EdfCatalog::get_status(catalog);
        if (job.code == 200 && catalog.generation != publication.catalog_generation) { code = 202; break; }
        if (!job.body.length() || job.readers == UINT16_MAX) { code = 503; break; }
        code = job.code;
        ++job.readers;
        if (job.deliveries < job.consumers) ++job.deliveries;
        job.delivered = true;
        out.id_ = job.id;
        out.data_ = job.body.c_str();
        out.length_ = job.body.length();
        break;
    }
    xSemaphoreGive(mutex);
    return code;
}

void Result::reset() {
    if (!id_) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto &job : jobs) if (job.id == id_) {
        if (job.readers) --job.readers;
        if (reusable(job)) reset_job(job);
        break;
    }
    xSemaphoreGive(mutex);
    id_ = 0;
    data_ = nullptr;
    length_ = 0;
}

}  // namespace EdfReport
#else
namespace EdfReport {
void init() {}
void tick() {}
void get_status(Status &out) {
    out = {};
    snprintf(out.error, sizeof(out.error), "%s", "unsupported");
}
uint32_t revision() { return 0; }
bool submit(const Request &, uint32_t &, const char **error) {
    if (error) *error = "unsupported";
    return false;
}
void cancel(uint32_t) {}
int poll(uint32_t, Result &out) { out.reset(); return 422; }
void Result::reset() { id_ = 0; data_ = nullptr; length_ = 0; }
}  // namespace EdfReport
#endif
