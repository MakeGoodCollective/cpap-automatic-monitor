#include "edf_report.h"
#include "board.h"
#include "sd_storage.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <utility>

namespace EdfReport {

bool Request::operator==(const Request &other) const {
    return action == other.action && view == other.view && day == other.day &&
        period == other.period && revision == other.revision;
}

bool valid(const Request &request) {
    if (request.action > Action::Reindex || uint8_t(request.action) == 2 || request.view > View::Period ||
        request.day == UINT16_MAX) return false;
    if (request.action == Action::History && request.view != View::Day) return false;
    if (request.action == Action::Reindex && (request.view != View::Day || request.revision)) return false;
    if (request.view == View::Period && request.period != 1 && request.period != 7 &&
        request.period != 30 && request.period != 90 && request.period != 180 &&
        request.period != 365) return false;
    return true;
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
#include "export_sync.h"
#include "debug_log.h"
#include "memory_manager.h"
#include "large_text_buffer.h"
#include "hex_util.h"
#include "report_edf.h"
#include "report_artifact_builder.h"
#include "settings_defs.h"
#include <Arduino.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <freertos/semphr.h>
#include <cstdarg>
#include <cstdio>
#include <new>

namespace EdfReport {
namespace {
constexpr uint32_t STATS_CACHE_VERSION = 7;
constexpr size_t MAX_BINS = 16384;
constexpr size_t MAX_EVENTS = 4096;
constexpr size_t MAX_DISK_EVENTS = 65536;
constexpr size_t MAX_BODY = 512 * 1024;
constexpr size_t INTERNAL_BODY = 16 * 1024;
constexpr size_t INTERNAL_WORK = 8 * 1024;
constexpr uint32_t RETAIN_MS = 6 * 60 * 60 * 1000;
constexpr uint32_t DELIVERY_MS = 30000;
constexpr size_t SLOT_COUNT = 2;
constexpr size_t RETAINED_SLOTS = 32;
constexpr double LEAK_THRESHOLD = 24;
constexpr const char *CACHE_DIR = "/airbridge/report";
constexpr const char *SUFFIXES[] = {"BRP", "PLD", "SAD", "EVE", "CSL"};
char dataset_id[65] = {};
uint32_t rebuild_generation = 0;

struct ChannelSpec {
    const char *id, *label, *unit, *signal;
    uint8_t file;
    ReportArtifact::SignalId artifact_id;
};
using ReportArtifact::SignalId;
constexpr ChannelSpec CHANNELS[] = {
    {"Press", "Pressure", "cmH2O", "Press.2s", 1, SignalId::Press},
    {"EprPress", "Expiratory pressure", "cmH2O", "EprPress.2s", 1, SignalId::EprPress},
    {"MaskPress", "Mask pressure", "cmH2O", "MaskPress.2s", 1, SignalId::MaskPress},
    {"Leak", "Leak", "L/min", "Leak.2s", 1, SignalId::Leak},
    {"MinVent", "Minute ventilation", "L/min", "MinVent.2s", 1, SignalId::MinVent},
    {"RespRate", "Respiratory rate", "bpm", "RespRate.2s", 1, SignalId::RespRate},
    {"TidVol", "Tidal volume", "mL", "TidVol.2s", 1, SignalId::TidVol},
    {"SpO2", "SpO2", "%", "SpO2.1s", 2, SignalId::SpO2},
    {"Pulse", "Pulse", "bpm", "Pulse.1s", 2, SignalId::Pulse},
    {"FlowLim", "Flow limitation", "", "FlowLim.2s", 1, SignalId::FlowLim},
    {"Snore", "Snore", "", "Snore.2s", 1, SignalId::Snore},
    {"TgtVent", "Target ventilation", "L/min", "TgtVent.2s", 1, SignalId::TgtVent},
    {"Ti", "Inspiratory time", "s", "Ti.2s", 1, SignalId::Ti},
    {"IERatio", "I:E", "%", "IERatio.2s", 1, SignalId::IERatio},
    {"AlvMinVent", "Alveolar ventilation", "L/min", "AlvMinVent.2s", 1, SignalId::AlvMinVent},
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
    // Summaries and history nest during preparation; keep their headers off
    // the worker stack along with the separately allocated samples/events.
    static void *operator new(size_t size, const std::nothrow_t &) noexcept {
        return aircannect::Memory::alloc_large(size, size <= INTERNAL_WORK);
    }
    static void operator delete(void *memory) noexcept { aircannect::Memory::free(memory); }
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
    bool has_values = false, artifacts_checked = false;
    uint8_t artifact_key[32] = {};
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
CachedDay *cached_day_entry(uint16_t day, bool create) {
    if (!day) return nullptr;
    size_t slot = daily_capacity;
    for (size_t i = 0; i < daily_capacity; ++i) {
        if (daily_cache[i].day == day) {
            daily_cache[i].updated = ++daily_update;
            return &daily_cache[i];
        }
        if (!daily_cache[i].day && slot == daily_capacity) slot = i;
    }
    if (!create) return nullptr;
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
    if (!daily_capacity) return nullptr; // Cache allocation failure does not fail a report.
    if (slot == daily_capacity) {
        slot = 0;
        for (size_t i = 1; i < daily_capacity; ++i)
            if (int32_t(daily_cache[i].updated - daily_cache[slot].updated) < 0) slot = i;
    }
    daily_cache[slot] = {};
    daily_cache[slot].day = day;
    daily_cache[slot].updated = ++daily_update;
    return &daily_cache[slot];
}
bool cached_day(uint16_t day, DailyValues &out, bool build) {
    const auto *cached = cached_day_entry(day, false);
    if (!cached || !cached->has_values || (build && !cached->values.ready)) return false;
    out = cached->values;
    return true;
}
void remember_day(uint16_t day, const DailyValues &values) {
    if (auto *cached = cached_day_entry(day, true)) {
        cached->values = values;
        cached->has_values = true;
    }
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
    uint32_t artifact_revision = 0;
    uint16_t readers = 0;
    uint16_t consumers = 1, deliveries = 0;
    bool delivered = false, cancelled = false, shared = false;
    bool background = false;
    mutable bool artifact_preempted = false;
    uint32_t foreground_epoch = 0;
    int code = 0;
    const char *error = nullptr;
    mutable const char *issue = nullptr;
    mutable char issue_path[96] = {};
    mutable uint64_t source_bytes = 0;
    mutable uint32_t source_reads = 0, source_us = 0, tiles_us = 0;
    bool diagnostics = false;
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
        job.request.action == Action::Summary ? "summary" : job.request.action == Action::History ? "history" :
        job.request.action == Action::Prepare ? "prepare" : "reindex",
        unsigned(job.request.day), job.code, reason ? reason : "complete",
        expected || !job.issue_path[0] ? "-" : job.issue_path);
}

struct JobTiming {
    bool enabled;
    uint32_t id, started_us;
    Action action;
    uint32_t index_us = 0;
    const Job *job;

    explicit JobTiming(Job *job)
        : enabled(job && Log::get_cat_level(CAT_REPORT) >= LOG_DEBUG),
          id(job ? job->id : 0), started_us(stamp()),
          action(job ? job->request.action : Action::Days), job(job) {
        if (job) {
            job->diagnostics = enabled;
            job->source_bytes = 0;
            job->source_reads = job->source_us = job->tiles_us = 0;
        }
    }

    uint32_t stamp() const { return enabled ? micros() : 0; }

    void log(int code, size_t bytes) const {
        if (!enabled) return;
        Log::logf(CAT_REPORT, LOG_DEBUG,
            "report job=%lu action=%s code=%d bytes=%u ms total=%lu "
            "idx=%lu src=%lu tiles=%lu\n",
            static_cast<unsigned long>(id), action == Action::Days ? "days" :
                action == Action::Summary ? "summary" : action == Action::History ? "history" :
                action == Action::Prepare ? "prepare" : "reindex", code,
            static_cast<unsigned>(bytes), static_cast<unsigned long>((stamp() - started_us) / 1000),
            static_cast<unsigned long>(index_us / 1000), static_cast<unsigned long>(job->source_us / 1000),
            static_cast<unsigned long>(job->tiles_us / 1000));
        if (job->source_reads) Log::logf(CAT_REPORT, LOG_DEBUG,
            "report job=%lu source reads=%lu bytes=%llu\n", static_cast<unsigned long>(id),
            static_cast<unsigned long>(job->source_reads), static_cast<unsigned long long>(job->source_bytes));
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
uint32_t foreground_epoch = 0;
uint32_t artifact_reads = 0;
bool index_deleting = false;
uint32_t backfill_revision = 0;
uint16_t backfill_before = UINT16_MAX;
uint8_t backfill_period = 0;
constexpr uint16_t PERIODS[] = {1, 7, 30, 90, 180, 365};
uint32_t published_index_data = 0, published_index_artifacts = UINT32_MAX;
bool index_commit_pending = false;
ReportArtifact::Document retired_index;
uint32_t retired_index_files = 0;

struct CachedPublication {
    ReportArtifact::Document document;
    uint16_t day = 0, period = 0;
    ReportArtifact::Lookup state = ReportArtifact::Lookup::Failed;
};
CachedPublication *publications = nullptr;
size_t publication_count = 0, publication_capacity = 0;
uint32_t publication_files_revision = 0;

uint32_t background_retry = 0;
bool backfill_enabled = true;
IndexedSession *index = nullptr;
size_t index_count = 0;
uint32_t catalog_generation = 0, file_revision = 0;
uint32_t invalidation_generation = 0, indexed_invalidation = 0;
bool index_ready = false;
struct IndexScan {
    bool active = false, full = false;
    bool detail_changed = false, summary_changed = false;
    uint32_t catalog = 0, files = 0, invalidation = 0;
    uint32_t started_ms = 0, resumes = 0;
    size_t next = 0;
} index_scan;
bool cache_usage_known = false;
uint64_t cache_bytes = 0;
uint32_t cache_generation = 0;
enum class IndexFailure : uint8_t { Error, WaitingCatalog, Preempted };
IndexFailure index_failure = IndexFailure::Error;
const char *index_error = nullptr;
size_t index_error_entry = SIZE_MAX;
bool migrate_cache(const SdStorage::Session &session);
bool artifact_headroom(const SdStorage::Session &session, uint16_t day);
void invalidate_str();
bool dataset(const SdStorage::Session &session);

void invalidate_publications() {
    aircannect::Memory::free(publications);
    publications = nullptr;
    publication_count = publication_capacity = 0;
    publication_files_revision = SdStorage::files_revision();
}

CachedPublication *publication_entry(uint16_t day, uint16_t period) {
    if (publication_files_revision != SdStorage::files_revision()) invalidate_publications();
    // Descending days followed by periods: normal cold discovery appends, and
    // index serialization finds retained metadata without another linear scan.
    const uint32_t key = period ? 65536u + period : 65535u - day;
    size_t slot = 0, end = publication_count;
    while (slot < end) {
        const size_t mid = slot + (end - slot) / 2;
        const auto &entry = publications[mid];
        const uint32_t candidate = entry.period ? 65536u + entry.period : 65535u - entry.day;
        if (candidate < key) slot = mid + 1;
        else end = mid;
    }
    if (slot < publication_count && publications[slot].period == period &&
        (period || publications[slot].day == day)) {
        auto &entry = publications[slot];
        // Only the selected end date is retained for each of the six periods.
        if (entry.day != day) entry = {};
        entry.day = day;
        entry.period = period;
        return &entry;
    }
    if (publication_count == publication_capacity) {
        const size_t capacity = std::min(index_count + sizeof(PERIODS) / sizeof(PERIODS[0]),
            std::max(size_t(16), publication_capacity * 2));
        if (capacity <= publication_capacity) return nullptr;
        const size_t bytes = capacity * sizeof(CachedPublication);
        auto *grown = static_cast<CachedPublication *>(aircannect::Memory::realloc_large(
            publications, bytes, bytes <= INTERNAL_WORK));
        if (!grown) return nullptr;
        publications = grown;
        publication_capacity = capacity;
    }
    memmove(publications + slot + 1, publications + slot,
        (publication_count - slot) * sizeof(CachedPublication));
    auto *entry = new (&publications[slot]) CachedPublication;
    ++publication_count;
    entry->day = day;
    entry->period = period;
    return entry;
}

void remember_publication(uint16_t day, uint16_t period, const ReportArtifact::Document &document) {
    if (auto *entry = publication_entry(day, period)) {
        entry->document = document;
        entry->state = ReportArtifact::Lookup::Ready;
    }
}

ReportArtifact::Lookup lookup_publication(const SdStorage::Session &session,
    uint16_t day, uint16_t period, const char *prefix, ReportArtifact::Document &document) {
    if (!session.valid()) return ReportArtifact::Lookup::Failed;
    auto *entry = publication_entry(day, period);
    if (entry && entry->state != ReportArtifact::Lookup::Failed) {
        document = entry->document;
        return entry->state;
    }
    const auto state = ReportArtifact::document_lookup(session, prefix, nullptr, document);
    if (state == ReportArtifact::Lookup::Ready) {
        Payload verified;
        // Validate old publications once, including fallback to an older slot.
        // A failed read may be transient; never memoize it as missing.
        if (!ReportArtifact::document_read(session, prefix, document, verified, nullptr))
            return ReportArtifact::Lookup::Failed;
    }
    if (entry && session.valid()) {
        entry->document = document;
        entry->state = state;
    }
    return state;
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
    // Owner-only work batches: SD operations remain individually bounded, and
    // CPU codec/histogram loops also yield without a delay per sample or tile.
    static uint32_t batch_us = 0;
    const uint32_t now = micros();
    if (uint32_t(now - batch_us) >= 4000) { vTaskDelay(1); batch_us = micros(); }
    if (__atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE) || SdStorage::maintenance_pending()) {
        if (job) job->artifact_preempted = true;
        return true;
    }
    return (job && __atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) ||
        (job && job->background && job->foreground_epoch != __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE)) ||
        !AirSenseState::local_background_allowed() || !session.valid();
}

void source_path(const IndexedSession &entry, uint8_t file, const char *ext,
                 char (&out)[96]) {
    snprintf(out, sizeof(out), "/DATALOG/%s/%s_%s.%s", entry.entry.therapy_day,
             entry.entry.file_prefix, SUFFIXES[file], ext);
}
void cache_path(const IndexedSession &entry, const char *ext, char (&out)[96]) {
    if (rebuild_generation) snprintf(out, sizeof(out), "%s/%s/%s.g%lu.%s", CACHE_DIR,
        entry.entry.therapy_day, entry.entry.file_prefix, static_cast<unsigned long>(rebuild_generation), ext);
    else snprintf(out, sizeof(out), "%s/%s/%s.%s", CACHE_DIR,
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
    const bool resume = index_scan.active && index_scan.catalog == changes.catalog.generation &&
        index_scan.files == files && index_scan.invalidation == invalidation &&
        changes.catalog.entries == index_count;
    if (!resume) {
        // A changed source invalidates the partially updated in-memory index.
        // Mere SD/therapy preemption retains both the cursor and change flags.
        const bool full = index_scan.active || !index_ready || invalidated || !incremental ||
            files != file_revision || changes.catalog.entries < index_count;
        if (full) { invalidate_day(); invalidate_str(); dataset_id[0] = 0; }
        if (!dataset(session)) { index_error = "dataset_identity"; return false; }
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
        index_scan = {true, full, !index_ready || changes.catalog.entries < old_count, false,
                      changes.catalog.generation, files, invalidation, millis(), 0, 0};
        index_ready = false;
    } else ++index_scan.resumes;
    const size_t count = index_scan.full ? index_count : changes.count;
    for (; index_scan.next < count; ++index_scan.next) {
        if (aborted(nullptr, session)) {
            index_failure = IndexFailure::Preempted;
            return false;
        }
        const size_t i = index_scan.full ? index_scan.next : changes.indexes[index_scan.next];
        index_error_entry = i;
        index_error = "catalog_index";
        if (i >= index_count) return false;
        IndexedSession next;
        index_error = "catalog_read";
        if (!session.run([&](fs::FS &) { return EdfCatalog::read(i, next.entry); })) return false;
        index_error = "catalog_day";
        if (!catalog_day(next.entry.therapy_day, next.day)) return false;
        // STR/Identification readiness is not a detail-data revision.
        index_scan.summary_changed |= index[i].entry.str_revision != next.entry.str_revision ||
            ((index[i].entry.flags ^ next.entry.flags) & EdfCatalog::ENTRY_STR_READY);
        const bool changed = strcmp(index[i].entry.file_prefix, next.entry.file_prefix) ||
            strcmp(index[i].entry.therapy_day, next.entry.therapy_day) ||
            ((index[i].entry.flags ^ next.entry.flags) & EdfCatalog::ENTRY_LIVE_COMPLETE);
        if (index_scan.full || changed) {
            index_error = "source_fingerprint";
            if (!fingerprint(session, next)) return false;
            const bool key_changed = !(index[i].key == next.key);
            if (changed || key_changed) {
                if (index[i].day) invalidate_day(index[i].day);
                invalidate_day(next.day);
            }
            // A changed fingerprint is not proof that the sources can rebuild
            // the old cache. Its header rejects mismatches; retain the bytes
            // until a validated replacement is available.
            index_scan.detail_changed |= changed || key_changed;
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
    if (index_scan.full) {
        cache_usage_known = false;
        if (!migrate_cache(session)) {
            index_error = "cache_migrate";
            if (aborted(nullptr, session)) index_failure = IndexFailure::Preempted;
            return false;
        }
    }
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (aborted(nullptr, session) || !current.ready || current.generation != index_scan.catalog ||
        SdStorage::files_revision() != index_scan.files ||
        __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE) != index_scan.invalidation) {
        index_failure = IndexFailure::Preempted;
        return false;
    }
    const bool detail_changed = index_scan.detail_changed, summary_changed = index_scan.summary_changed;
    if (summary_changed) invalidate_str();
    catalog_generation = changes.catalog.generation;
    file_revision = files;
    indexed_invalidation = invalidation;
    index_ready = true;
    if ((detail_changed || summary_changed) && !invalidated) __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
    portENTER_CRITICAL(&status_mux);
    status.catalog_generation = catalog_generation;
    status.files_revision = file_revision;
    status.sessions = index_count;
    status.data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (detail_changed || summary_changed || invalidated) ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_REPORT, LOG_DEBUG, "EDF index ready entries=%u checked=%u resumes=%lu elapsed=%lums\n",
        unsigned(index_count), unsigned(count), static_cast<unsigned long>(index_scan.resumes),
        static_cast<unsigned long>(millis() - index_scan.started_ms));
    index_scan = {};
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
    ReportArtifact::Producer *artifact = nullptr;
    Input(const SdStorage::Session &session, const Job &job) : session(session), job(job) {
        void *memory = aircannect::Memory::alloc_large(sizeof(ReportEdf::Header),
                                                      sizeof(ReportEdf::Header) <= INTERNAL_WORK);
        if (memory) header = new (memory) ReportEdf::Header;
    }
    ~Input() {
        aircannect::Memory::free(header);
        aircannect::Memory::free(record);
    }
    size_t read_source(uint8_t *out, size_t bytes) {
        const uint32_t started = job.diagnostics ? micros() : 0;
        const size_t count = file.read(out, bytes);
        if (job.diagnostics) {
            job.source_us += uint32_t(micros() - started);
            job.source_bytes += count;
            ++job.source_reads;
        }
        return count;
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
                if (input.read_source(out + done, take) != take)
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
            if (read_source(record + at, take) != take) return note_issue(job, "source_read", path);
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
            if (artifact) {
                const uint32_t started = job.diagnostics ? micros() : 0;
                if (!artifact->record(*header, record, header->record_bytes)) {
                    note_issue(job, artifact->error(), path);
                    artifact = nullptr;
                }
                if (job.diagnostics) job.tiles_us += uint32_t(micros() - started);
            }
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

};

double chart_scale(size_t c, const ReportEdf::Signal &signal);

bool artifact_identity(const IndexedSession &entry, uint8_t kind, char (&prefix)[96], uint8_t (&key)[32]) {
    snprintf(prefix, sizeof(prefix), "%s/%s/%s_%s", CACHE_DIR, entry.entry.therapy_day,
             entry.entry.file_prefix, SUFFIXES[kind]);
    if (rebuild_generation) snprintf(prefix + strlen(prefix), sizeof(prefix) - strlen(prefix),
        ".g%lu", static_cast<unsigned long>(rebuild_generation));
    uint8_t identity[20] = {};
    SdStorage::put_le16(identity, ReportArtifact::VERSION);
    identity[2] = kind;
    identity[3] = (entry.key.checksums >> kind) & 1;
    SdStorage::put_le32(identity + 4, uint32_t(entry.key.sizes[kind]));
    SdStorage::put_le32(identity + 8, uint32_t(entry.key.sizes[kind] >> 32));
    memcpy(identity + 12, entry.key.sidecars[kind], 8);
    return ReportArtifact::fingerprint(identity, sizeof(identity), key);
}

struct ArtifactBuild {
    Input &input;
    std::unique_ptr<ReportArtifact::Producer> producer;

    ArtifactBuild(Input &input, const IndexedSession &entry, uint8_t kind, bool enabled) : input(input) {
        if (!enabled) return;
        char prefix[96];
        uint8_t key[32];
        if (!artifact_identity(entry, kind, prefix, key)) { note_issue(input.job, "artifact_digest"); return; }
        ReportArtifact::Publication existing;
        const auto lookup = ReportArtifact::lookup(input.session, prefix, key, existing);
        if (lookup == ReportArtifact::Lookup::Ready) return;
        if (lookup != ReportArtifact::Lookup::Missing) {
            note_issue(input.job, lookup == ReportArtifact::Lookup::Invalid ? "artifact_invalid" : "artifact_lookup", prefix);
            return;
        }
        ReportArtifact::Selection selection[ReportArtifact::MAX_TRACKS];
        size_t count = 0;
        for (size_t c = 0; c <= CHANNEL_COUNT; ++c) {
            if ((c ? CHANNELS[c - 1].file : 0) != kind) continue;
            const int signal = ReportEdf::find_signal(*input.header, c ? CHANNELS[c - 1].signal : "Flow.40ms");
            if (signal < 0) continue;
            selection[count++] = {uint16_t(c ? CHANNELS[c - 1].artifact_id : SignalId::Flow),
                uint16_t(signal), chart_scale(c, input.header->signals[signal]),
                c ? CHANNELS[c - 1].id : "Flow", c ? CHANNELS[c - 1].unit : "L/min"};
        }
        if (!count) return;
        producer.reset(new (std::nothrow) ReportArtifact::Producer(input.session, [](void *context) {
            const auto &input = *static_cast<Input *>(context);
            return aborted(&input.job, input.session);
        }, &input));
        if (!producer || !producer->begin(prefix, key, *input.header, selection, count)) {
            note_issue(input.job, producer ? producer->error() : "artifact_alloc", prefix);
            producer.reset();
            return;
        }
        input.artifact = producer.get();
    }
    ~ArtifactBuild() { input.artifact = nullptr; }
    bool needed() const { return producer != nullptr; }
    void finish() {
        input.artifact = nullptr;
        ReportArtifact::Publication publication;
        const uint32_t started = input.job.diagnostics ? micros() : 0;
        if (producer && !producer->finish(publication) && !aborted(&input.job, input.session))
            note_issue(input.job, producer->error(), input.path);
        if (input.job.diagnostics) input.job.tiles_us += uint32_t(micros() - started);
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
    if (!artifact_headroom(session, entry.day)) return false;
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
                   const IndexedSession &entry, SessionData &out, bool artifacts = false) {
    out.header.key = entry.key;
    out.header.day = entry.day;
    out.header.partial = entry.key.present != 31 || entry.key.checksums != 31;
    Input input(session, job);
    uint8_t readable_numeric = 0;
    // Gather descriptors before allocating the bounded exact histograms.
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
    for (uint8_t kind : {uint8_t(0), uint8_t(1), uint8_t(2)}) {
        if (!(readable_numeric & (1u << kind))) continue;
        if (!kind && !artifacts) continue;
        if (!input.open(session, entry, kind)) return false;
        ArtifactBuild artifact(input, entry, kind, artifacts);
        if (kind) {
            if (!scan_channels(input, kind, out)) return false;
        } else if (artifact.needed()) {
            const int flow = ReportEdf::find_signal(*input.header, "Flow.40ms");
            if (!input.scan_records(&flow, 1, 0, input.header->records,
                [](size_t, uint64_t, const ReportEdf::RecordSamples &) { return true; })) return false;
        }
        input.file.close();
        artifact.finish();
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
                  const IndexedSession &entry, SessionData &out, bool artifacts = false) {
    if (!artifacts && load_cache(session, entry, out)) {
        if (out.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
        return true;
    }
    out.~SessionData();
    ::new (&out) SessionData;
    if (!build_session(session, job, entry, out, artifacts))
        return note_issue(job, "session_build", entry.entry.file_prefix);
    if (out.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
    // The cache is derived and replaceable; failed persistence does not discard
    // useful parsed data or modify any source EDF/sidecar.
    if (!out.header.partial && !store_cache(session, job, entry, out) && !aborted(&job, session))
        note_issue(job, "cache_write", entry.entry.file_prefix);
    return !aborted(&job, session);
}

uint16_t last_day(const Request &request) {
    if (request.day) return request.day;
    uint16_t day = 0;
    for (size_t i = 0; i < index_count; ++i)
        if (index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE)
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

bool summary_json(const SdStorage::Session &session, Job &job);
bool history_json(const SdStorage::Session &session, Job &job, bool build);
bool window_events(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, const CacheHeader &header,
                   int64_t from, int64_t to, Payload &out, bool &comma, bool &truncated);

bool dataset(const SdStorage::Session &session) {
    if (*dataset_id) return true;
    constexpr const char *prefix = "/airbridge/report/dataset";
    ReportArtifact::Document document;
    Payload bytes;
    const auto found = ReportArtifact::document_lookup(session, prefix, nullptr, document);
    if (found == ReportArtifact::Lookup::Missing) {
        uint8_t id[32], key[32] = {};
        esp_fill_random(id, sizeof(id));
        if (!ReportArtifact::document_publish(session, prefix, key, id, sizeof(id), document)) return false;
    } else if (found != ReportArtifact::Lookup::Ready) return false;
    if (!ReportArtifact::document_read(session, prefix, document, bytes, nullptr) || bytes.length() != 32) return false;
    ReportArtifact::Document rebuild;
    const auto generation = ReportArtifact::document_lookup(session, "/airbridge/report/rebuild", nullptr, rebuild);
    rebuild_generation = 0;
    if (generation == ReportArtifact::Lookup::Ready) {
        Payload value;
        if (!ReportArtifact::document_read(session, "/airbridge/report/rebuild", rebuild, value, nullptr) || value.length() != 4) return false;
        rebuild_generation = SdStorage::get_le32(reinterpret_cast<const uint8_t *>(value.c_str()));
    } else if (generation != ReportArtifact::Lookup::Missing) return false;
    return aircannect::hex_encode(reinterpret_cast<const uint8_t *>(bytes.c_str()), 32,
        dataset_id, sizeof(dataset_id), aircannect::HexCase::Lower);
}

void publication_prefix(uint16_t day, char (&path)[96], uint16_t period = 0) {
    const time_t civil = int64_t(day) * 86400;
    tm date = {};
    gmtime_r(&civil, &date);
    char directory[9];
    strftime(directory, sizeof(directory), "%Y%m%d", &date);
    if (period) snprintf(path, sizeof(path), "%s/%s/period-%u", CACHE_DIR, directory, unsigned(period));
    else snprintf(path, sizeof(path), "%s/%s/day", CACHE_DIR, directory);
}

void period_prefix(const Request &request, char (&path)[96]) {
    publication_prefix(last_day(request), path, request.period);
}

bool publication_key(const Request &request, uint8_t (&key)[32]) {
    Payload identity;
    const uint16_t last = last_day(request);
    const uint16_t first = request.view == View::Day ? (last > 13 ? last - 13 : 0) : first_day(request, last);
    identity.print("publication-1:%s:%u:%u:%u:%lu", dataset_id, first, last, request.period,
                   static_cast<unsigned long>(rebuild_generation));
    for (size_t i = 0; i < index_count; ++i) {
        const auto &entry = index[i];
        if (!selected(entry, first, last)) continue;
        identity.print("/%u:%s:%s:%u:%u:%u:%u", unsigned(i), entry.entry.therapy_day,
            entry.entry.file_prefix, entry.entry.flags, entry.entry.str_revision,
            entry.key.present, entry.key.checksums);
        for (size_t k = 0; k < 5; ++k) {
            identity.print(":%llu:", static_cast<unsigned long long>(entry.key.sizes[k]));
            identity.append(reinterpret_cast<const char *>(entry.key.sidecars[k]), 8);
        }
    }
    return identity.good() && ReportArtifact::fingerprint(
        reinterpret_cast<const uint8_t *>(identity.c_str()), identity.length(), key);
}

bool artifact_headroom(const SdStorage::Session &session, uint16_t day) {
    SdStorage::Status storage;
    if (!session.run([](fs::FS &) { SdStorage::refresh_usage(); return true; })) return false;
    SdStorage::get_status(storage);
    uint64_t needed = 128 * 1024;
    for (size_t i = 0; i < index_count; ++i) if (selected(index[i], day, day))
        for (size_t kind = 0; kind < 3; ++kind) needed += index[i].key.sizes[kind] * 3;
    // Preserve room for another complete 24h numeric recording in the largest
    // supported schema, plus STR. Artifacts are optional, source EDF is not.
    const uint64_t reserve = 1440ull * (Air10Edf::record_size(Air10Edf::brp_schema(true)) +
        Air10Edf::record_size(Air10Edf::pld_schema()) + Air10Edf::record_size(Air10Edf::sad_schema())) +
        366ull * Air10Edf::record_size(Air10Edf::str_schema()) + 65536;
    return storage.total_bytes > storage.used_bytes && storage.total_bytes - storage.used_bytes >= needed + reserve;
}

bool prepare_period(const SdStorage::Session &session, Job &job) {
    if (!dataset(session)) return note_issue(job, "dataset_identity");
    char prefix[96]; uint8_t key[32];
    period_prefix(job.request, prefix);
    if (!publication_key(job.request, key)) return note_issue(job, "publication_key");
    ReportArtifact::Document document;
    if (ReportArtifact::document_lookup(session, prefix, key, document) == ReportArtifact::Lookup::Ready)
        return ReportArtifact::document_read(session, prefix, document, job.body, key);
    if (!summary_json(session, job) || job.issue || aborted(&job, session)) return false;
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (current.generation != catalog_generation || SdStorage::files_revision() != file_revision) return false;
    if (!ReportArtifact::document_publish(session, prefix, key,
        reinterpret_cast<const uint8_t *>(job.body.c_str()), job.body.length(), document))
        return note_issue(job, "publication_write", prefix);
    remember_publication(last_day(job.request), job.request.period, document);
    portENTER_CRITICAL(&status_mux);
    ++status.artifact_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

bool publish_histograms(const SdStorage::Session &session, const IndexedSession &entry,
                        const SessionData &data, ReportArtifact::Document &document) {
    Payload bytes;
    uint8_t header[16] = {};
    memcpy(header, "ABHIST01", 8);
    SdStorage::put_le16(header + 8, 1);
    SdStorage::put_le16(header + 10, CHANNEL_COUNT);
    const uint32_t length = sizeof(header) + CHANNEL_COUNT * 72 + total_bins(data.header) * 8;
    SdStorage::put_le32(header + 12, length);
    if (!bytes.reserve(length) || !bytes.append(reinterpret_cast<const char *>(header), sizeof(header))) return false;
    const auto u64 = [](uint8_t *out, uint64_t value) {
        SdStorage::put_le32(out, uint32_t(value));
        SdStorage::put_le32(out + 4, uint32_t(value >> 32));
    };
    const auto real = [&](uint8_t *out, double value) {
        static_assert(sizeof(value) == sizeof(uint64_t), "binary64 histogram values");
        uint64_t bits; memcpy(&bits, &value, sizeof(bits)); u64(out, bits);
    };
    size_t offset = 0;
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        const auto &channel = data.header.channels[c];
        uint8_t descriptor[72] = {};
        SdStorage::put_le16(descriptor, uint16_t(CHANNELS[c].artifact_id));
        SdStorage::put_le32(descriptor + 4, channel.bins);
        SdStorage::put_le32(descriptor + 8, uint32_t(channel.signal.digital_min));
        SdStorage::put_le32(descriptor + 12, uint32_t(channel.signal.digital_max));
        real(descriptor + 16, channel.signal.physical_min);
        real(descriptor + 24, channel.signal.physical_max);
        u64(descriptor + 32, channel.count); u64(descriptor + 40, channel.missing);
        real(descriptor + 48, channel.sum); real(descriptor + 56, channel.coverage_seconds);
        SdStorage::put_le32(descriptor + 64, channel.signal.samples_per_record);
        if (!bytes.append(reinterpret_cast<const char *>(descriptor), sizeof(descriptor))) return false;
        for (uint32_t bin = 0; bin < channel.bins; ++bin) {
            uint8_t value[8]; u64(value, data.bins[offset++]);
            if (!bytes.append(reinterpret_cast<const char *>(value), sizeof(value))) return false;
        }
    }
    char prefix[96]; uint8_t key[32];
    cache_path(entry, "histograms", prefix);
    return bytes.length() == length && ReportArtifact::fingerprint(
        reinterpret_cast<const uint8_t *>(bytes.c_str()), bytes.length(), key) &&
        ReportArtifact::document_publish(session, prefix, key,
            reinterpret_cast<const uint8_t *>(bytes.c_str()), bytes.length(), document);
}

bool prepare_json(const SdStorage::Session &session, Job &job) {
    if (job.request.view == View::Period) return prepare_period(session, job);
    const uint16_t day = last_day(job.request);
    if (!dataset(session)) return note_issue(job, "dataset_identity");
    char manifest[96], identity[65]; uint8_t manifest_key[32];
    publication_prefix(day, manifest);
    if (!publication_key(job.request, manifest_key)) return note_issue(job, "publication_key");
    ReportArtifact::Document saved;
    const auto known = ReportArtifact::document_lookup(session, manifest, manifest_key, saved);
    if (known == ReportArtifact::Lookup::Ready) {
        // Revalidate references only after a source/storage change or eviction.
        // scan_index clears this owner cache when files_revision changes.
        auto *cached = cached_day_entry(day, true);
        const bool checked = cached && cached->artifacts_checked &&
            !memcmp(cached->artifact_key, manifest_key, sizeof(manifest_key));
        for (size_t i = 0; !checked && i < index_count; ++i) {
            const auto &entry = index[i];
            if (!selected(entry, day, day)) continue;
            for (uint8_t kind = 0; kind < 3; ++kind) {
                if (!(entry.key.present & (1u << kind))) continue;
                char prefix[96]; uint8_t key[32];
                ReportArtifact::Publication publication;
                if (!artifact_identity(entry, kind, prefix, key) ||
                    ReportArtifact::lookup(session, prefix, key, publication) != ReportArtifact::Lookup::Ready)
                    return note_issue(job, "artifact_lookup", entry.entry.file_prefix);
            }
        }
        if (cached) {
            memcpy(cached->artifact_key, manifest_key, sizeof(manifest_key));
            cached->artifacts_checked = true;
        }
        if (ReportArtifact::document_read(session, manifest, saved, job.body, manifest_key)) return true;
        return note_issue(job, "publication_read", manifest);
    }
    if (known == ReportArtifact::Lookup::Failed || known == ReportArtifact::Lookup::Invalid)
        return note_issue(job, "publication_read", manifest);
    if (!artifact_headroom(session, day)) return note_issue(job, "recording_headroom");
    job.body.clear();
    aircannect::hex_encode(manifest_key, 32, identity, sizeof(identity), aircannect::HexCase::Lower);
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"dataset\":\"%s\",\"identity\":\"%s\",\"day\":%u,\"artifacts\":[", dataset_id, identity, day);
    bool comma = false;
    for (size_t i = 0; i < index_count; ++i) {
        const auto &entry = index[i];
        if (!selected(entry, day, day)) continue;
        bool missing = false;
        for (uint8_t kind = 0; kind < 3; ++kind) {
            if (!(entry.key.present & (1u << kind))) continue;
            char prefix[96]; uint8_t key[32];
            ReportArtifact::Publication publication;
            if (!artifact_identity(entry, kind, prefix, key)) return note_issue(job, "artifact_digest");
            const auto found = ReportArtifact::lookup(session, prefix, key, publication);
            if (found == ReportArtifact::Lookup::Missing) missing = true;
            else if (found != ReportArtifact::Lookup::Ready) return note_issue(job, "artifact_lookup", prefix);
        }
        if (missing) {
            std::unique_ptr<SessionData> data(new (std::nothrow) SessionData);
            if (!data) return note_issue(job, "artifact_alloc");
            if (!session_data(session, job, entry, *data, true)) return false;
        }
        for (uint8_t kind = 0; kind < 3; ++kind) {
            if (!(entry.key.present & (1u << kind))) continue;
            char prefix[96], digest[65]; uint8_t key[32];
            ReportArtifact::Publication publication;
            if (!artifact_identity(entry, kind, prefix, key) ||
                ReportArtifact::lookup(session, prefix, key, publication) != ReportArtifact::Lookup::Ready)
                return note_issue(job, "artifact_unavailable", entry.entry.file_prefix);
            aircannect::hex_encode(publication.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
            job.body.print("%s{\"session\":%u,\"day\":\"%s\",\"kind\":\"%s\",\"digest\":\"%s\",\"bytes\":%u}",
                           comma ? "," : "", unsigned(i), entry.entry.therapy_day, SUFFIXES[kind], digest, publication.bytes);
            comma = true;
        }
    }
    job.body.print("],\"event_groups\":[");
    comma = false;
    for (size_t i = 0; i < index_count; ++i) {
        const auto &entry = index[i];
        if (!selected(entry, day, day)) continue;
        std::unique_ptr<SessionData> storage(new (std::nothrow) SessionData);
        if (!storage) return note_issue(job, "session_alloc");
        auto &data = *storage;
        if (!session_data(session, job, entry, data)) return false;
        ReportArtifact::Document histogram;
        if (!publish_histograms(session, entry, data, histogram)) return note_issue(job, "histogram_publish");
        Payload events;
        bool event_comma = false, truncated = false;
        events.print("{\"events\":[");
        if (!window_events(session, job, entry, data.header, 0, INT64_MAX,
                           events, event_comma, truncated) || truncated) return note_issue(job, "event_limit");
        events.print("]}");
        char prefix[96], digest[65];
        cache_path(entry, "events", prefix);
        uint8_t event_key[32];
        ReportArtifact::Document publication;
        if (!events.good() || !ReportArtifact::fingerprint(reinterpret_cast<const uint8_t *>(events.c_str()), events.length(), event_key) ||
            !ReportArtifact::document_publish(session, prefix, event_key,
                reinterpret_cast<const uint8_t *>(events.c_str()), events.length(), publication)) return note_issue(job, "events_publish");
        aircannect::hex_encode(publication.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
        job.body.print("%s{\"session\":%u,\"day\":\"%s\",\"digest\":\"%s\",\"bytes\":%u,",
            comma ? "," : "", unsigned(i), entry.entry.therapy_day, digest, publication.bytes);
        aircannect::hex_encode(histogram.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
        job.body.print("\"histogram\":{\"day\":\"%s\",\"digest\":\"%s\",\"bytes\":%u}}",
            entry.entry.therapy_day, digest, histogram.bytes);
        comma = true;
    }
    job.body.print("],\"summary\":");
    Payload envelope;
    envelope.swap(job.body);
    // History uses the same exact session statistics, never browser aggregates.
    if (!history_json(session, job, true)) return false;
    job.body.clear();
    if (!summary_json(session, job)) return false;
    envelope.append(job.body.c_str(), job.body.length());
    envelope.print("}");
    job.body.swap(envelope);
    if (!job.body.good() || aborted(&job, session)) return false;
    if (job.issue) {
        ReportArtifact::Document previous;
        const bool prior = ReportArtifact::document_lookup(session, manifest, nullptr, previous) == ReportArtifact::Lookup::Ready;
        if (strcmp(job.issue, "partial_source") || prior) return false;
    }
    // A catalog/STR update during production must not become the current head.
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (current.generation != catalog_generation || SdStorage::files_revision() != file_revision) return false;
    if (!ReportArtifact::document_publish(session, manifest, manifest_key,
        reinterpret_cast<const uint8_t *>(job.body.c_str()), job.body.length(), saved))
        return note_issue(job, "publication_write", manifest);
    remember_publication(day, 0, saved);
    if (auto *cached = cached_day_entry(day, true)) {
        memcpy(cached->artifact_key, manifest_key, sizeof(manifest_key));
        cached->artifacts_checked = true;
    }
    __atomic_add_fetch(&cache_generation, 1, __ATOMIC_RELEASE);
    portENTER_CRITICAL(&status_mux);
    ++status.artifact_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}
void append_event(Payload &body, const Event &event, bool &comma, const char *type = nullptr) {
    body.print("%s{\"time_ms\":%lld,\"duration_ms\":%lld,\"type\":\"%s\"}",
               comma ? "," : "", static_cast<long long>(event.time_ms),
               static_cast<long long>(event.duration_ms), type ? type : event_name(event.kind));
    comma = true;
}

bool days_json(const SdStorage::Session &session, Job &job) {
    if (!dataset(session)) return note_issue(job, "dataset_identity");
    const size_t bytes = index_count * sizeof(uint16_t);
    auto *days = bytes ? static_cast<uint16_t *>(aircannect::Memory::alloc_large(bytes,
                                                       bytes <= INTERNAL_WORK)) : nullptr;
    if (bytes && !days) return false;
    size_t count = 0;
    bool partial = false;
    for (size_t i = 0; i < index_count; ++i) {
        if (!(index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE)) continue;
        days[count++] = index[i].day;
        if (index[i].key.present != 31 || index[i].key.checksums != 31) partial = true;
    }
    if (count) std::sort(days, days + count, std::greater<uint16_t>());
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"dataset\":\"%s\",\"partial\":%s,\"days\":[", dataset_id, partial ? "true" : "false");
    for (size_t n = 0; n < count;) {
        const uint16_t day = days[n];
        size_t end = n + 1;
        while (end < count && days[end] == day) ++end;
        job.body.print("%s{\"day\":%u,\"session_count\":%u", n ? "," : "", day,
                       static_cast<unsigned>(end - n));
        char prefix[96]; uint8_t key[32];
        publication_prefix(day, prefix);
        Request request; request.day = day;
        ReportArtifact::Document document;
        if (publication_key(request, key) &&
            lookup_publication(session, day, 0, prefix, document) == ReportArtifact::Lookup::Ready) {
            char digest[65], identity[65];
            aircannect::hex_encode(document.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
            aircannect::hex_encode(document.key, 32, identity, sizeof(identity), aircannect::HexCase::Lower);
            job.body.print(",\"publication\":{\"day\":\"%.8s\",\"digest\":\"%s\",\"bytes\":%u},\"identity\":\"%s\",\"stale\":%s",
                prefix + strlen(CACHE_DIR) + 1, digest, document.bytes, identity, memcmp(key, document.key, 32) ? "true" : "false");
        }
        job.body.print("}");
        n = end;
    }
    job.body.print("],\"periods\":[");
    bool period_comma = false;
    for (uint16_t period : PERIODS) {
        Request request; request.view = View::Period; request.period = period;
        char prefix[96]; uint8_t key[32];
        period_prefix(request, prefix);
        ReportArtifact::Document document;
        if (!publication_key(request, key) ||
            lookup_publication(session, last_day(request), period, prefix, document) != ReportArtifact::Lookup::Ready) continue;
        char digest[65];
        aircannect::hex_encode(document.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
        job.body.print("%s{\"period\":%u,\"day\":\"%.8s\",\"digest\":\"%s\",\"bytes\":%u,\"stale\":%s}",
            period_comma ? "," : "", period, prefix + strlen(CACHE_DIR) + 1,
            digest, document.bytes, memcmp(key, document.key, 32) ? "true" : "false");
        period_comma = true;
    }
    job.body.print("]}");
    aircannect::Memory::free(days);
    return job.body.good();
}

bool discard_index(const SdStorage::Session &session, const uint8_t digest[32]) {
    // Pair admission with open_artifact: a reader cannot enter between the
    // last active-read check and removal. No I/O runs under the critical lock.
    portENTER_CRITICAL(&status_mux);
    const bool admitted = !__atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE) && !index_deleting;
    if (admitted) index_deleting = true;
    portEXIT_CRITICAL(&status_mux);
    if (!admitted) return false;
    const bool removed = ReportArtifact::index_discard(session, digest);
    portENTER_CRITICAL(&status_mux);
    index_deleting = false;
    portEXIT_CRITICAL(&status_mux);
    return removed;
}

bool retire_index(const SdStorage::Session &session) {
    if (!retired_index.bytes) return true;
    if (retired_index_files != SdStorage::files_revision()) {
        retired_index = {};
        return true; // External edits invalidate provenance; leave it for discovery.
    }
    if (!discard_index(session, retired_index.digest)) return false;
    retired_index = {};
    return true;
}

bool publish_index(const SdStorage::Session &session) {
    if (!retire_index(session)) return false;
    const uint32_t artifacts = status.artifact_revision;
    const uint32_t data = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (published_index_data == data && published_index_artifacts == artifacts) {
        index_commit_pending = false;
        return true;
    }
    Job snapshot;
    snapshot.data_revision = data;
    if (!days_json(session, snapshot)) return false;
    ReportArtifact::Document document;
    uint8_t key[32];
    if (!ReportArtifact::fingerprint(reinterpret_cast<const uint8_t *>(snapshot.body.c_str()), snapshot.body.length(), key) ||
        !ReportArtifact::index_publish(session, key,
            reinterpret_cast<const uint8_t *>(snapshot.body.c_str()), snapshot.body.length(), document, retired_index)) return false;
    published_index_data = data;
    published_index_artifacts = artifacts;
    retired_index_files = SdStorage::files_revision();
    index_commit_pending = false;
    retire_index(session);
    return true;
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
        std::unique_ptr<SessionData> storage(new (std::nothrow) SessionData);
        if (!storage) return note_issue(job, "session_alloc");
        auto &data = *storage;
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

double chart_scale(size_t c, const ReportEdf::Signal &signal) {
    if ((c == 0 || c == 4) && !strcmp(signal.unit, "L/s")) return 60;
    if (c == 7 && !strcmp(signal.unit, "L")) return 1000;
    return 1;
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
    if (rebuild_generation) return true;
    SdStorage::Reader directory;
    if (!directory.open(session, CACHE_DIR, true))
        return session.run([&](fs::FS &fs) { return !fs.exists(CACHE_DIR); });
    bool end = false;
    SdStorage::Reader::Entry file;
    while (!end) {
        if (aborted(nullptr, session)) return false;
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
    std::unique_ptr<SessionData> data(new (std::nothrow) SessionData);
    if (!data) return note_issue(job, "session_alloc");
    if (!session_data(session, job, entry, *data)) return false;
    header = data->header;
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

bool reindex(const SdStorage::Session &session, Job &job) {
    if (!dataset(session) || rebuild_generation == UINT32_MAX) return note_issue(job, "rebuild_generation");
    uint8_t value[4], key[32] = {};
    SdStorage::put_le32(value, rebuild_generation + 1);
    ReportArtifact::Document document;
    if (!ReportArtifact::document_publish(session, "/airbridge/report/rebuild", key, value, sizeof(value), document))
        return note_issue(job, "rebuild_publish");
    ++rebuild_generation;
    invalidate_day(); invalidate_str();
    backfill_revision = 0; backfill_before = UINT16_MAX; backfill_period = 0;
    indexed_invalidation = __atomic_add_fetch(&invalidation_generation, 1, __ATOMIC_RELEASE);
    job.data_revision = __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
    published_index_data = 0;
    portENTER_CRITICAL(&status_mux);
    status.data_revision = job.data_revision;
    ++status.artifact_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    job.body.print("{\"source\":\"edf\",\"rebuilding\":true,\"data_revision\":%lu}",
        static_cast<unsigned long>(job.data_revision));
    Log::logf(CAT_REPORT, LOG_INFO, "Report rebuild queued generation=%lu; previous publications preserved\n",
        static_cast<unsigned long>(rebuild_generation));
    return job.body.good();
}

void defer_job(Job *job) {
    if (!job) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) reset_job(*job);
    else job->state = JobState::Queued;
    xSemaphoreGive(mutex);
}

bool enqueue_backfill(const SdStorage::Session &session) {
    if (!backfill_enabled || ExportSync::busy()) return false;
    if (background_retry && int32_t(millis() - background_retry) < 0) return false;
    if (background_retry) { background_retry = 0; backfill_before = UINT16_MAX; backfill_period = 0; }
    const uint32_t current = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (backfill_revision != current) { backfill_revision = current; backfill_before = UINT16_MAX; backfill_period = 0; }
    uint16_t day = 0;
    for (size_t i = 0; i < index_count; ++i)
        if (selected(index[i], 1, backfill_before - (backfill_before != 0))) day = std::max(day, index[i].day);
    if (!day) backfill_before = 0;
    if (!day && backfill_period >= sizeof(PERIODS) / sizeof(PERIODS[0])) return false;
    if (!dataset(session)) return false;
    Request request; request.action = Action::Prepare; request.day = day;
    char prefix[96]; uint8_t key[32];
    if (day) publication_prefix(day, prefix);
    else {
        request.view = View::Period; request.period = PERIODS[backfill_period];
        period_prefix(request, prefix);
    }
    if (!publication_key(request, key)) return false;
    ReportArtifact::Document document;
    const bool ready = lookup_publication(session, last_day(request), day ? 0 : request.period,
        prefix, document) == ReportArtifact::Lookup::Ready && !memcmp(key, document.key, sizeof(key));
    bool refused = false;
    if (!ready) {
        strcat(prefix, ".refused");
        refused = ReportArtifact::document_lookup(session, prefix, key, document) == ReportArtifact::Lookup::Ready;
    }
    if (ready || refused) {
        if (day) backfill_before = day; else ++backfill_period;
        return true;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    Job *slot = nullptr;
    for (auto &candidate : jobs) if (reusable(candidate) || evictable(candidate)) { slot = &candidate; break; }
    if (slot) {
        reset_job(*slot);
        slot->request = request;
        slot->id = next_id++;
        if (!next_id) next_id = 1;
        slot->state = JobState::Queued;
        slot->background = true;
        slot->foreground_epoch = __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE);
        slot->consumers = 0;
    }
    xSemaphoreGive(mutex);
    return slot != nullptr;
}

bool work_once() {
    static const char *reported_index_error = nullptr;
    Job *job = nullptr;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto &candidate : jobs) {
        if (candidate.state != JobState::Free && reusable(candidate)) reset_job(candidate);
        if (candidate.state == JobState::Queued && (!job ||
            (job->background && !candidate.background) || (job->background == candidate.background &&
            int32_t(candidate.id - job->id) < 0))) job = &candidate;
    }
    if (job) job->state = JobState::Running;
    xSemaphoreGive(mutex);
    if (!AirSenseState::local_background_allowed()) {
        defer_job(job); publish(State::Blocked, "not_idle"); return false;
    }
    if (!SdStorage::local_access_allowed()) { defer_job(job); publish(State::Blocked, "usb_storage"); return false; }
    if (!SdStorage::mounted()) { defer_job(job); publish(State::Blocked, "no_sd"); return false; }
    if (__atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE)) {
        defer_job(job); publish(State::WaitingStorage, "artifact_read"); return false;
    }
    if (job && job->background && ExportSync::busy()) {
        defer_job(job); publish(State::WaitingStorage, "export_pending"); return false;
    }
    if (!job && !dirty() && !index_commit_pending && !retired_index.bytes && published_index_artifacts == status.artifact_revision &&
        (!backfill_enabled || (!backfill_before && backfill_period >= sizeof(PERIODS) / sizeof(PERIODS[0])) || ExportSync::busy() ||
        (background_retry && int32_t(millis() - background_retry) < 0))) { publish(State::Ready); return false; }
    SdStorage::Session session;
    if (!session.begin()) { defer_job(job); publish(State::WaitingStorage, "storage_busy"); return false; }
    JobTiming timing(job);
    publish(State::Building);
    const uint32_t index_started = timing.stamp();
    bool recovered_index = false;
    if (job && job->request.action == Action::Days && !index_ready) {
        ReportArtifact::Document document;
        Payload saved;
        if (ReportArtifact::document_lookup(session, "/airbridge/report/index", nullptr, document) == ReportArtifact::Lookup::Ready &&
            ReportArtifact::document_read(session, "/airbridge/report/index", document, saved, nullptr) &&
            saved.length() && saved.c_str()[0] == '{') {
            job->body.print("{\"stale\":true,");
            job->body.append(saved.c_str() + 1, saved.length() - 1);
            recovered_index = job->body.good();
        }
    }
    const bool indexed = recovered_index || refresh_index(session);
    timing.index_us = timing.stamp() - index_started;
    if (!indexed) {
        const bool preempted = !session.valid() || !AirSenseState::local_background_allowed();
        session.end();
        const bool reading = __atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE);
        if (preempted || reading || index_failure == IndexFailure::Preempted || index_failure == IndexFailure::WaitingCatalog) {
            defer_job(job);
            publish(preempted ? State::Blocked : State::WaitingStorage,
                    preempted ? "not_idle" : reading ? "artifact_read" : "catalog_not_ready");
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
                timing.log(code, bytes);
                xSemaphoreGive(mutex);
                portENTER_CRITICAL(&status_mux);
                ++status.revision;
                portEXIT_CRITICAL(&status_mux);
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
        const bool checkpoint = index_commit_pending && !ExportSync::busy();
        if (checkpoint && !publish_index(session))
            Log::logf(CAT_REPORT, LOG_DEBUG, "Report index publication deferred\n");
        const bool more = enqueue_backfill(session);
        // Backfill publishes each day independently. Persist the derived list
        // only when this run finishes or safely pauses, not after every day.
        if (!more && !checkpoint && !ExportSync::busy() && !publish_index(session))
            Log::logf(CAT_REPORT, LOG_DEBUG, "Report index publication deferred\n");
        session.end();
        publish(State::Ready);
        return more;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    job->state = JobState::Running;
    job->data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    job->code = 0;
    job->error = job->issue = nullptr;
    job->issue_path[0] = 0;
    job->artifact_preempted = false;
    job->foreground_epoch = __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE);
    xSemaphoreGive(mutex);
    bool success = false;
    if (__atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) error_body(*job, 410, "cancelled");
    else if (job->request.revision && job->request.revision != job->data_revision)
        error_body(*job, 409, "stale_revision");
    else {
        if (recovered_index) success = true;
        else if (job->request.action == Action::Reindex) success = reindex(session, *job);
        else if (job->request.action == Action::History) {
            start_json(job->body, revision() + 1, job->data_revision);
            job->body.print("\"day\":%u", last_day(job->request));
            success = history_json(session, *job, true);
            job->body.print("}");
        } else if (job->request.action == Action::Prepare) success = prepare_json(session, *job);
        else success = job->request.action == Action::Days ? days_json(session, *job) : summary_json(session, *job);
        if (success) job->code = 200;
    }
    const bool interrupted = !AirSenseState::local_background_allowed() || !session.valid();
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const bool changed = !recovered_index && (catalog.generation != catalog_generation || SdStorage::files_revision() != file_revision ||
        job->data_revision != __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE));
    const bool yielded = job->artifact_preempted || __atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE) ||
        (job->background && job->foreground_epoch != __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE));
    if (success && !job->background && job->request.action == Action::Prepare &&
        !interrupted && !changed && !yielded) {
        // Includes post-therapy preparation before export. Failure to checkpoint
        // must not invalidate an independently committed day publication.
        index_commit_pending = true;
        if (!publish_index(session)) Log::logf(CAT_REPORT, LOG_DEBUG, "Report index publication deferred\n");
    }
    if (job->background && !interrupted && !changed && !yielded) {
        if (job->request.view == View::Period) ++backfill_period;
        else backfill_before = job->request.day;
        if (!success && job->issue && (!strcmp(job->issue, "partial_source") ||
            !strcmp(job->issue, "source_header") || !strcmp(job->issue, "artifact_lookup") ||
            !strcmp(job->issue, "numeric_files_missing"))) {
            char prefix[96]; uint8_t key[32]; ReportArtifact::Document refused;
            if (job->request.view == View::Period) period_prefix(job->request, prefix);
            else publication_prefix(job->request.day, prefix);
            strcat(prefix, ".refused");
            if (publication_key(job->request, key)) ReportArtifact::document_publish(session, prefix, key,
                reinterpret_cast<const uint8_t *>(job->issue), strlen(job->issue), refused);
        } else if (!success && (!job->issue || strcmp(job->issue, "recording_headroom"))) background_retry = millis() + 30000;
    }
    session.end();
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) error_body(*job, 410, "cancelled");
    else if (interrupted || changed || yielded) {
        job->body.~Payload();
        new (&job->body) Payload;
        job->state = JobState::Queued;
        job->code = 0;
        xSemaphoreGive(mutex);
        publish(interrupted ? State::Blocked : State::Building, interrupted ? "not_idle" : "");
        return yielded;
    } else if (!success && !job->code) error_body(*job, 503, "report_build_failed");
    job->state = JobState::Complete;
    job->completed_ms = millis();
    job->cache_generation = __atomic_load_n(&cache_generation, __ATOMIC_ACQUIRE);
    job->artifact_revision = status.artifact_revision;
    const int code = job->code;
    const size_t bytes = job->body.length();
    log_result(*job);
    timing.log(code, bytes);
    xSemaphoreGive(mutex);
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    publish(State::Ready);
    return true;
}

void worker(void *) {
    while (true) {
        if (work_once()) continue;
        Status publication;
        get_status(publication);
        xSemaphoreTake(wake, background_retry || publication.state == State::WaitingStorage || publication.state == State::Blocked
            ? pdMS_TO_TICKS(1000) : portMAX_DELAY);
    }
}

void artifact_read_released() {
    __atomic_sub_fetch(&artifact_reads, 1, __ATOMIC_ACQ_REL);
    if (wake) xSemaphoreGive(wake);
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
            (request.action != Action::Days || job.artifact_revision == publication.artifact_revision) &&
            ((request.action != Action::Summary && request.action != Action::History && request.action != Action::Days) ||
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
            if (job.state == JobState::Queued) job.background = false;
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
    __atomic_add_fetch(&foreground_epoch, 1, __ATOMIC_RELEASE);
    id = slot->id;
    xSemaphoreGive(mutex);
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    xSemaphoreGive(wake);
    return true;
}

uint32_t prepare_before_export(const char *therapy_day) {
    Request request; request.action = Action::Prepare;
    if (!therapy_day || !catalog_day(therapy_day, request.day)) return 0;
    uint32_t id = 0;
    return submit(request, id) ? id : 0;
}

bool preparation_pending(uint32_t id) {
    if (!id || !mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
    bool pending = false;
    for (const auto &job : jobs) if (job.id == id)
        pending = job.state == JobState::Queued || job.state == JobState::Running;
    xSemaphoreGive(mutex);
    return pending;
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

StorageBrowser::StartResult open_artifact(const char *day, const char *digest,
    bool document, StorageBrowser::Request &operation, StorageBrowser::Ready ready,
    std::weak_ptr<StorageBrowser::Transfer> &active) {
    uint16_t parsed = 0;
    const bool root = day && !strcmp(day, "index");
    if ((!root && (!day || !catalog_day(day, parsed))) || !digest || strlen(digest) != 64)
        return StorageBrowser::StartResult::BadRequest;
    for (const char *p = digest; *p; ++p)
        if (!(*p >= '0' && *p <= '9') && !(*p >= 'a' && *p <= 'f')) return StorageBrowser::StartResult::BadRequest;
    if (root && !document) return StorageBrowser::StartResult::BadRequest;
    // Keep priority until the reader releases SD, not just until backfill
    // observes an epoch change and immediately starts its next attempt.
    portENTER_CRITICAL(&status_mux);
    if (index_deleting) {
        portEXIT_CRITICAL(&status_mux);
        return StorageBrowser::StartResult::Busy;
    }
    __atomic_add_fetch(&artifact_reads, 1, __ATOMIC_ACQ_REL);
    portEXIT_CRITICAL(&status_mux);
    __atomic_add_fetch(&foreground_epoch, 1, __ATOMIC_RELEASE);
    operation.kind = StorageBrowser::Kind::Immutable;
    operation.selection[0] = 0;
    snprintf(operation.path, sizeof(operation.path), "%s/%s%s%.*s%s%s.%s", CACHE_DIR,
        root ? "" : day, root ? "" : "/", document ? 2 : 0, digest,
        document ? "/" : "", digest, document ? "abp" : "abt");
    const auto result = StorageBrowser::start(operation, std::move(ready), active, artifact_read_released);
    if (result != StorageBrowser::StartResult::Started) artifact_read_released();
    return result;
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
uint32_t prepare_before_export(const char *) { return 0; }
bool preparation_pending(uint32_t) { return false; }
StorageBrowser::StartResult open_artifact(const char *, const char *, bool, StorageBrowser::Request &,
    StorageBrowser::Ready, std::weak_ptr<StorageBrowser::Transfer> &) {
    return StorageBrowser::StartResult::Unavailable;
}
int poll(uint32_t, Result &out) { out.reset(); return 422; }
void Result::reset() { id_ = 0; data_ = nullptr; length_ = 0; }
}  // namespace EdfReport
#endif
