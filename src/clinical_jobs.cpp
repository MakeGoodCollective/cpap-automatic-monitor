#include "clinical_jobs.h"
#include "app_config.h"
#include "debug_log.h"
#include "memory_manager.h"
#include <esp_heap_caps.h>
#include <freertos/semphr.h>
#include <utility>
#include <new>
#include <atomic>

namespace ClinicalJobs {
namespace {
constexpr uint32_t DEADLINE_MS = 30000;
constexpr uint32_t RETAIN_MS = 30000;
constexpr uint8_t SLOT_COUNT = 4;
enum State : uint8_t { Free, Queued, Running, Complete };
struct Job {
    State state = Free;
    Kind kind = Kind::Read;
    bool delivered = false;
    uint16_t readers = 0;
    uint32_t id = 0, queued_ms = 0, completed_ms = 0;
    int code = 0;
    String body, result;
    ClinicalSettings::Snapshot snapshot;
    SleepReport::Request report;
    SleepReport::Snapshot *report_snapshot = nullptr;
    size_t report_length = 0;

    ~Job() { aircannect::Memory::free(report_snapshot); }
};
Job jobs[SLOT_COUNT];
SemaphoreHandle_t mutex = nullptr;
std::atomic<TaskHandle_t> task{nullptr};
std::atomic<uint32_t> completed_revision{0};
Handler handler = nullptr;
uint32_t next_id = 1, active_since = 0;

bool reusable(const Job &job) {
    return job.state == Free || (job.state == Complete && !job.readers &&
        uint32_t(millis() - job.completed_ms) >= (job.delivered ? 2000u : RETAIN_MS));
}

void reset_job(Job &job) {
    // Arduino String move-assignment from an empty value retains capacity.
    job.~Job();
    new (&job) Job;
}

void worker(void *) {
    while (true) {
        CustomSettings::reclaim();
        Job *job = nullptr;
        String body;
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (Job &candidate : jobs) {
            if (candidate.state == Complete && reusable(candidate))
                reset_job(candidate);
            if (candidate.state == Queued &&
                (!job || int32_t(candidate.id - job->id) < 0)) job = &candidate;
        }
        if (!job) {
            task = nullptr;
            xSemaphoreGive(mutex);
            break;
        }
        job->state = Running;
        active_since = job->queued_ms;
        body = std::move(job->body);
        xSemaphoreGive(mutex);
        String result;
        int code = 504;
        if (remaining_ms()) {
            if (job->kind == Kind::Write) code = handler(body, result);
            else if (job->kind == Kind::Report) {
                void *memory = aircannect::Memory::alloc_large(sizeof(SleepReport::Snapshot));
                if (!memory) {
                    code = 503;
                    result = "{\"error\":\"report_allocation_failed\"}";
                } else {
                    job->report_snapshot = new (memory) SleepReport::Snapshot;
                    code = SleepReport::collect(*job->report_snapshot, job->report, remaining_ms);
                    if (code == 200)
                        job->report_length = SleepReport::json_length(*job->report_snapshot);
                    else
                        result = String("{\"error\":\"") + job->report_snapshot->error + "\"}";
                }
            } else {
                code = ClinicalSettings::collect(job->snapshot);
                if (code != 200) result = job->snapshot.error();
            }
        }
        if (!remaining_ms() && code == 200) code = 504;
        if (code == 504) {
            job->snapshot.reset();
            aircannect::Memory::free(job->report_snapshot);
            job->report_snapshot = nullptr;
            job->report_length = 0;
            result = job->kind == Kind::Report ? "{\"error\":\"report_deadline\"}" :
                "{\"error\":\"settings_deadline\",\"partial_write_possible\":true}";
        }
        const char *operation = job->kind == Kind::Write ? "write" :
                                job->kind == Kind::Report ? "report" : "read";
        log_cat_t category = job->kind == Kind::Report ? CAT_REPORT : CAT_CONFIG;
        if (code != 200) {
            Log::logf(category, code >= 500 ? LOG_ERROR : LOG_WARN,
                      "%s failed status=%d reason=%.64s%s\n", operation, code, result.c_str(),
                      code == 504 && job->kind == Kind::Write ? "; partial write possible" : "");
        } else {
            Log::logf(category, LOG_DEBUG, "%s job=%u elapsed=%ums\n",
                      operation, job->id, (unsigned)(millis() - active_since));
        }
        xSemaphoreTake(mutex, portMAX_DELAY);
        job->result = std::move(result);
        job->code = code;
        job->completed_ms = millis();
        job->state = Complete;
        xSemaphoreGive(mutex);
        ++completed_revision;
    }
    CustomSettings::reclaim();
    vTaskDeleteWithCaps(nullptr);
}

bool start_worker() {
    TaskHandle_t created_task = nullptr;
    BaseType_t created = xTaskCreatePinnedToCoreWithCaps(
        worker, "clinical", 6144, nullptr, 2, &created_task, 0,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (created != pdPASS)
        created = xTaskCreatePinnedToCoreWithCaps(
            worker, "clinical", 6144, nullptr, 2, &created_task, 0,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (created == pdPASS) task = created_task;
    else Log::logf(CAT_WEB, LOG_ERROR, "Settings/report worker allocation failed\n");
    return created == pdPASS;
}
}

void init(Handler callback) {
    if (mutex) return;
    handler = callback;
    mutex = xSemaphoreCreateMutex();
    if (!mutex) Log::logf(CAT_WEB, LOG_ERROR, "Settings/report mutex allocation failed\n");
}

void tick() {
    static uint32_t last_cleanup_ms = 0;
    uint32_t now = millis();
    if (uint32_t(now - last_cleanup_ms) < 1000) return;
    last_cleanup_ms = now;

    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return;
    for (Job &job : jobs)
        if (job.state == Complete && reusable(job)) reset_job(job);
    xSemaphoreGive(mutex);
    CustomSettings::reclaim();
}

uint32_t revision() { return completed_revision.load(); }

Result::~Result() { reset(); }

Result::Result(Result &&other) noexcept
    : id_(std::exchange(other.id_, 0)), data_(std::exchange(other.data_, nullptr)),
      snapshot_(std::exchange(other.snapshot_, nullptr)),
      report_(std::exchange(other.report_, nullptr)), length_(std::exchange(other.length_, 0)) {}

void Result::reset() {
    if (!id_) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (Job &job : jobs) {
        if (job.id != id_) continue;
        if (job.readers) job.readers--;
        if (reusable(job)) reset_job(job);
        break;
    }
    xSemaphoreGive(mutex);
    id_ = 0;
    data_ = nullptr;
    snapshot_ = nullptr;
    report_ = nullptr;
    length_ = 0;
}

size_t Result::read(Cursor &cursor, size_t offset,
                    char *out, size_t capacity) const {
    if (report_) {
        auto *report_cursor = std::get_if<SleepReport::Cursor>(&cursor);
        if (!report_cursor) report_cursor = &cursor.emplace<SleepReport::Cursor>();
        return report_cursor->read(*report_, out, capacity);
    }
    if (snapshot_) {
        auto *settings_cursor = std::get_if<ClinicalSettings::Cursor>(&cursor);
        if (!settings_cursor) settings_cursor = &cursor.emplace<ClinicalSettings::Cursor>();
        return settings_cursor->read(*snapshot_, out, capacity);
    }
    if (!data_ || offset >= length_) return 0;
    size_t count = min(capacity, length_ - offset);
    memcpy(out, data_ + offset, count);
    return count;
}

bool submit(Kind kind, String &&body, uint32_t &id, SleepReport::Request report) {
    if (!mutex || body.length() > MAX_BODY_SIZE || xSemaphoreTake(mutex, 0) != pdTRUE)
        return false;
    Job *available = nullptr;
    bool mutation_pending = false;
    for (Job &job : jobs) {
        if (reusable(job)) available = &job;
        if (job.kind == Kind::Write && (job.state == Queued || job.state == Running))
            mutation_pending = true;
    }
    if (kind != Kind::Write && !mutation_pending) {
        for (Job &job : jobs) {
            if (job.kind == kind && (kind != Kind::Report || job.report == report) &&
                (job.state == Queued || job.state == Running)) {
                id = job.id;
                xSemaphoreGive(mutex);
                return true;
            }
        }
    }
    if (!available || mutation_pending) {
        xSemaphoreGive(mutex);
        return false;
    }
    reset_job(*available);
    available->kind = kind;
    available->report = report;
    available->body = std::move(body);
    available->id = next_id++;
    if (!next_id) next_id = 1;
    available->queued_ms = millis();
    available->state = Queued;
    if (!task && !start_worker()) {
        reset_job(*available);
        xSemaphoreGive(mutex);
        return false;
    }
    id = available->id;
    xSemaphoreGive(mutex);
    return true;
}

int poll(uint32_t id, Result &result) {
    result.reset();
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return 503;
    int code = 410;
    for (Job &job : jobs) {
        if (job.state == Free || job.id != id) continue;
        if (job.state == Complete) {
            if (uint32_t(millis() - job.completed_ms) < RETAIN_MS) {
                size_t length = job.report_length ? job.report_length :
                    job.snapshot.length() ? job.snapshot.length() : job.result.length();
                if (!length || job.readers == UINT16_MAX) {
                    code = 503;
                    break;
                }
                job.readers++;
                result.id_ = job.id;
                result.snapshot_ = job.snapshot.length() ? &job.snapshot : nullptr;
                result.report_ = job.report_length ? job.report_snapshot : nullptr;
                result.data_ = result.snapshot_ || result.report_ ? nullptr : job.result.c_str();
                result.length_ = length;
                code = job.code;
                job.delivered = true;
            }
        } else code = 202;
        break;
    }
    xSemaphoreGive(mutex);
    return code;
}

uint16_t remaining_ms(uint32_t reserve_ms) {
    if (xTaskGetCurrentTaskHandle() != task.load()) return UINT16_MAX;
    uint32_t elapsed = millis() - active_since;
    if (elapsed >= DEADLINE_MS || reserve_ms >= DEADLINE_MS - elapsed) return 0;
    return DEADLINE_MS - elapsed - reserve_ms;
}
}
