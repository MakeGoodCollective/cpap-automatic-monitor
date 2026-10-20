#include "export_sync.h"
#include "edf_report.h"

#include "board.h"
#include <string.h>
#include <strings.h>

#if AB_STORAGE_HAS_SDCARD

#include <Arduino.h>
#include <FS.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "app_config.h"
#include "background_operation_control.h"
#include "crc.h"
#include "debug_log.h"
#include "job_progress_watch.h"
#include "memory_manager.h"
#include "sd_storage.h"
#include "sleephq_sync.h"
#include "storage_smb_client.h"
#include "tls_memory.h"
#include "uart_arbiter.h"
#include "airsense_state.h"
#include "air10_clock.h"
#include "wifi_setup.h"

namespace ExportSync {
namespace {

using aircannect::BackgroundOperationControl;
using aircannect::BackgroundOperationTimeoutPolicy;
using aircannect::StorageSmbClient;
using aircannect::StorageSmbOperationResult;
using aircannect::StorageSmbRemoteStat;

constexpr size_t UPLOAD_BUFFER_PSRAM = 16 * 1024;
constexpr size_t UPLOAD_BUFFER_FALLBACK = 4 * 1024;
constexpr uint32_t OPERATION_IDLE_TIMEOUT_MS = 60000;
constexpr uint32_t SLEEPHQ_IDLE_TIMEOUT_MS = 5 * 60 * 1000UL;
constexpr uint32_t DNS_TIMEOUT_MS = 15000;
// Upload callers stay on the stack during TLS certificate verification.
constexpr uint16_t EXPORT_TASK_STACK = 12 * 1024;
static const char *const SESSION_SUFFIXES[] = {"BRP", "PLD", "SAD", "EVE", "CSL"};
static const char *const ROOT_FILES[] = {"/Identification.tgt", "/Identification.crc", "/STR.edf"};

enum class RequestKind : uint8_t {
    PostTherapy,
    Manual,
    Startup,
    Retry,
};

struct Request {
    RequestKind kind;
    Engine engine;
    Action action;
    EdfCatalog::Entry entry;
    uint32_t generation;
    uint32_t config_revision;
    uint32_t queued_ms;
    char from_day[9];
    char to_day[9];
};

struct ExportProgress {
    aircannect::JobProgressWatch watch;
    uint32_t generation = 0, config_revision = 0, queued_ms = 0;
    Engine engine = Engine::Smb;
    Action action = Action::Sync;
    char from_day[9] = {}, to_day[9] = {};
    const char *phase = "admission";
    const char *reason = "queued";
    bool active = false;
    bool queued = false;
};

struct RunContext {
    uint32_t generation;
    BackgroundOperationControl operation;
    bool network_only;
    uint32_t config_revision;
    Config::Section section;
    // Reservation expiry cannot undo cancellation already observed by I/O.
    bool storage_yielded;
    ExportProgress *progress;
    uint64_t progress_step;
    const SleepHqSync::Progress *sleephq_progress;
    const Request *request;
    bool defer_storage_yield;
};

enum class RunResult : uint8_t { Finished, Yielded };

struct EngineAdapter {
    Engine id;
    Config::Section section;
    uint8_t actions;
    bool AirBridgeConfig::*enabled;
    bool AirBridgeConfig::*automatic;
    bool (*configured)();
    State *state;
    CheckStatus *check;
    portMUX_TYPE *mux;
    void (*set_state)(State, const char *);
    bool (*check_connection)(RunContext &, char *, size_t);
    RunResult (*run)(const Request &);

    bool supports(Action action) const {
        return action <= Action::Repair && (actions & (1u << unsigned(action)));
    }
};

extern const EngineAdapter engines[];
static const EngineAdapter *engine_adapter(Engine id) {
    for (size_t i = 0; i < size_t(Engine::Count); ++i)
        if (engines[i].id == id) return &engines[i];
    return nullptr;
}

struct SmbConfig {
    char endpoint[161];
    char user[aircannect::AC_STORAGE_SMB_USER_MAX];
    char password[aircannect::AC_STORAGE_SMB_PASSWORD_MAX];
};

static Status status = {true, State::Disabled};
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
static SleepHqStatus sleephq_status = {true, State::Disabled};
static portMUX_TYPE sleephq_status_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t request_queue = nullptr;
static TaskHandle_t export_task_handle = nullptr;
static SdStorage::Session storage;
static volatile uint32_t abort_generation = 1;
static uint32_t status_revision = 0;
static bool check_pending = false;
static bool backlog_refresh = true;
static bool backlog_files_changed = false;
static uint32_t backlog_smb_revision = UINT32_MAX;
static uint32_t backlog_shq_revision = UINT32_MAX;
static uint32_t backlog_catalog_generation = UINT32_MAX;
static uint32_t backlog_files_revision = UINT32_MAX;
enum : uint8_t { SMB_VALID = 1, SHQ_VALID = 2, WAITING = 4, SHQ_PENDING = 8 };
struct BacklogEntry {
    uint8_t smb_files;
    uint8_t shq_files;
    uint8_t flags;
};
static BacklogEntry *backlog_entries = nullptr;
static uint32_t backlog_entry_count = 0;
static bool roots_cached = false;
static uint32_t roots_pending = 0;
static uint32_t roots_endpoint = 0;
static uint32_t roots_catalog_generation = 0;

// Only export_task mutates retry policy; published timing lives in its status.
struct SleepHqRetry {
    uint32_t config_revision;
    uint32_t due_ms;
    uint8_t attempt;
    uint8_t heap_attempt;
    bool pending;
    bool blocked;
    Action action;
    char from_day[9];
    char to_day[9];
};
static SleepHqRetry sleephq_retry = {};
// Worker-owned diagnostics survive cooperative retries without retaining I/O.
static ExportProgress export_progress[size_t(Engine::Count)];

static void publish_activity(Engine engine, const char *phase, const char *wait,
                             const char *file = nullptr);

static void note_export_progress(ExportProgress &progress, const char *phase,
                                 const char *reason, bool paused = false,
                                 const aircannect::JobProgressPoint *point = nullptr,
                                 const char *file = nullptr) {
    if (!progress.active) return;
    progress.phase = phase;
    progress.reason = reason;
    publish_activity(progress.engine, phase, reason, file);
    auto event = point ? progress.watch.observe(millis(), *point, paused)
                       : progress.watch.check(millis(), paused);
    using Event = aircannect::JobProgressWatch::Event;
    if (event == Event::None) return;
    Log::logf(CAT_EXPORT, event == Event::Resumed ? LOG_INFO : LOG_WARN,
        "[%s] Sync %s task=%lu phase=%.20s wait=%.16s idle=%lus tries=%lu\n",
        engine_name(progress.engine), event == Event::Resumed ? "resumed" : "stalled",
        (unsigned long)progress.queued_ms,
        phase, reason, (unsigned long)(progress.watch.notice_ms() / 1000),
        (unsigned long)progress.watch.attempts());
}

static ExportProgress &begin_export_progress(const Request &request) {
    auto &progress = export_progress[size_t(request.engine)];
    const bool same = progress.active && progress.config_revision == request.config_revision &&
        progress.action == request.action && !strcmp(progress.from_day, request.from_day) &&
        !strcmp(progress.to_day, request.to_day) &&
        (request.kind == RequestKind::Retry ||
         (progress.generation == request.generation && progress.queued_ms == request.queued_ms));
    if (!same) {
        progress = {};
        progress.active = true;
        progress.engine = request.engine;
        progress.generation = request.generation;
        progress.config_revision = request.config_revision;
        progress.queued_ms = request.queued_ms;
        progress.action = request.action;
        memcpy(progress.from_day, request.from_day, sizeof(progress.from_day));
        memcpy(progress.to_day, request.to_day, sizeof(progress.to_day));
        progress.watch.check(millis(), true);
    }
    return progress;
}

static bool select_export_request(const Request &request, const EngineAdapter &engine,
                                   Request &selected) {
    if (request.kind == RequestKind::PostTherapy) {
        if (!(Config::get().*engine.enabled && Config::get().*engine.automatic)) return false;
    } else if (request.engine != engine.id) return false;
    selected = request;
    selected.engine = engine.id;
    if (request.kind == RequestKind::PostTherapy)
        selected.config_revision = Config::revision(engine.section);
    // An automatic trigger cannot replace a retained SleepHQ retry.
    return engine.id != Engine::SleepHq || selected.kind == RequestKind::Manual ||
        selected.kind == RequestKind::Retry ||
        sleephq_retry.config_revision != selected.config_revision ||
        (!sleephq_retry.pending && !sleephq_retry.blocked);
}

static void check_waiting_exports() {
    const bool standby = AirSenseState::device_standby();
    const bool local = SdStorage::local_access_allowed();
    const bool network = WiFi.status() == WL_CONNECTED;
    const bool readers = SdStorage::waiting_for_readers();
    for (auto &progress : export_progress) {
        if (!progress.active || !progress.queued) continue;
        if (progress.config_revision != Config::revision(engine_adapter(progress.engine)->section)) {
            progress.active = false;
            continue;
        }
        note_export_progress(progress, "admission",
            !standby ? "standby" : !local ? "usb" : !network ? "network" : readers ? "sd_reader_wait" : "export_queue",
            !standby || !local || !network || readers);
    }
}

static void initialize_export_progress(const Request &request) {
    for (size_t i = 0; i < size_t(Engine::Count); ++i) {
        const auto &engine = engines[i];
        Request selected;
        if (!select_export_request(request, engine, selected)) continue;
        begin_export_progress(selected).queued = true;
    }
    check_waiting_exports();
}

static void check_export_progress(RunContext &run, const char *wait_reason = nullptr,
                                  const SdStorage::IoEvent *io_event = nullptr) {
    check_waiting_exports();
    if (!run.progress) return;
    const bool standby = AirSenseState::device_standby();
    const bool local = SdStorage::local_access_allowed();
    const bool network = WiFi.status() == WL_CONNECTED;
    const bool actual_sd_wait = io_event && *io_event == SdStorage::IoEvent::Waiting;
    // A queued request's cause is more precise than another reader's wait or
    // the network state observed while this local SD operation is blocked.
    const bool readers = io_event ? *io_event == SdStorage::IoEvent::WaitingForReader : SdStorage::waiting_for_readers();
    aircannect::JobProgressPoint point;
    const auto *source = run.sleephq_progress;
    if (source) point = {run.progress_step + source->checkpoint_step, source->checkpoint_units};
    note_export_progress(*run.progress, source && source->phase ? source->phase : run.progress->phase,
        !standby ? "standby" : !local ? "usb" : wait_reason ? wait_reason :
        !network ? "network" : readers ? "sd_reader_wait" : run.storage_yielded ? "storage_busy" : "working",
        !standby || !local || readers || (!network && !actual_sd_wait), source ? &point : nullptr,
        source ? source->file : nullptr);
}

static void advance_export(RunContext &run, uint64_t units, const char *phase,
                            const char *file = nullptr) {
    if (!run.progress) return;
    const aircannect::JobProgressPoint point{run.progress_step, units};
    note_export_progress(*run.progress, phase, "working", false, &point, file);
}

static void publish_change() {
    __atomic_add_fetch(&status_revision, 1, __ATOMIC_RELEASE);
}

static void set_check(Engine id, uint32_t generation, CheckState state, uint32_t revision,
                      const char *error = "") {
    const auto &engine = *engine_adapter(id);
    portMUX_TYPE *mux = engine.mux;
    CheckStatus &check = *engine.check;
    portENTER_CRITICAL(mux);
    if (generation != __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE)) {
        portEXIT_CRITICAL(mux);
        return;
    }
    check.state = state;
    check.config_revision = revision;
    snprintf(check.error, sizeof(check.error), "%s", error);
    portEXIT_CRITICAL(mux);
    publish_change();
}

static bool generation_aborted(uint32_t generation) {
    return generation !=
               __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE) ||
           AirSenseState::rop() == 1;
}

static bool request_aborted(const Request &request) {
    if (generation_aborted(request.generation)) return true;
    if (request.kind == RequestKind::PostTherapy && request.engine == Engine::Count) {
        for (size_t i = 0; i < size_t(Engine::Count); ++i) {
            const auto &engine = engines[i];
            if (Config::get().*engine.enabled && Config::get().*engine.automatic)
                return false;
        }
        return true;
    }
    const auto *engine = engine_adapter(request.engine);
    return !engine || Config::revision(engine->section) != request.config_revision;
}

static void invalidate_backlog_entry(uint32_t index, uint8_t mask) {
    if (index < backlog_entry_count) backlog_entries[index].flags &= ~mask;
    request_backlog_refresh();
}

static void copy_text(char *dst, size_t capacity, const char *src) {
    if (!dst || !capacity) return;
    if (!src) src = "";
    strncpy(dst, src, capacity - 1);
    dst[capacity - 1] = 0;
}

static void publish_activity(Engine engine, const char *phase, const char *wait,
                             const char *file) {
    auto &activity = engine == Engine::Smb ? status.activity : sleephq_status.activity;
    auto *mux = engine == Engine::Smb ? &status_mux : &sleephq_status_mux;
    const bool file_phase = !strcmp(phase, "upload") || !strcmp(phase, "attach") ||
        !strcmp(phase, "hash") || !strcmp(phase, "root_inventory") ||
        !strcmp(phase, "file_inventory") || !strcmp(phase, "file_complete");
    portENTER_CRITICAL(mux);
    copy_text(activity.phase, sizeof(activity.phase), phase);
    copy_text(activity.wait, sizeof(activity.wait), wait);
    if (!file_phase) activity.file[0] = 0;
    else if (file) copy_text(activity.file, sizeof(activity.file), file);
    portEXIT_CRITICAL(mux);
    publish_change();
}

static void set_state(State state, const char *error = nullptr) {
    portENTER_CRITICAL(&status_mux);
    __atomic_store_n(&status.state, state, __ATOMIC_RELEASE);
    if (state == State::Pending) status.activity = {};
    if (error) copy_text(status.last_error, sizeof(status.last_error), error);
    else if (state != State::Error) status.last_error[0] = 0;
    portEXIT_CRITICAL(&status_mux);
    publish_change();
}

static void set_sleephq_state(State state, const char *error = nullptr) {
    portENTER_CRITICAL(&sleephq_status_mux);
    __atomic_store_n(&sleephq_status.state, state, __ATOMIC_RELEASE);
    if (state == State::Pending) sleephq_status.activity = {};
    if (error) {
        copy_text(sleephq_status.last_error,
                  sizeof(sleephq_status.last_error), error);
    } else if (state != State::Error) {
        sleephq_status.last_error[0] = 0;
    }
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();
}

static void settle_request(const Request &request) {
    for (size_t i = 0; i < size_t(Engine::Count); ++i) {
        const auto &engine = engines[i];
        if (request.kind != RequestKind::PostTherapy && request.engine != engine.id) continue;
        auto &progress = export_progress[i];
        if (progress.queued && progress.queued_ms == request.queued_ms &&
            progress.generation == request.generation) {
            progress.queued = false;
            progress.active = false;
        }
        portENTER_CRITICAL(engine.mux);
        if (*engine.state == State::Pending) *engine.state = State::Idle;
        portEXIT_CRITICAL(engine.mux);
    }
    publish_change();
}

static void publish_sleephq_progress(
    void *context, const SleepHqSync::Progress &progress) {
    portENTER_CRITICAL(&sleephq_status_mux);
    sleephq_status.files_seen = progress.files_seen;
    sleephq_status.files_uploaded = progress.files_uploaded;
    sleephq_status.files_skipped = progress.files_skipped;
    sleephq_status.files_attached = progress.files_attached;
    sleephq_status.bytes_uploaded = progress.bytes_uploaded;
    sleephq_status.import_id = progress.import_id;
    copy_text(sleephq_status.current_day,
              sizeof(sleephq_status.current_day), progress.current_day);
    copy_text(sleephq_status.import_status,
              sizeof(sleephq_status.import_status), progress.import_status);
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();
    auto *run = static_cast<RunContext *>(context);
    if (run && run->progress) {
        run->progress->phase = progress.phase ? progress.phase : "inventory";
        check_export_progress(*run);
    }
}

static bool run_interrupted(const RunContext &run) {
    return run.storage_yielded || generation_aborted(run.generation) ||
           Config::revision(run.section) != run.config_revision ||
           (run.network_only ? !AirSenseState::device_standby() : !storage.valid());
}

static bool run_aborted(void *context) {
    RunContext *run = static_cast<RunContext *>(context);
    if (!run) return true;
    const bool network_request = run->sleephq_progress && run->sleephq_progress->network_request;
    if (!run->network_only && !run->defer_storage_yield && !network_request && storage.yield_requested())
        run->storage_yielded = true;
    check_export_progress(*run);
    return run_interrupted(*run);
}

static void export_storage_wait(void *context, SdStorage::IoEvent event, const char *path) {
    if (event == SdStorage::IoEvent::Complete) return;
    auto &run = *static_cast<RunContext *>(context);
    check_export_progress(run, event == SdStorage::IoEvent::WaitingForReader ? "sd_reader_wait" :
        path ? "sd_write_wait" : "sd_io_wait", &event);
}

static bool resume_export_storage(const Request &request, RunContext &run) {
    if (!run.storage_yielded || !storage.valid() || request_aborted(request)) return false;
    storage.end();
    Log::logf(CAT_EXPORT, LOG_DEBUG, "[%s] Sync paused for storage\n", engine_name(request.engine));
    while (!request_aborted(request) && SdStorage::local_access_allowed()) {
        const bool ready = WiFi.status() == WL_CONNECTED && AirSenseState::device_standby();
        if (!ready) storage.end();
        check_export_progress(run);
        if (ready && storage.begin()) {
            run.storage_yielded = false;
            // Admission waiting is not a network operation's idle timeout.
            run.operation.note_progress(millis());
            check_export_progress(run);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

static bool yield_sleephq_storage(void *context) {
    auto &run = *static_cast<RunContext *>(context);
    run.storage_yielded = true;
    return run.request && resume_export_storage(*run.request, run);
}

struct SmbRequestScope {
    RunContext &run;
    bool previous;
    explicit SmbRequestScope(RunContext &context) : run(context), previous(run.defer_storage_yield) {
        run.defer_storage_yield = true;
    }
    ~SmbRequestScope() { run.defer_storage_yield = previous; }
};

struct SmbSource {
    const char *path;
    bool root;
    uint64_t size = 0, offset = 0;
    uint32_t files = SdStorage::files_revision(SdStorage::FileScope::Export), catalog = EdfCatalog::revision();
    SdStorage::Session token = storage;

    bool checkpoint(RunContext &run, SdStorage::Reader *input, bool &changed,
                    char *error, size_t error_size) {
        changed = false;
        if (!token.valid()) {
            copy_text(error, error_size, "source_changed");
            return false;
        }
        if (!run_aborted(&run)) return true;
        if (!run.storage_yielded || !storage.valid() || !run.request) {
            copy_text(error, error_size, "preempted");
            return false;
        }
        if (input) input->close();
        if (!resume_export_storage(*run.request, run)) {
            copy_text(error, error_size, "preempted");
            return false;
        }
        changed = files != SdStorage::files_revision(SdStorage::FileScope::Export) ||
            (root && catalog != EdfCatalog::revision());
        token = storage;
        if (!changed && input && (!input->open(storage, path) ||
            input->size() != size || !input->seek(offset))) {
            copy_text(error, error_size, "source_changed");
            return false;
        }
        return true;
    }
};

static bool wait_resolve(StorageSmbClient &client, RunContext &run,
                         char *error, size_t error_size) {
    advance_export(run, 0, "resolve");
    SmbRequestScope request_scope(run);
    const uint32_t started = millis();
    while (true) {
        const auto stop = run.operation.stop_reason(millis());
        if (stop != aircannect::BackgroundOperationStop::None) {
            copy_text(error, error_size, aircannect::background_operation_stop_error(stop));
            return false;
        }
        const StorageSmbOperationResult result =
            client.resolve_host(error, error_size);
        if (result == StorageSmbOperationResult::Ready) return true;
        if (result == StorageSmbOperationResult::Error) return false;
        if (static_cast<uint32_t>(millis() - started) >= DNS_TIMEOUT_MS) {
            copy_text(error, error_size, "dns_timeout");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool wait_connect(StorageSmbClient &client, RunContext &run,
                         char *error, size_t error_size) {
    advance_export(run, 0, "connect");
    SmbRequestScope request_scope(run);
    while (true) {
        const StorageSmbOperationResult result =
            client.step_connect(error, error_size, &run.operation);
        if (result == StorageSmbOperationResult::Ready) return true;
        if (result == StorageSmbOperationResult::Error) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool wait_ensure_directory(StorageSmbClient &client,
                                  const char *path, RunContext &run,
                                  char *error, size_t error_size) {
    SmbRequestScope request_scope(run);
    if (!path || !path[0]) return true;
    while (true) {
        const StorageSmbOperationResult result = client.step_ensure_directory(
            path, error, error_size, &run.operation);
        if (result == StorageSmbOperationResult::Ready) return true;
        if (result == StorageSmbOperationResult::Error) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool wait_stat(StorageSmbClient &client, const char *path,
                      StorageSmbRemoteStat &stat, RunContext &run,
                      char *error, size_t error_size) {
    SmbRequestScope request_scope(run);
    while (true) {
        const StorageSmbOperationResult result =
            client.step_stat(path, stat, error, error_size, &run.operation);
        if (result == StorageSmbOperationResult::Ready) return true;
        if (result == StorageSmbOperationResult::Error) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool wait_open(StorageSmbClient &client, const char *path,
                      RunContext &run, char *error, size_t error_size) {
    SmbRequestScope request_scope(run);
    while (true) {
        const StorageSmbOperationResult result = client.step_open_writer(
            path, error, error_size, &run.operation);
        if (result == StorageSmbOperationResult::Ready) return true;
        if (result == StorageSmbOperationResult::Error) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool wait_write(StorageSmbClient &client, const uint8_t *data,
                       size_t size, RunContext &run,
                       char *error, size_t error_size) {
    SmbRequestScope request_scope(run);
    while (true) {
        size_t written = 0;
        const StorageSmbOperationResult result = client.step_write(
            data, size, written, error, error_size, &run.operation);
        if (result == StorageSmbOperationResult::Ready)
            return written == size;
        if (result == StorageSmbOperationResult::Error) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool wait_close(StorageSmbClient &client, RunContext &run,
                       char *error, size_t error_size) {
    SmbRequestScope request_scope(run);
    while (true) {
        const StorageSmbOperationResult result = client.step_close_writer(
            error, error_size, &run.operation);
        if (result == StorageSmbOperationResult::Ready) return true;
        if (result == StorageSmbOperationResult::Error) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void disconnect(StorageSmbClient &client, RunContext &run) {
    // Cleanup may be interrupted too, but cannot change the transfer's outcome.
    RunContext cleanup = run;
    cleanup.operation.ctx = &cleanup;
    for (uint16_t i = 0; i < 200; i++) {
        const StorageSmbOperationResult result =
            client.step_disconnect(&cleanup.operation);
        if (result != StorageSmbOperationResult::Waiting) return;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    client.abort_connection();
}

static void note_seen() {
    portENTER_CRITICAL(&status_mux);
    status.files_seen++;
    portEXIT_CRITICAL(&status_mux);
    publish_change();
}

static void note_skipped() {
    portENTER_CRITICAL(&status_mux);
    status.files_skipped++;
    portEXIT_CRITICAL(&status_mux);
    publish_change();
}

static void note_uploaded(uint64_t bytes) {
    portENTER_CRITICAL(&status_mux);
    status.files_uploaded++;
    status.bytes_uploaded += bytes;
    portEXIT_CRITICAL(&status_mux);
    publish_change();
}

static uint32_t endpoint_hash(const SmbConfig &config) {
    uint32_t crc = crc32_ieee_initial();
    crc = crc32_ieee_update(crc,
        reinterpret_cast<const uint8_t *>(config.endpoint),
        strlen(config.endpoint));
    const uint8_t newline = '\n';
    crc = crc32_ieee_update(crc, &newline, 1);
    crc = crc32_ieee_update(crc,
        reinterpret_cast<const uint8_t *>(config.user), strlen(config.user));
    return crc32_ieee_finish(crc);
}

static bool state_marker_path(const SmbConfig &config,
                              const char *name,
                              char *directory, size_t directory_size,
                              char *path, size_t path_size) {
    const uint32_t hash = endpoint_hash(config);
    const int dir_len = snprintf(directory, directory_size,
                                 "/airbridge/smb_%08lx",
                                 static_cast<unsigned long>(hash));
    const int path_len = snprintf(path, path_size, "%s/%s.done",
                                  directory, name);
    return dir_len > 0 && static_cast<size_t>(dir_len) < directory_size &&
           path_len > 0 && static_cast<size_t>(path_len) < path_size;
}

static bool write_smb_marker(const char *directory, const char *path,
                              const uint8_t *marker, size_t marker_size) {
    char partial[88];
    if (snprintf(partial, sizeof(partial), "%s.part", path) >=
        static_cast<int>(sizeof(partial))) return false;
    return storage.mutate(directory, [&](fs::FS &fs) {
        if (!fs.exists(directory) && !fs.mkdir(directory)) return false;
        fs.remove(partial);
        fs::File file = fs.open(partial, FILE_WRITE);
        const bool written = file && file.write(marker, marker_size) == marker_size;
        if (file) {
            file.flush();
            file.close();
        }
        if (!written) {
            fs.remove(partial);
            return false;
        }
        if (fs.exists(path)) fs.remove(path);
        if (!fs.rename(partial, path)) {
            fs.remove(partial);
            return false;
        }
        return true;
    });
}

static bool write_state_marker(const SmbConfig &config,
                               const EdfCatalog::Entry &entry) {
    char directory[48];
    char path[80];
    if (!state_marker_path(config, entry.file_prefix, directory, sizeof(directory),
                           path, sizeof(path))) return false;
    uint8_t marker[12] = {'A', 'B', 'S', 'M', 1, entry.flags, 0, 0};
    SdStorage::put_le32(marker + 8, entry.finalized_epoch);
    return write_smb_marker(directory, path, marker, sizeof(marker));
}

static bool state_marker_exists(const SmbConfig &config,
                                const EdfCatalog::Entry &entry) {
    char directory[48];
    char path[80];
    if (!state_marker_path(config, entry.file_prefix, directory, sizeof(directory),
                           path, sizeof(path))) return false;
    SdStorage::Reader file;
    if (!file.open(storage, path)) return false;
    uint8_t marker[12];
    const bool valid = file && file.size() == sizeof(marker) &&
        file.read(marker, sizeof(marker)) == sizeof(marker) &&
        memcmp(marker, "ABSM", 4) == 0 && marker[4] == 1 &&
        marker[5] == entry.flags &&
        (static_cast<uint32_t>(marker[8]) |
         static_cast<uint32_t>(marker[9]) << 8 |
         static_cast<uint32_t>(marker[10]) << 16 |
         static_cast<uint32_t>(marker[11]) << 24) == entry.finalized_epoch;
    if (file) file.close();
    return valid;
}

struct RootFileState {
    bool exists = false;
    bool confirmed = false;
    char directory[48];
    char path[80];
    uint8_t marker[20] = {'A', 'B', 'S', 'R', 1};
    uint32_t files_revision = 0, catalog_revision = 0;
    SdStorage::Session source;
};

static bool read_root_state(const SmbConfig &config, const char *path,
                             uint8_t *buffer, size_t buffer_size,
                             RootFileState &out, char *error, size_t error_size,
                             RunContext &run) {
    advance_export(run, 0, "root_inventory", path);
    for (;;) {
        SmbSource source{path, true};
        out.confirmed = false;
        if (!state_marker_path(config, path + 1, out.directory, sizeof(out.directory),
                               out.path, sizeof(out.path))) return false;
        if (!storage.run([&](fs::FS &fs) { out.exists = fs.exists(path); return true; }))
            return false;
        if (!out.exists) return true;

        SdStorage::Reader input;
        if (!input.open(storage, path)) {
            snprintf(error, error_size, "local_open:%s", path);
            return false;
        }
        const uint64_t size = input.size();
        source.size = size;
        uint32_t crc = crc32_ieee_initial();
        bool changed = false;
        for (uint64_t remaining = size; remaining;) {
            if (!source.checkpoint(run, &input, changed, error, error_size)) return false;
            if (changed) break;
            const size_t wanted = remaining < buffer_size ? remaining : buffer_size;
            if (input.read(buffer, wanted) != wanted) {
                copy_text(error, error_size, "local_read_short");
                return false;
            }
            crc = crc32_ieee_update(crc, buffer, wanted);
            remaining -= wanted;
            source.offset += wanted;
            advance_export(run, size - remaining, "root_inventory", path);
        }
        input.close();
        if (changed) continue;
        SdStorage::put_le32(out.marker + 8, static_cast<uint32_t>(size));
        SdStorage::put_le32(out.marker + 12, static_cast<uint32_t>(size >> 32));
        SdStorage::put_le32(out.marker + 16, crc32_ieee_finish(crc));

        SdStorage::Reader marker;
        if (marker.open(storage, out.path)) {
            uint8_t saved[sizeof(out.marker)];
            out.confirmed = marker.size() == sizeof(saved) &&
                marker.read(saved, sizeof(saved)) == sizeof(saved) &&
                memcmp(saved, out.marker, sizeof(saved)) == 0;
        }
        out.files_revision = source.files;
        out.catalog_revision = source.catalog;
        out.source = source.token;
        return source.token.valid();
    }
}

static bool sync_smb_file(StorageSmbClient &client, const char *local_path,
                          const RootFileState *root, uint8_t *buffer,
                          size_t buffer_size, RunContext &run,
                          char *error, size_t error_size, const SmbConfig *config = nullptr) {
    note_seen();
    if (root && !root->source.valid()) {
        copy_text(error, error_size, "source_changed");
        return false;
    }
    RootFileState refreshed;
    for (;;) {
        SmbSource source{local_path, root != nullptr};
        bool changed = false;
        if (!source.checkpoint(run, nullptr, changed, error, error_size)) return false;
        if (root && (root->files_revision != SdStorage::files_revision(SdStorage::FileScope::Export) ||
                     root->catalog_revision != EdfCatalog::revision())) {
            if (!config) { copy_text(error, error_size, "source_changed"); return false; }
            if (!read_root_state(*config, local_path, buffer, buffer_size, refreshed,
                                 error, error_size, run)) return false;
            root = &refreshed;
            if (!root->exists) return true;
        }
        source = SmbSource{local_path, root != nullptr};
        advance_export(run, 0, "file_inventory", local_path);
        SdStorage::Reader input;
        if (!input.open(storage, local_path)) {
            snprintf(error, error_size, "local_open:%s", local_path);
            return false;
        }
        const uint64_t local_size = input.size();
        source.size = local_size;
        char remote[aircannect::AC_STORAGE_SMB_REMOTE_PATH_MAX];
        if (!client.make_remote_path(local_path, remote, sizeof(remote))) {
            input.close();
            copy_text(error, error_size, "remote_path_too_long");
            return false;
        }

        StorageSmbRemoteStat remote_stat;
        if (!wait_stat(client, remote, remote_stat, run, error, error_size)) {
            input.close();
            return false;
        }
        if ((!root || root->confirmed) && remote_stat.exists && !remote_stat.directory &&
            remote_stat.size == local_size) {
            input.close();
            note_skipped();
            advance_export(run, UINT64_MAX, "file_complete");
            return true;
        }

        char parent[aircannect::AC_STORAGE_SMB_REMOTE_PATH_MAX];
        copy_text(parent, sizeof(parent), remote);
        char *slash = strrchr(parent, '/');
        if (slash) {
            *slash = 0;
            if (!wait_ensure_directory(client, parent, run, error, error_size)) {
                input.close();
                return false;
            }
        }
        // A failed overwrite must not leave confirmation of the previous contents.
        if (root) roots_cached = false;
        if (root && !storage.mutate(root->path, [&](fs::FS &fs) {
            return !fs.exists(root->path) || fs.remove(root->path);
        })) {
            copy_text(error, error_size, "state_marker_remove");
            return false;
        }
        advance_export(run, 0, "upload", local_path);
        if (!wait_open(client, remote, run, error, error_size)) {
            input.close();
            return false;
        }

        uint64_t sent = 0;
        bool success = true;
        while (sent < local_size) {
            if (!source.checkpoint(run, &input, changed, error, error_size)) {
                success = false;
                break;
            }
            if (changed) break;
            const size_t remaining = static_cast<size_t>(local_size - sent);
            const size_t wanted = min(buffer_size, remaining);
            const size_t read = input.read(buffer, wanted);
            if (read != wanted ||
                !wait_write(client, buffer, read, run, error, error_size)) {
                if (read != wanted) copy_text(error, error_size, "local_read_short");
                success = false;
                break;
            }
            sent += read;
            source.offset = sent;
            run.operation.note_progress(millis());
            advance_export(run, sent, "upload", local_path);
            taskYIELD();
        }
        input.close();
        if (changed && success) {
            if (!wait_close(client, run, error, error_size)) return false;
            continue;
        }
        if (success) success = wait_close(client, run, error, error_size);
        if (!success) {
            client.abort_connection();
            return false;
        }

        if (!wait_stat(client, remote, remote_stat, run, error, error_size) ||
            !remote_stat.exists || remote_stat.directory ||
            remote_stat.size != local_size) {
            copy_text(error, error_size, "remote_verify_failed");
            return false;
        }
        if (!source.token.valid()) {
            copy_text(error, error_size, "source_changed");
            return false;
        }
        note_uploaded(sent);
        if (root && !write_smb_marker(root->directory, root->path,
                                      root->marker, sizeof(root->marker))) {
            copy_text(error, error_size, "state_marker_write");
            return false;
        }
        Log::logf(CAT_EXPORT, LOG_DEBUG,
                  "[SMB] uploaded %s bytes=%llu\n", local_path,
                  static_cast<unsigned long long>(sent));
        advance_export(run, UINT64_MAX, "file_complete");
        return true;
    }
}

static bool sync_session_files(StorageSmbClient &client,
                               const EdfCatalog::Entry &entry,
                               uint8_t *buffer, size_t buffer_size,
                               RunContext &run,
                               char *error, size_t error_size) {
    char path[128];
    for (const char *suffix : SESSION_SUFFIXES) {
        ++run.progress_step;
        snprintf(path, sizeof(path), "/DATALOG/%s/%s_%s.edf",
                 entry.therapy_day, entry.file_prefix, suffix);
        if (!sync_smb_file(client, path, nullptr, buffer, buffer_size,
                           run, error, error_size)) return false;
        snprintf(path, sizeof(path), "/DATALOG/%s/%s_%s.crc",
                 entry.therapy_day, entry.file_prefix, suffix);
        ++run.progress_step;
        if (!sync_smb_file(client, path, nullptr, buffer, buffer_size,
                           run, error, error_size)) return false;
    }
    return true;
}

static bool sync_root_files(StorageSmbClient &client, const SmbConfig &config,
                            uint8_t *buffer, size_t buffer_size,
                            RunContext &run,
                            char *error, size_t error_size) {
    for (const char *path : ROOT_FILES) {
        ++run.progress_step;
        RootFileState state;
        if (!read_root_state(config, path, buffer, buffer_size, state, error, error_size, run))
            return false;
        ++run.progress_step;
        if (!state.exists) continue;
        if (!sync_smb_file(client, path, &state, buffer, buffer_size,
                           run, error, error_size, &config)) return false;
    }
    return true;
}

static bool snapshot_smb_config(SmbConfig &out,
                                char *error, size_t error_size, bool require_enabled = true) {
    const AirBridgeConfig &config = Config::get();
    if (require_enabled && !config.smb_enabled) return false;
    if (config.smb_endpoint.isEmpty()) {
        copy_text(error, error_size, "endpoint_missing");
        return false;
    }
    if (config.smb_endpoint.length() >= sizeof(out.endpoint) ||
        config.smb_user.length() >= sizeof(out.user) ||
        config.smb_password.length() >= sizeof(out.password)) {
        copy_text(error, error_size, "configuration_too_long");
        return false;
    }
    copy_text(out.endpoint, sizeof(out.endpoint), config.smb_endpoint.c_str());
    copy_text(out.user, sizeof(out.user), config.smb_user.c_str());
    copy_text(out.password, sizeof(out.password), config.smb_password.c_str());
    return true;
}

template <typename T>
static void capture_run_outcome(RunOutcome &finished, const T &result,
                                bool success, uint32_t epoch) {
    finished = {};
    finished.available = true;
    finished.success = success;
    finished.finished_epoch = epoch;
    finished.files_uploaded = result.files_uploaded;
    finished.files_skipped = result.files_skipped;
    finished.bytes_uploaded = result.bytes_uploaded;
    if (!success) copy_text(finished.error, sizeof(finished.error), result.last_error);
    if constexpr (std::is_same<T, SleepHqStatus>::value) {
        finished.files_attached = result.files_attached;
        finished.import_id = result.import_id;
        copy_text(finished.import_status, sizeof(finished.import_status), result.import_status);
    }
}

static Backlog completed_backlog(const RunContext &run, const EdfCatalog::Status &catalog,
                                 uint32_t files_revision, uint32_t waiting_sessions) {
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (!current.ready || current.generation != catalog.generation ||
        SdStorage::files_revision(SdStorage::FileScope::Export) != files_revision || run_interrupted(run)) return {};

    Backlog completed = {};
    completed.known = true;
    completed.waiting_sessions = waiting_sessions;
    completed.config_revision = run.config_revision;
    completed.catalog_generation = catalog.generation;
    completed.files_revision = files_revision;
    // The next refresh must read the completed markers, not prior queue counts.
    const uint8_t mask = run.section == Config::Section::Smb ? SMB_VALID : SHQ_VALID;
    for (uint32_t i = 0; i < backlog_entry_count; ++i)
        backlog_entries[i].flags &= ~mask;
    return completed;
}

static bool run_smb(const Request &request, RunContext &run) {
    portENTER_CRITICAL(&status_mux);
    status.files_seen = 0;
    status.files_uploaded = 0;
    status.files_skipped = 0;
    status.bytes_uploaded = 0;
    copy_text(status.current_day, sizeof(status.current_day),
              request.kind == RequestKind::PostTherapy
                  ? request.entry.therapy_day : "");
    portEXIT_CRITICAL(&status_mux);
    publish_change();

    char error[64] = {};
    SmbConfig config = {};
    if (!snapshot_smb_config(config, error, sizeof(error))) {
        set_state(error[0] ? State::Error : State::Disabled,
                  error[0] ? error : nullptr);
        return !error[0];
    }
    set_state(State::Working);
    advance_export(run, 0, "catalog");

    const uint32_t files_revision = SdStorage::files_revision(SdStorage::FileScope::Export);
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    if (!catalog.ready) {
        set_state(State::Error, "catalog_not_ready");
        return false;
    }

    size_t buffer_size = UPLOAD_BUFFER_PSRAM;
    uint8_t *buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(buffer_size, false));
    if (!buffer) {
        buffer_size = UPLOAD_BUFFER_FALLBACK;
        buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(buffer_size));
    }
    if (!buffer) {
        set_state(State::Error, "upload_buffer_alloc");
        return false;
    }

    run.operation.started_ms = millis();
    run.operation.last_progress_ms = run.operation.started_ms;
    run.operation.timeout_ms = OPERATION_IDLE_TIMEOUT_MS;
    run.operation.timeout_policy = BackgroundOperationTimeoutPolicy::NoProgress;
    run.operation.should_abort = run_aborted;
    run.operation.ctx = &run;

    StorageSmbClient client;
    bool success = client.configure(config.endpoint, config.user,
                                    config.password, error, sizeof(error)) &&
                   wait_resolve(client, run, error, sizeof(error)) &&
                   wait_connect(client, run, error, sizeof(error));
    if (success) run.operation.note_progress(millis());

    uint32_t waiting_sessions = 0;
    if (success) {
        for (uint32_t i = catalog.entries; success && i > 0; i--) {
            SmbSource checkpoint{nullptr, false};
            bool changed = false;
            if (!checkpoint.checkpoint(run, nullptr, changed, error, sizeof(error))) {
                success = false;
                break;
            }
            EdfCatalog::Entry entry;
            run.progress_step = uint64_t(catalog.entries - i + 1) << 32;
            if (!storage.run([&](fs::FS &) { return EdfCatalog::read(i - 1, entry); })) {
                copy_text(error, sizeof(error), "catalog_read");
                success = false;
                break;
            }
            advance_export(run, 0, "catalog");
            if (!entry.recording_ready()) {
                ++waiting_sessions;
                continue;
            }
            if (state_marker_exists(config, entry)) continue;
            invalidate_backlog_entry(i - 1, SMB_VALID);
            portENTER_CRITICAL(&status_mux);
            copy_text(status.current_day, sizeof(status.current_day),
                      entry.therapy_day);
            portEXIT_CRITICAL(&status_mux);
            publish_change();
            success = sync_session_files(client, entry, buffer, buffer_size,
                                         run, error, sizeof(error));
            if (success && !write_state_marker(config, entry)) {
                copy_text(error, sizeof(error), "state_marker_write");
                success = false;
            }
        }
        if (success) {
            run.progress_step = uint64_t(catalog.entries + 1) << 32;
            success = sync_root_files(client, config, buffer, buffer_size, run,
                                      error, sizeof(error));
        }
    }

    if (client.connected()) disconnect(client, run);
    else client.abort_connection();
    heap_caps_free(buffer);
    if (!success || run_interrupted(run)) {
        if (run_interrupted(run)) set_state(State::Idle);
        else set_state(State::Error,
                       error[0] ? error : "smb_sync_failed");
        return false;
    }

    const Backlog completed = completed_backlog(run, catalog, files_revision, waiting_sessions);
    const uint32_t epoch = static_cast<uint32_t>(time(nullptr));
    portENTER_CRITICAL(&status_mux);
    status.last_sync_epoch = epoch;
    status.current_day[0] = 0;
    status.backlog = completed;
    capture_run_outcome(status.last_run, status, true, epoch);
    __atomic_store_n(&status.state, State::Idle, __ATOMIC_RELEASE);
    status.last_error[0] = 0;
    portEXIT_CRITICAL(&status_mux);
    roots_pending = 0;
    roots_cached = completed.known;
    roots_endpoint = endpoint_hash(config);
    roots_catalog_generation = catalog.generation;
    publish_change();
    return true;
}

static bool snapshot_sleephq_config(SleepHqSync::Config &out,
                                    char *error, size_t error_size, bool require_enabled = true) {
    const AirBridgeConfig &config = Config::get();
    if (require_enabled && !config.sleephq_enabled) return false;
    if (config.sleephq_client_id.isEmpty() ||
        config.sleephq_client_secret.isEmpty()) {
        copy_text(error, error_size, "credentials_missing");
        return false;
    }
    if (config.sleephq_client_id.length() >= sizeof(out.client_id) ||
        config.sleephq_client_secret.length() >= sizeof(out.client_secret) ||
        config.sleephq_team_id.length() >= sizeof(out.team_id) ||
        config.sleephq_device_id.length() >= sizeof(out.device_id)) {
        copy_text(error, error_size, "configuration_too_long");
        return false;
    }
    copy_text(out.client_id, sizeof(out.client_id),
              config.sleephq_client_id.c_str());
    copy_text(out.client_secret, sizeof(out.client_secret),
              config.sleephq_client_secret.c_str());
    copy_text(out.team_id, sizeof(out.team_id),
              config.sleephq_team_id.c_str());
    copy_text(out.device_id, sizeof(out.device_id),
              config.sleephq_device_id.c_str());
    return true;
}

static uint32_t sleephq_retry_delay(const char *error, uint8_t attempt,
                                   uint8_t heap_attempt) {
    bool retryable = false;
    if (strncmp(error, "http_", 5) == 0) {
        unsigned code = 0;
        char extra = 0;
        retryable = sscanf(error, "http_%u%c", &code, &extra) == 1 &&
            (code == 404 || code == 408 || code == 429 || (code >= 500 && code <= 599));
    } else {
        static const char *const transient[] = {
            "connect_failed", "write_failed", "response_header_failed",
            "response_incomplete", "operation_timeout", "import_process_timeout",
            "import_missing", "import_closed_before_upload", "tls_heap_guard",
            "http_client_alloc_failed", "request_alloc_failed", "token_alloc_failed",
            "request_header_alloc_failed", "response_alloc_failed", "upload_buffer_alloc",
            "catalog_not_ready", "catalog_read", "storage_unavailable", "state_directory",
            "local_open", "local_read_short", "inflight_write", "inflight_remove",
            "done_marker_write", "source_changed", "source_read_failed", "day_not_ready",
            "remote_inventory_changed", "remote_inventory_alloc", "hash_buffer_alloc",
        };
        for (const char *value : transient)
            if (strcmp(error, value) == 0) { retryable = true; break; }
    }
    if (!retryable) return 0;
    // Brief heap pressure gets three short retries, then normal backoff.
    if (strcmp(error, "tls_heap_guard") == 0 && heap_attempt < 3) return 10000;
    constexpr uint32_t delays[] = {15 * 60 * 1000UL, 60 * 60 * 1000UL, 6 * 60 * 60 * 1000UL};
    return delays[attempt < 3 ? attempt : 2];
}

static void publish_sleephq_retry(uint32_t delay_ms = 0) {
    const uint32_t retry_at = delay_ms
        ? static_cast<uint32_t>(time(nullptr)) + delay_ms / 1000 : 0;
    portENTER_CRITICAL(&sleephq_status_mux);
    sleephq_status.retry_pending = delay_ms != 0;
    sleephq_status.retry_at_epoch = retry_at;
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();
}

static void reset_sleephq_retry(uint32_t revision) {
    sleephq_retry = {};
    sleephq_retry.config_revision = revision;
    publish_sleephq_retry();
}

static void sleephq_failed(const char *error, const SleepHqSync::Progress &progress = {},
                           const char *request = "") {
    const uint32_t delay_ms = sleephq_retry_delay(
        error, sleephq_retry.attempt, sleephq_retry.heap_attempt);
    if (delay_ms == 10000) ++sleephq_retry.heap_attempt;
    else if (delay_ms && sleephq_retry.attempt < UINT8_MAX) ++sleephq_retry.attempt;
    sleephq_retry.pending = delay_ms != 0;
    sleephq_retry.blocked = !delay_ms;
    sleephq_retry.due_ms = millis() + delay_ms;
    auto &diagnostics = export_progress[size_t(Engine::SleepHq)];
    if (diagnostics.active)
        note_export_progress(diagnostics, progress.phase ? progress.phase : "admission",
                             delay_ms ? "retry_backoff" : "failed", delay_ms != 0);
    set_sleephq_state(State::Error, error);
    publish_sleephq_retry(delay_ms);
    Log::logf(CAT_EXPORT, delay_ms ? LOG_WARN : LOG_ERROR,
              "[SLEEPHQ] Sync failed: %s phase=%s import=%lu retry=%lus request=%s file=%s\n",
              error, progress.phase ? progress.phase : "admission",
              (unsigned long)progress.import_id, (unsigned long)(delay_ms / 1000),
              request[0] ? request : "-", progress.file[0] ? progress.file : "-");
}

static bool run_sleephq(const Request &request, RunContext &run) {
    portENTER_CRITICAL(&sleephq_status_mux);
    sleephq_status.files_seen = 0;
    sleephq_status.files_uploaded = 0;
    sleephq_status.files_skipped = 0;
    sleephq_status.files_attached = 0;
    sleephq_status.bytes_uploaded = 0;
    sleephq_status.import_id = 0;
    sleephq_status.import_status[0] = 0;
    copy_text(sleephq_status.current_day,
              sizeof(sleephq_status.current_day),
              request.kind == RequestKind::PostTherapy
                  ? request.entry.therapy_day : "");
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();

    const uint32_t config_revision = run.config_revision;
    if (request.kind == RequestKind::Manual ||
        sleephq_retry.config_revision != config_revision)
        reset_sleephq_retry(config_revision);
    sleephq_retry.pending = false;
    sleephq_retry.action = request.action;
    copy_text(sleephq_retry.from_day, sizeof(sleephq_retry.from_day), request.from_day);
    copy_text(sleephq_retry.to_day, sizeof(sleephq_retry.to_day), request.to_day);
    publish_sleephq_retry();
    char error[96] = {};
    SleepHqSync::Config config = {};
    if (!snapshot_sleephq_config(config, error, sizeof(error))) {
        if (error[0]) sleephq_failed(error);
        else set_sleephq_state(State::Disabled);
        return !error[0];
    }
    if (!aircannect::TlsMemory::status().installed) {
        sleephq_failed("tls_allocator_install");
        return false;
    }

    set_sleephq_state(State::Working);
    advance_export(run, 0, "catalog");

    const uint32_t files_revision = SdStorage::files_revision(SdStorage::FileScope::Export);
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    if (!catalog.ready) {
        sleephq_failed("catalog_not_ready");
        return false;
    }

    run.operation.started_ms = millis();
    run.operation.last_progress_ms = run.operation.started_ms;
    run.operation.timeout_ms = SLEEPHQ_IDLE_TIMEOUT_MS;
    run.operation.timeout_policy = BackgroundOperationTimeoutPolicy::NoProgress;
    run.operation.should_abort = run_aborted;
    run.operation.ctx = &run;

    SleepHqSync::Progress progress = {};
    run.sleephq_progress = &progress;
    SleepHqSync::Run sync(config, yield_sleephq_storage, &run);
    SleepHqSync::BacklogChanges changes = {};
    auto step = [&](auto &&action) {
        for (;;) {
            if (action()) return true;
            if (sync.refresh_pending() && !run.storage_yielded &&
                storage.valid() && !request_aborted(request)) {
                error[0] = 0;
                continue;
            }
            if (!run.storage_yielded || !sync.pause_for_yield(storage) ||
                !resume_export_storage(request, run)) return false;
            error[0] = 0;
        }
    };
    bool success = request.action == Action::Repair || step([&] {
        return sync.resume(storage, run.operation, progress,
            publish_sleephq_progress, &run, error, sizeof(error), &changes);
    });
    bool backlog_changed = changes.other_entries;
    aircannect::LargeTextBuffer visited(65536, 4096);
    uint32_t selected_days = 0, waiting_sessions = 0;
    for (uint32_t i = catalog.entries; success && i > 0; i--) {
        if (!step([&] { return !run_aborted(&run); })) {
            success = false;
            break;
        }
        EdfCatalog::Entry entry;
        if (!storage.run([&](fs::FS &) { return EdfCatalog::read(i - 1, entry); })) {
            copy_text(error, sizeof(error), "catalog_read");
            success = false;
            break;
        }
        if (!entry.recording_ready()) {
            ++waiting_sessions;
            continue;
        }
        if (request.from_day[0] && (strcmp(entry.therapy_day, request.from_day) < 0 ||
                                   strcmp(entry.therapy_day, request.to_day) > 0)) continue;
        bool seen = false;
        for (size_t offset = 0; offset < visited.length(); offset += 9)
            if (!memcmp(visited.c_str() + offset, entry.therapy_day, 8)) { seen = true; break; }
        if (seen) continue;
        if (!visited.append(entry.therapy_day, 9)) {
            copy_text(error, sizeof(error), "day_inventory_alloc");
            success = false;
            break;
        }
        ++selected_days;
        run.progress_step = uint64_t(catalog.entries - i + 1) << 32;
        progress.checkpoint_step = 0;
        progress.checkpoint_units = 0;
        progress.phase = "day_inventory";
        advance_export(run, 0, "day_inventory");
        success = step([&] {
            const bool ok = sync.sync_day(
                storage, entry.therapy_day, request.action == Action::Repair, run.operation, progress,
                publish_sleephq_progress, &run, error, sizeof(error), &changes);
            backlog_changed |= changes.other_entries;
            return ok;
        });
        backlog_changed |= changes.other_entries;
    }
    if (backlog_changed) {
        for (uint32_t n = 0; n < backlog_entry_count; ++n)
            backlog_entries[n].flags &= ~SHQ_VALID;
        portENTER_CRITICAL(&sleephq_status_mux);
        sleephq_status.backlog = {};
        portEXIT_CRITICAL(&sleephq_status_mux);
        request_backlog_refresh();
        publish_change();
    }

    if (success && request.action == Action::Repair && !selected_days) {
        copy_text(error, sizeof(error), "day_not_found");
        success = false;
    }

    if (!success || run_interrupted(run)) {
        if (run_interrupted(run)) set_sleephq_state(State::Idle);
        else sleephq_failed(error[0] ? error : "sleephq_sync_failed",
                            progress, sync.last_request());
        run.sleephq_progress = nullptr;
        return false;
    }

    // A range repair proves only its selected days, never the global queue.
    const Backlog completed = request.from_day[0] ? Backlog{} :
        completed_backlog(run, catalog, files_revision, waiting_sessions);
    const uint32_t epoch = static_cast<uint32_t>(time(nullptr));
    portENTER_CRITICAL(&sleephq_status_mux);
    sleephq_status.last_sync_epoch = epoch;
    sleephq_status.current_day[0] = 0;
    if (!request.from_day[0]) sleephq_status.backlog = completed;
    capture_run_outcome(sleephq_status.last_run, sleephq_status, true, epoch);
    __atomic_store_n(&sleephq_status.state, State::Idle, __ATOMIC_RELEASE);
    sleephq_status.last_error[0] = 0;
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();
    reset_sleephq_retry(config_revision);
    run.sleephq_progress = nullptr;
    return true;
}

static bool check_smb(RunContext &run, char *error, size_t error_size) {
    SmbConfig config = {};
    StorageSmbClient client;
    char remote[aircannect::AC_STORAGE_SMB_REMOTE_PATH_MAX];
    StorageSmbRemoteStat stat;
    bool success = snapshot_smb_config(config, error, error_size, false) &&
        client.configure(config.endpoint, config.user, config.password, error, error_size) &&
        wait_resolve(client, run, error, error_size) &&
        wait_connect(client, run, error, error_size) &&
        client.make_remote_path("/", remote, sizeof(remote)) &&
        wait_stat(client, remote, stat, run, error, error_size);
    if (success && (!stat.exists || !stat.directory)) {
        success = false;
        copy_text(error, error_size, "directory_missing");
    }
    if (client.connected()) disconnect(client, run);
    else client.abort_connection();
    return success;
}

static bool check_sleephq(RunContext &run, char *error, size_t error_size) {
    SleepHqSync::Config config = {};
    if (!aircannect::TlsMemory::status().installed) {
        copy_text(error, error_size, "tls_allocator_install");
        return false;
    }
    return snapshot_sleephq_config(config, error, error_size, false) &&
        SleepHqSync::check_account(config, run.operation, error, error_size);
}

static void run_check(const Request &request) {
    const auto &engine = *engine_adapter(request.engine);
    char error[96] = {};
    bool success = false;
    set_check(request.engine, request.generation, CheckState::Working, request.config_revision);
    RunContext run = {};
    run.generation = request.generation;
    run.network_only = true;
    run.config_revision = request.config_revision;
    run.section = engine.section;
    run.operation.started_ms = request.queued_ms;
    run.operation.timeout_ms = OPERATION_IDLE_TIMEOUT_MS;
    run.operation.should_abort = run_aborted;
    run.operation.ctx = &run;

    if (run.operation.stop_reason(millis()) != aircannect::BackgroundOperationStop::None) {
        copy_text(error, sizeof(error), "preempted_or_timed_out");
    } else if (Config::revision(run.section) != request.config_revision) {
        copy_text(error, sizeof(error), "configuration_changed");
    } else if (WiFi.status() != WL_CONNECTED) {
        copy_text(error, sizeof(error), "wifi_disconnected");
    } else if (!AirSenseState::device_standby()) {
        copy_text(error, sizeof(error), "standby_required");
    } else {
        success = engine.check_connection(run, error, sizeof(error));
    }
    if (request.generation != __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE)) return;
    if (run_aborted(&run)) {
        success = false;
        copy_text(error, sizeof(error), "preempted");
    }
    set_check(request.engine, request.generation, success ? CheckState::Passed : CheckState::Failed,
              request.config_revision, success ? "" : error[0] ? error : "check_failed");
    __atomic_store_n(&check_pending, false, __ATOMIC_RELEASE);
    Log::logf(CAT_EXPORT, success ? LOG_INFO : LOG_ERROR, "[%s] Check %s%s%s\n",
              engine_name(request.engine), success ? "passed" : "failed",
              error[0] ? ": " : "", error);
}

static void log_backlog_result(const char *destination, const Backlog &previous,
                               const Backlog &current) {
    const auto failed = [](const Backlog &value) {
        return value.error[0] &&
            strcmp(value.error, "endpoint_missing") && strcmp(value.error, "credentials_missing");
    };
    if (failed(current) && (!failed(previous) || strcmp(previous.error, current.error)))
        Log::logf(CAT_EXPORT, LOG_WARN, "[%s] Backlog failed generation=%lu: %s\n",
            destination, (unsigned long)current.catalog_generation, current.error);
    else if (current.known && !failed(current) && failed(previous))
        Log::logf(CAT_EXPORT, LOG_INFO, "[%s] Backlog recovered files=%lu\n",
            destination, (unsigned long)current.files);
}

static void retain_backlog_counts(Backlog &current, const Backlog &previous) {
    if (current.known || !previous.known ||
        current.files_revision != previous.files_revision ||
        current.catalog_generation != previous.catalog_generation ||
        current.config_revision != previous.config_revision) return;

    // A failed redundant scan does not refute the owner's same-source proof.
    // Keep the scan error independent of the still-known queue counts.
    current.known = true;
    current.files = previous.files;
    current.sessions = previous.sessions;
    current.waiting_sessions = previous.waiting_sessions;
}

static void refresh_backlog() {
    const uint32_t files_revision = SdStorage::files_revision(SdStorage::FileScope::Export);
    EdfCatalog::Changes changes;
    const bool incremental = EdfCatalog::changes_since(backlog_catalog_generation, changes);
    const auto &catalog = changes.catalog;
    const uint32_t smb_revision = Config::revision(Config::Section::Smb);
    const uint32_t shq_revision = Config::revision(Config::Section::SleepHq);
    if (!__atomic_load_n(&backlog_refresh, __ATOMIC_ACQUIRE) &&
        catalog.generation == backlog_catalog_generation &&
        files_revision == backlog_files_revision &&
        smb_revision == backlog_smb_revision && shq_revision == backlog_shq_revision) return;
    if (uxQueueMessagesWaiting(request_queue) || !catalog.ready ||
        !AirSenseState::device_standby() || !storage.begin()) return;
    RunContext run = {};
    run.generation = __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE);
    run.section = Config::Section::Smb;
    run.config_revision = smb_revision;
    __atomic_store_n(&backlog_refresh, false, __ATOMIC_RELEASE);
    const bool files_changed = __atomic_exchange_n(&backlog_files_changed, false, __ATOMIC_ACQ_REL) ||
        files_revision != backlog_files_revision;
    if (files_changed) roots_cached = false;

    Backlog smb = {}, shq = {};
    smb.config_revision = smb_revision;
    shq.config_revision = shq_revision;
    smb.catalog_generation = shq.catalog_generation = catalog.generation;
    smb.files_revision = shq.files_revision = files_revision;
    SmbConfig smb_config = {};
    SleepHqSync::Config shq_config = {};
    smb.known = snapshot_smb_config(smb_config, smb.error, sizeof(smb.error), false);
    shq.known = snapshot_sleephq_config(shq_config, shq.error, sizeof(shq.error), false);

    bool cache_ready = true;
    if (catalog.entries > backlog_entry_count) {
        // Three bytes per session; large catalogs require PSRAM.
        void *grown = aircannect::Memory::realloc_large(backlog_entries,
            catalog.entries * sizeof(BacklogEntry), catalog.entries <= 128);
        cache_ready = grown != nullptr;
        if (cache_ready) {
            backlog_entries = static_cast<BacklogEntry *>(grown);
            memset(backlog_entries + backlog_entry_count, 0,
                   (catalog.entries - backlog_entry_count) * sizeof(BacklogEntry));
            backlog_entry_count = catalog.entries;
        }
    }
    uint8_t invalidate = 0;
    if (!incremental || files_changed) invalidate = SMB_VALID | SHQ_VALID;
    if (smb_revision != backlog_smb_revision) invalidate |= SMB_VALID;
    if (shq_revision != backlog_shq_revision) invalidate |= SHQ_VALID;
    for (uint32_t i = 0; i < backlog_entry_count; ++i)
        backlog_entries[i].flags &= ~invalidate;
    for (uint32_t i = 0; i < changes.count; ++i) {
        if (changes.indexes[i] < backlog_entry_count)
            backlog_entries[changes.indexes[i]].flags = 0;
    }
    if (cache_ready) {
        backlog_catalog_generation = catalog.generation;
        backlog_files_revision = files_revision;
        backlog_smb_revision = smb_revision;
        backlog_shq_revision = shq_revision;
    } else {
        smb.known = shq.known = false;
        copy_text(smb.error, sizeof(smb.error), "backlog_alloc");
        copy_text(shq.error, sizeof(shq.error), "backlog_alloc");
    }

    for (uint32_t i = 0; i < catalog.entries && (smb.known || shq.known); ++i) {
        if (uxQueueMessagesWaiting(request_queue) || run_aborted(&run)) {
            storage.end();
            __atomic_store_n(&backlog_refresh, true, __ATOMIC_RELEASE);
            return;
        }
        auto &cached = backlog_entries[i];
        const bool read_smb = smb.known && !(cached.flags & SMB_VALID);
        const bool read_shq = shq.known && !(cached.flags & SHQ_VALID);
        EdfCatalog::Entry entry = {};
        if ((read_smb || read_shq) &&
            !storage.run([&](fs::FS &) { return EdfCatalog::read(i, entry); })) {
            smb.known = shq.known = false;
            copy_text(smb.error, sizeof(smb.error), "catalog_read");
            copy_text(shq.error, sizeof(shq.error), "catalog_read");
            break;
        }
        if (read_smb || read_shq) {
            if (entry.recording_ready()) cached.flags &= ~WAITING;
            else cached.flags |= WAITING;
        }
        if (cached.flags & WAITING) {
            cached.flags |= SMB_VALID | SHQ_VALID;
            ++smb.waiting_sessions;
            ++shq.waiting_sessions;
            continue;
        }
        if (read_smb) {
            cached.smb_files = state_marker_exists(smb_config, entry) ? 0 :
                2 * (sizeof(SESSION_SUFFIXES) / sizeof(SESSION_SUFFIXES[0]));
            cached.flags |= SMB_VALID;
        }
        smb.files += cached.smb_files;
        smb.sessions += cached.smb_files ? 1 : 0;
        if (read_shq) {
            uint32_t files = 0;
            bool incomplete = false;
            shq.known = SleepHqSync::pending_files(storage, shq_config, entry,
                files, incomplete, shq.error, sizeof(shq.error));
            if (shq.known) {
                cached.shq_files = files;
                cached.flags = (cached.flags & ~SHQ_PENDING) | SHQ_VALID |
                    (incomplete ? SHQ_PENDING : 0);
            }
        }
        shq.files += cached.shq_files;
        shq.sessions += (cached.flags & SHQ_PENDING) ? 1 : 0;
        if (run_aborted(&run)) {
            if (!storage.valid()) cached.flags = 0;
            storage.end();
            __atomic_store_n(&backlog_refresh, true, __ATOMIC_RELEASE);
            return;
        }
    }
    if (smb.known && (!roots_cached || roots_endpoint != endpoint_hash(smb_config) ||
                      roots_catalog_generation != catalog.generation)) {
        uint8_t *buffer = static_cast<uint8_t *>(
            aircannect::Memory::alloc_large(SdStorage::READ_CHUNK_BYTES));
        uint32_t pending = 0;
        smb.known = buffer != nullptr;
        if (!buffer) copy_text(smb.error, sizeof(smb.error), "backlog_alloc");
        for (const char *path : ROOT_FILES) {
            if (!smb.known || run_aborted(&run)) break;
            RootFileState state;
            smb.known = read_root_state(smb_config, path, buffer,
                SdStorage::READ_CHUNK_BYTES, state, smb.error, sizeof(smb.error), run);
            if (state.exists && !state.confirmed) ++pending;
        }
        aircannect::Memory::free(buffer);
        roots_cached = smb.known && !run_interrupted(run);
        if (roots_cached) {
            roots_pending = pending;
            roots_endpoint = endpoint_hash(smb_config);
            roots_catalog_generation = catalog.generation;
        }
    }
    if (smb.known) smb.files += roots_pending;
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (!storage.valid() || !current.ready || current.generation != catalog.generation ||
        SdStorage::files_revision(SdStorage::FileScope::Export) != files_revision ||
        Config::revision(Config::Section::Smb) != smb_revision ||
        Config::revision(Config::Section::SleepHq) != shq_revision) {
        // An interrupted marker read is not evidence that the marker is absent.
        for (uint32_t i = 0; i < backlog_entry_count; ++i)
            backlog_entries[i].flags = 0;
        roots_cached = false;
        storage.end();
        __atomic_store_n(&backlog_refresh, true, __ATOMIC_RELEASE);
        return;
    }
    if (run.storage_yielded) {
        storage.end();
        __atomic_store_n(&backlog_refresh, true, __ATOMIC_RELEASE);
        return;
    }
    storage.end();
    Backlog previous_smb, previous_shq;
    portENTER_CRITICAL(&status_mux);
    previous_smb = status.backlog;
    retain_backlog_counts(smb, previous_smb);
    status.backlog = smb;
    portEXIT_CRITICAL(&status_mux);
    portENTER_CRITICAL(&sleephq_status_mux);
    previous_shq = sleephq_status.backlog;
    retain_backlog_counts(shq, previous_shq);
    sleephq_status.backlog = shq;
    portEXIT_CRITICAL(&sleephq_status_mux);
    log_backlog_result("SMB", previous_smb, smb);
    log_backlog_result("SleepHQ", previous_shq, shq);
    publish_change();
}

template <typename T>
static void publish_run_outcome(const T &result, bool success) {
    constexpr bool shq = std::is_same<T, SleepHqStatus>::value;
    auto *mux = shq ? &sleephq_status_mux : &status_mux;
    auto &finished = shq ? sleephq_status.last_run : status.last_run;
    const uint32_t epoch = static_cast<uint32_t>(time(nullptr));
    portENTER_CRITICAL(mux);
    capture_run_outcome(finished, result, success, epoch);
    portEXIT_CRITICAL(mux);
    publish_change();
}

template <typename T>
static RunResult run_logged(const Request &request,
                             bool (*run)(const Request &, RunContext &),
                             void (*snapshot)(T &), bool generic_error = true) {
    const char *name = engine_name(request.engine);
    const auto section = engine_adapter(request.engine)->section;
    const uint32_t config_revision = Config::revision(section);
    if (request_aborted(request)) {
        return RunResult::Finished;
    }
    const uint32_t started = millis();
    Log::logf(CAT_EXPORT, LOG_INFO, "[%s] Sync started reason=%s wait=%lums\n", name,
              request.action == Action::Repair ? "repair" : request.kind == RequestKind::Retry ? "retry" :
              request.kind == RequestKind::PostTherapy ? "post_therapy" :
              request.kind == RequestKind::Startup ? "startup" : "manual",
              (unsigned long)(started - request.queued_ms));
    RunContext context = {};
    context.generation = request.generation;
    context.request = &request;
    context.section = section;
    context.config_revision = config_revision;
    context.progress = &begin_export_progress(request);
    context.progress->watch.attempt();
    note_export_progress(*context.progress, "admission", "working");
    storage.set_io_observer(export_storage_wait, &context);
    const bool success = run(request, context);
    storage.set_io_observer(nullptr, nullptr);
    check_export_progress(context);
    T result;
    snapshot(result);
    if (!success && !run_interrupted(context) && result.state == State::Error)
        publish_run_outcome(result, false);
    if (success && result.state != State::Disabled) {
        context.progress_step = UINT64_MAX;
        advance_export(context, 0, "complete");
    }
    const bool retry_pending = request.engine == Engine::SleepHq && sleephq_retry.pending;
    if (retry_pending)
        note_export_progress(*context.progress, context.progress->phase, "retry_backoff", true);
    else if (!context.storage_yielded)
        context.progress->active = false;
    if (!success && (generation_aborted(request.generation) || !storage.valid() ||
                     Config::revision(section) != config_revision)) {
        Log::logf(CAT_EXPORT, LOG_INFO, "[%s] Sync interrupted: %s\n", name,
                  Config::revision(section) != config_revision ? "configuration changed" :
                  generation_aborted(request.generation) ? "therapy started" : "storage changed");
    } else if (!success && context.storage_yielded) {
        Log::logf(CAT_EXPORT, LOG_DEBUG, "[%s] Sync paused for storage\n", name);
    } else if (!success) {
        if (generic_error)
            Log::logf(CAT_EXPORT, LOG_ERROR, "[%s] Sync failed: %s\n", name, result.last_error);
    } else if (result.state == State::Disabled) {
        Log::logf(CAT_EXPORT, LOG_DEBUG, "[%s] Sync skipped: disabled\n", name);
    } else {
        char reused[64];
        if constexpr (std::is_same<T, SleepHqStatus>::value)
            snprintf(reused, sizeof(reused), "attached=%u present=%u", result.files_attached, result.files_skipped);
        else
            snprintf(reused, sizeof(reused), "skipped=%u", result.files_skipped);
        Log::logf(CAT_EXPORT, LOG_INFO,
                  "[%s] Sync complete uploaded=%u %s bytes=%llu elapsed=%ums\n",
                  name, result.files_uploaded, reused,
                  (unsigned long long)result.bytes_uploaded, (unsigned)(millis() - started));
    }
    Log::logf(CAT_EXPORT, LOG_DEBUG, "[%s] Worker stack minimum free=%u/%u bytes\n",
              name, (unsigned)uxTaskGetStackHighWaterMark(nullptr),
              (unsigned)EXPORT_TASK_STACK);
    return !success && context.storage_yielded ? RunResult::Yielded : RunResult::Finished;
}

constexpr uint8_t SYNC_CHECK_ACTIONS = (1u << unsigned(Action::Sync)) |
                                      (1u << unsigned(Action::Check));
const EngineAdapter engines[] = {
    {Engine::Smb, Config::Section::Smb, SYNC_CHECK_ACTIONS,
     &AirBridgeConfig::smb_enabled, &AirBridgeConfig::smb_auto_after_therapy,
     [] { return !Config::get().smb_endpoint.isEmpty(); },
     &status.state, &status.check, &status_mux, set_state, check_smb,
     [](const Request &request) { return run_logged(request, run_smb, get_status); }},
    {Engine::SleepHq, Config::Section::SleepHq,
     SYNC_CHECK_ACTIONS | (1u << unsigned(Action::Repair)),
     &AirBridgeConfig::sleephq_enabled, &AirBridgeConfig::sleephq_auto_after_therapy,
     [] { return !Config::get().sleephq_client_id.isEmpty() &&
                 !Config::get().sleephq_client_secret.isEmpty(); },
     &sleephq_status.state, &sleephq_status.check, &sleephq_status_mux, set_sleephq_state, check_sleephq,
     [](const Request &request) {
         return run_logged(request, run_sleephq, get_sleephq_status, false);
     }},
};
static_assert(sizeof(engines) / sizeof(engines[0]) == size_t(Engine::Count),
              "Every export engine needs an adapter");

static bool take_sleephq_retry(Request &request) {
    if (!sleephq_retry.pending ||
        static_cast<int32_t>(millis() - sleephq_retry.due_ms) < 0 ||
        !Config::get().sleephq_enabled || !AirSenseState::device_standby() ||
        WiFi.status() != WL_CONNECTED || !SdStorage::mounted() ||
        !SdStorage::local_access_allowed() || busy() ||
        __atomic_load_n(&check_pending, __ATOMIC_ACQUIRE)) return false;
    request = {};
    request.kind = RequestKind::Retry;
    request.engine = Engine::SleepHq;
    request.action = sleephq_retry.action;
    copy_text(request.from_day, sizeof(request.from_day), sleephq_retry.from_day);
    copy_text(request.to_day, sizeof(request.to_day), sleephq_retry.to_day);
    request.config_revision = sleephq_retry.config_revision;
    request.generation = __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE);
    request.queued_ms = millis();
    set_sleephq_state(State::Pending);
    return true;
}

static void export_task(void *) {
    while (true) {
        const uint32_t shq_revision = Config::revision(Config::Section::SleepHq);
        if (sleephq_retry.config_revision != shq_revision) {
            export_progress[size_t(Engine::SleepHq)].active = false;
            const bool failed = sleephq_retry.pending || sleephq_retry.blocked;
            reset_sleephq_retry(shq_revision);
            if (failed && __atomic_load_n(&sleephq_status.state, __ATOMIC_ACQUIRE) == State::Error)
                set_sleephq_state(State::Idle);
        }
        auto &retry_progress = export_progress[size_t(Engine::SleepHq)];
        if (retry_progress.active && sleephq_retry.pending) {
            const bool backoff = static_cast<int32_t>(millis() - sleephq_retry.due_ms) < 0;
            const bool standby = AirSenseState::device_standby();
            const bool local = SdStorage::local_access_allowed();
            const bool network = WiFi.status() == WL_CONNECTED;
            const bool readers = SdStorage::waiting_for_readers();
            note_export_progress(retry_progress, retry_progress.phase,
                !standby ? "standby" : !local ? "usb" : !network ? "network" :
                backoff ? "retry_backoff" : readers ? "sd_reader_wait" : "queued",
                !standby || !local || !network || backoff || readers);
        }
        refresh_backlog();
        Request request;
        if (xQueueReceive(request_queue, &request, pdMS_TO_TICKS(1000)) != pdTRUE &&
            !take_sleephq_retry(request))
            continue;
        if (request.generation !=
            __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE)) {
            continue;
        }
        if (request.action == Action::Check) {
            run_check(request);
            continue;
        }
        while ((WiFi.status() != WL_CONNECTED ||
                AirSenseState::rop() < 0) &&
               !request_aborted(request)) {
            refresh_backlog();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (request_aborted(request)) {
            settle_request(request);
            continue;
        }

        const uint32_t preparation = request.kind == RequestKind::PostTherapy
            ? EdfReport::prepare_before_export(request.entry.therapy_day) : 0;
        const uint32_t preparation_started = millis();
        while (!request_aborted(request)) {
            const bool preparing = EdfReport::preparation_pending(preparation) &&
                uint32_t(millis() - preparation_started) < 60000;
            if (!preparing) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (request_aborted(request)) {
            settle_request(request);
            continue;
        }
        request_backlog_refresh();

        initialize_export_progress(request);
        for (const auto &engine : engines) {
            if (generation_aborted(request.generation)) break;
            Request selected;
            if (!select_export_request(request, engine, selected)) continue;
            auto &progress = begin_export_progress(selected);
            progress.queued = false;
            while (!request_aborted(selected)) {
                const bool ready = WiFi.status() == WL_CONNECTED &&
                    AirSenseState::device_standby();
                const bool local = SdStorage::local_access_allowed();
                const bool readers = SdStorage::waiting_for_readers();
                check_waiting_exports();
                note_export_progress(progress, "admission",
                    !AirSenseState::device_standby() ? "standby" : !local ? "usb" :
                    !ready ? "network" : readers ? "sd_reader_wait" : "storage_busy", !ready || !local || readers);
                if (!ready) storage.end();
                if (!ready || !storage.begin()) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                const RunResult result = engine.run(selected);
                const bool yielded = result == RunResult::Yielded && storage.valid() &&
                    !request_aborted(selected);
                storage.end();
                if (!yielded) break;
            }
            storage.end();
            if (request_aborted(selected) && !sleephq_retry.pending) progress.active = false;
        }
        settle_request(request);
    }
}

}  // namespace

const char *state_name(State value) {
    switch (value) {
        case State::Unsupported: return "unsupported";
        case State::Disabled: return "disabled";
        case State::Idle: return "idle";
        case State::Pending: return "pending";
        case State::Working: return "working";
        case State::Error: return "error";
    }
    return "unknown";
}

static const char *enqueue_request(Engine id, Action action, bool startup = false,
                                   const char *from = nullptr, const char *to = nullptr);

void init() {
    if (request_queue || !SdStorage::mounted()) return;
    request_queue = xQueueCreate(4, sizeof(Request));
    BaseType_t created = pdFAIL;
    if (request_queue && aircannect::Memory::psram_available()) {
        created = xTaskCreatePinnedToCoreWithCaps(
            export_task, "export", EXPORT_TASK_STACK, nullptr, 1,
            &export_task_handle, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (request_queue && created != pdPASS) {
        export_task_handle = nullptr;
        created = xTaskCreatePinnedToCore(
            export_task, "export", EXPORT_TASK_STACK, nullptr, 1,
            &export_task_handle, 0);
    }
    if (!request_queue || created != pdPASS) {
        for (const auto &engine : engines)
            engine.set_state(State::Error, "task_initialization_failed");
        Log::logf(CAT_EXPORT, LOG_ERROR, "Task initialization failed\n");
        return;
    }
    for (const auto &engine : engines) {
        engine.set_state(Config::get().*engine.enabled ? State::Idle : State::Disabled, nullptr);
        if (Config::get().*engine.enabled && Config::get().*engine.automatic)
            (void)enqueue_request(engine.id, Action::Sync, true);
    }
}

void therapy_started() {
    const uint32_t generation = __atomic_add_fetch(&abort_generation, 1, __ATOMIC_ACQ_REL);
    if (request_queue) xQueueReset(request_queue);
    request_backlog_refresh();
    const bool checking = __atomic_exchange_n(&check_pending, false, __ATOMIC_ACQ_REL);
    for (const auto &engine : engines) {
        portENTER_CRITICAL(engine.mux);
        const CheckStatus check = *engine.check;
        if (*engine.state == State::Pending || *engine.state == State::Working)
            *engine.state = State::Idle;
        portEXIT_CRITICAL(engine.mux);
        if (checking && (check.state == CheckState::Pending || check.state == CheckState::Working))
            set_check(engine.id, generation, CheckState::Failed, check.config_revision, "preempted");
    }
    publish_change();
}

bool request_post_therapy(const EdfCatalog::Entry &entry) {
    if (!request_queue || AirSenseState::rop() == 1) return false;
    Request request = {};
    request.kind = RequestKind::PostTherapy;
    request.engine = Engine::Count; // Fan out to the enabled automatic engines.
    request.action = Action::Sync;
    request.entry = entry;
    request.generation = __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE);
    if (request_aborted(request)) return false;
    request.queued_ms = millis();
    if (xQueueSend(request_queue, &request, 0) != pdTRUE) return false;
    for (const auto &engine : engines) {
        if (!(Config::get().*engine.enabled && Config::get().*engine.automatic)) continue;
        portENTER_CRITICAL(engine.mux);
        if (*engine.state != State::Error) *engine.state = State::Pending;
        portEXIT_CRITICAL(engine.mux);
    }
    publish_change();
    return true;
}

static const char *enqueue_request(Engine id, Action action, bool startup,
                                   const char *from, const char *to) {
    const auto *engine = engine_adapter(id);
    if (!engine) return "invalid_engine";
    if (!engine->supports(action)) return "unsupported_action";
    const bool check = action == Action::Check;
    if (!request_queue) return "unavailable";
    if (AirSenseState::rop() == 1) return "not_idle";
    if (__atomic_load_n(&check_pending, __ATOMIC_ACQUIRE)) return "busy";
    if (check && __atomic_exchange_n(&check_pending, true, __ATOMIC_ACQ_REL))
        return "busy";
    Request request = {};
    request.kind = startup ? RequestKind::Startup : RequestKind::Manual;
    request.engine = id;
    request.action = action;
    request.generation =
        __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE);
    request.config_revision = Config::revision(engine->section);
    request.queued_ms = millis();
    copy_text(request.from_day, sizeof(request.from_day), from);
    copy_text(request.to_day, sizeof(request.to_day), to);
    if (check) set_check(id, request.generation, CheckState::Pending, request.config_revision);
    else engine->set_state(State::Pending, nullptr);
    if (xQueueSend(request_queue, &request, 0) != pdTRUE) {
        if (check) {
            set_check(id, request.generation, CheckState::NotRun, request.config_revision);
            __atomic_store_n(&check_pending, false, __ATOMIC_RELEASE);
        } else engine->set_state(State::Error, "queue_full");
        return "queue_full";
    }
    return nullptr;
}

bool request_manual(Engine engine, Action action, const char *from, const char *to,
                    const char **error) {
    if (!from) from = "";
    if (!to || !to[0]) to = from;
    auto valid = [](const char *day) {
        if (!day[0]) return true;
        if (strlen(day) != 8) return false;
        char dac[9];
        snprintf(dac, sizeof(dac), "%.2s%.2s%.4s", day + 6, day + 4, day);
        Air10Clock::Calendar calendar;
        return Air10Clock::parse_calendar(dac, "000000", calendar);
    };
    const char *reason = !valid(from) || !valid(to) || (!from[0] && to[0]) ||
        strcmp(from, to) > 0 || (action != Action::Repair && (from[0] || to[0]))
        ? "invalid_day_range" : action_blocked(engine, action);
    if (!reason) reason = enqueue_request(engine, action, false, from, to);
    if (error) *error = reason;
    return reason == nullptr;
}

void request_backlog_refresh(bool files_changed) {
    if (files_changed) __atomic_store_n(&backlog_files_changed, true, __ATOMIC_RELEASE);
    __atomic_store_n(&backlog_refresh, true, __ATOMIC_RELEASE);
}

static void validate_backlog(Backlog &out, Config::Section section) {
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    if (!catalog.ready || !SdStorage::mounted() ||
        out.config_revision != Config::revision(section) ||
        out.catalog_generation != catalog.generation ||
        out.files_revision != SdStorage::files_revision(SdStorage::FileScope::Export)) out = {};
}

uint32_t revision() {
    return __atomic_load_n(&status_revision, __ATOMIC_ACQUIRE);
}

PublicationStamp publication_stamp() {
    return {revision(), Config::revision(Config::Section::Smb),
        Config::revision(Config::Section::SleepHq), EdfCatalog::revision(), SdStorage::files_revision(SdStorage::FileScope::Export),
        WiFiSetup::revision(), Arbiter::get_state(), AirSenseState::rop(),
        SdStorage::mounted(), SdStorage::local_access_allowed(),
        __atomic_load_n(&backlog_refresh, __ATOMIC_ACQUIRE),
        __atomic_load_n(&check_pending, __ATOMIC_ACQUIRE)};
}

bool busy() {
    for (const auto &engine : engines) {
        const State state = __atomic_load_n(engine.state, __ATOMIC_ACQUIRE);
        if (state == State::Pending || state == State::Working) return true;
    }
    return false;
}

static State configured_state(State state, bool enabled) {
    if (state == State::Pending || state == State::Working) return state;
    if (!enabled) return State::Disabled;
    return state == State::Disabled ? State::Idle : state;
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = status;
    portEXIT_CRITICAL(&status_mux);
    out.state = configured_state(out.state, Config::get().smb_enabled);
    out.has_files = out.state == State::Working || out.last_sync_epoch ||
        out.files_seen || out.files_uploaded || out.files_skipped;
    if (out.check.config_revision != Config::revision(Config::Section::Smb)) out.check = {};
    validate_backlog(out.backlog, Config::Section::Smb);
}

void get_sleephq_status(SleepHqStatus &out) {
    portENTER_CRITICAL(&sleephq_status_mux);
    out = sleephq_status;
    portEXIT_CRITICAL(&sleephq_status_mux);
    out.state = configured_state(out.state, Config::get().sleephq_enabled);
    out.has_files = out.state == State::Working || out.last_sync_epoch ||
        out.files_seen || out.files_uploaded || out.files_skipped;
    if (out.check.config_revision != Config::revision(Config::Section::SleepHq)) out.check = {};
    validate_backlog(out.backlog, Config::Section::SleepHq);
}

const char *action_blocked(Engine id, Action action) {
    const auto *engine = engine_adapter(id);
    if (!engine) return "invalid_engine";
    if (!engine->supports(action)) return "unsupported_action";
    const auto &cfg = Config::get();
    if (action != Action::Check && !(cfg.*engine->enabled)) return "disabled";
    if (!SdStorage::local_access_allowed()) return "usb_storage_active";
    if (!SdStorage::mounted()) return "sd_unavailable";
    if (!engine->configured()) return "not_configured";
    if (!AirSenseState::device_standby()) return "not_idle";
    if (WiFi.status() != WL_CONNECTED) return "offline";
    if (!request_queue) return "unavailable";
    if (busy() || __atomic_load_n(&check_pending, __ATOMIC_ACQUIRE)) return "busy";
    return nullptr;
}

const char *backlog_state(const Backlog &backlog, bool smb) {
    const auto &cfg = Config::get();
    if (smb ? cfg.smb_endpoint.isEmpty() :
        cfg.sleephq_client_id.isEmpty() || cfg.sleephq_client_secret.isEmpty())
        return "unconfigured";
    if (!backlog.known) return backlog.error[0] ? "error" : "unknown";
    return !backlog.files && backlog.sessions ? "confirming" : "ready";
}

}  // namespace ExportSync

#else

namespace ExportSync {

const char *action_blocked(Engine, Action) { return "unsupported"; }
const char *backlog_state(const Backlog &, bool) { return "unsupported"; }

const char *state_name(State) { return "unsupported"; }
void init() {}
uint32_t revision() { return 0; }
PublicationStamp publication_stamp() { return {}; }
bool busy() { return false; }
void therapy_started() {}
bool request_post_therapy(const EdfCatalog::Entry &) { return false; }
bool request_manual(Engine, Action, const char *, const char *, const char **error) {
    if (error) *error = "unsupported";
    return false;
}
void request_backlog_refresh(bool) {}

void get_status(Status &out) {
    out = {};
    out.state = State::Unsupported;
}

void get_sleephq_status(SleepHqStatus &out) {
    out = {};
    out.state = State::Unsupported;
}

}  // namespace ExportSync

#endif

const char *ExportSync::engine_name(Engine engine) {
    switch (engine) {
        case Engine::Smb: return "SMB";
        case Engine::SleepHq: return "SLEEPHQ";
        case Engine::Count: break;
    }
    return "UNKNOWN";
}

bool ExportSync::parse_engine(const char *name, Engine &engine) {
    if (!name) return false;
    for (unsigned i = 0; i < unsigned(Engine::Count); ++i) {
        const auto candidate = static_cast<Engine>(i);
        if (!strcasecmp(name, engine_name(candidate))) {
            engine = candidate;
            return true;
        }
    }
    return false;
}

const char *ExportSync::check_state_name(CheckState state) {
    switch (state) {
        case CheckState::NotRun: return "not_run";
        case CheckState::Pending: return "pending";
        case CheckState::Working: return "working";
        case CheckState::Passed: return "passed";
        case CheckState::Failed: return "failed";
    }
    return "not_run";
}
