#include "export_sync.h"
#include "edf_report.h"

#include "board.h"

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
#include "memory_manager.h"
#include "sd_storage.h"
#include "sleephq_sync.h"
#include "storage_smb_client.h"
#include "tls_memory.h"
#include "uart_arbiter.h"
#include "airsense_state.h"
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
    ManualSmb,
    ManualSleepHq,
    StartupSmb,
    StartupSleepHq,
    CheckSmb,
    CheckSleepHq,
};

struct Request {
    RequestKind kind;
    EdfCatalog::Entry entry;
    uint32_t generation;
    uint32_t config_revision;
    uint32_t queued_ms;
};

struct RunContext {
    uint32_t generation;
    BackgroundOperationControl operation;
    bool network_only;
    uint32_t config_revision;
    Config::Section section;
};

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

static void publish_change() {
    __atomic_add_fetch(&status_revision, 1, __ATOMIC_RELEASE);
}

static void set_check(bool smb, uint32_t generation, CheckState state, uint32_t revision,
                      const char *error = "") {
    portMUX_TYPE *mux = smb ? &status_mux : &sleephq_status_mux;
    CheckStatus &check = smb ? status.check : sleephq_status.check;
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

static bool export_ready(const EdfCatalog::Entry &entry) {
    constexpr uint8_t required = EdfCatalog::ENTRY_LIVE_COMPLETE |
        EdfCatalog::ENTRY_IDENTIFICATION_READY | EdfCatalog::ENTRY_STR_READY;
    return (entry.flags & required) == required;
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

static void set_state(State state, const char *error = nullptr) {
    portENTER_CRITICAL(&status_mux);
    __atomic_store_n(&status.state, state, __ATOMIC_RELEASE);
    if (error) copy_text(status.last_error, sizeof(status.last_error), error);
    else if (state != State::Error) status.last_error[0] = 0;
    portEXIT_CRITICAL(&status_mux);
    publish_change();
}

static void set_sleephq_state(State state, const char *error = nullptr) {
    portENTER_CRITICAL(&sleephq_status_mux);
    __atomic_store_n(&sleephq_status.state, state, __ATOMIC_RELEASE);
    if (error) {
        copy_text(sleephq_status.last_error,
                  sizeof(sleephq_status.last_error), error);
    } else if (state != State::Error) {
        sleephq_status.last_error[0] = 0;
    }
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();
}

static void publish_sleephq_progress(
    void *, const SleepHqSync::Progress &progress) {
    portENTER_CRITICAL(&sleephq_status_mux);
    sleephq_status.files_seen = progress.files_seen;
    sleephq_status.files_uploaded = progress.files_uploaded;
    sleephq_status.files_skipped = progress.files_skipped;
    sleephq_status.bytes_uploaded = progress.bytes_uploaded;
    sleephq_status.import_id = progress.import_id;
    copy_text(sleephq_status.current_day,
              sizeof(sleephq_status.current_day), progress.current_day);
    copy_text(sleephq_status.import_status,
              sizeof(sleephq_status.import_status), progress.import_status);
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();
}

static bool run_aborted(void *context) {
    const RunContext *run = static_cast<const RunContext *>(context);
    return !run || generation_aborted(run->generation) ||
           (run->network_only ? !AirSenseState::device_standby() ||
               Config::revision(run->section) != run->config_revision : !storage.valid());
}

static bool wait_resolve(StorageSmbClient &client, RunContext &run,
                         char *error, size_t error_size) {
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
    while (true) {
        const StorageSmbOperationResult result = client.step_close_writer(
            error, error_size, &run.operation);
        if (result == StorageSmbOperationResult::Ready) return true;
        if (result == StorageSmbOperationResult::Error) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void disconnect(StorageSmbClient &client, RunContext &run) {
    for (uint16_t i = 0; i < 200; i++) {
        const StorageSmbOperationResult result =
            client.step_disconnect(&run.operation);
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
    return storage.run([&](fs::FS &fs) {
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
};

static bool read_root_state(const SmbConfig &config, const char *path,
                             uint8_t *buffer, size_t buffer_size,
                             RootFileState &out, char *error, size_t error_size) {
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
    uint32_t crc = crc32_ieee_initial();
    for (uint64_t remaining = size; remaining;) {
        const size_t wanted = remaining < buffer_size ? remaining : buffer_size;
        if (input.read(buffer, wanted) != wanted) {
            copy_text(error, error_size, "local_read_short");
            return false;
        }
        crc = crc32_ieee_update(crc, buffer, wanted);
        remaining -= wanted;
    }
    input.close();
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
    return storage.valid();
}

static bool sync_smb_file(StorageSmbClient &client, const char *local_path,
                          const RootFileState *root, uint8_t *buffer,
                          size_t buffer_size, RunContext &run,
                          char *error, size_t error_size) {
    note_seen();
    SdStorage::Reader input;
    if (!input.open(storage, local_path)) {
        snprintf(error, error_size, "local_open:%s", local_path);
        return false;
    }
    const uint64_t local_size = input.size();
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
    if (root && !storage.run([&](fs::FS &fs) {
        return !fs.exists(root->path) || fs.remove(root->path);
    })) {
        copy_text(error, error_size, "state_marker_remove");
        return false;
    }
    if (!wait_open(client, remote, run, error, error_size)) {
        input.close();
        return false;
    }

    uint64_t sent = 0;
    bool success = true;
    while (sent < local_size) {
        if (run_aborted(&run)) {
            copy_text(error, error_size, "preempted");
            success = false;
            break;
        }
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
        run.operation.note_progress(millis());
        taskYIELD();
    }
    input.close();
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
    note_uploaded(sent);
    if (root && !write_smb_marker(root->directory, root->path,
                                  root->marker, sizeof(root->marker))) {
        copy_text(error, error_size, "state_marker_write");
        return false;
    }
    Log::logf(CAT_EXPORT, LOG_DEBUG,
              "[SMB] uploaded %s bytes=%llu\n", local_path,
              static_cast<unsigned long long>(sent));
    return true;
}

static bool sync_session_files(StorageSmbClient &client,
                               const EdfCatalog::Entry &entry,
                               uint8_t *buffer, size_t buffer_size,
                               RunContext &run,
                               char *error, size_t error_size) {
    char path[128];
    for (const char *suffix : SESSION_SUFFIXES) {
        snprintf(path, sizeof(path), "/DATALOG/%s/%s_%s.edf",
                 entry.therapy_day, entry.file_prefix, suffix);
        if (!sync_smb_file(client, path, nullptr, buffer, buffer_size,
                           run, error, error_size)) return false;
        snprintf(path, sizeof(path), "/DATALOG/%s/%s_%s.crc",
                 entry.therapy_day, entry.file_prefix, suffix);
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
        RootFileState state;
        if (!read_root_state(config, path, buffer, buffer_size, state, error, error_size))
            return false;
        if (!state.exists) continue;
        if (!sync_smb_file(client, path, &state, buffer, buffer_size,
                           run, error, error_size)) return false;
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

static bool run_smb(const Request &request) {
    char error[64] = {};
    SmbConfig config = {};
    if (!snapshot_smb_config(config, error, sizeof(error))) {
        set_state(error[0] ? State::Error : State::Disabled,
                  error[0] ? error : nullptr);
        return !error[0];
    }
    set_state(State::Working);
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

    RunContext run = {};
    run.generation = request.generation;
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

    if (success) {
        for (uint32_t i = catalog.entries; success && i > 0; i--) {
            EdfCatalog::Entry entry;
            if (!storage.run([&](fs::FS &) { return EdfCatalog::read(i - 1, entry); })) {
                copy_text(error, sizeof(error), "catalog_read");
                success = false;
                break;
            }
            if (!export_ready(entry) || state_marker_exists(config, entry)) continue;
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
        if (success)
            success = sync_root_files(client, config, buffer, buffer_size, run,
                                      error, sizeof(error));
    }

    if (client.connected()) disconnect(client, run);
    else client.abort_connection();
    heap_caps_free(buffer);
    if (!success) {
        if (run_aborted(&run)) set_state(State::Idle);
        else set_state(State::Error,
                       error[0] ? error : "smb_sync_failed");
        return false;
    }

    portENTER_CRITICAL(&status_mux);
    status.last_sync_epoch = static_cast<uint32_t>(time(nullptr));
    status.current_day[0] = 0;
    portEXIT_CRITICAL(&status_mux);
    roots_pending = 0;
    roots_cached = true;
    roots_endpoint = endpoint_hash(config);
    roots_catalog_generation = catalog.generation;
    publish_change();
    set_state(State::Idle);
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

static bool run_sleephq(const Request &request) {
    char error[96] = {};
    SleepHqSync::Config config = {};
    if (!snapshot_sleephq_config(config, error, sizeof(error))) {
        set_sleephq_state(error[0] ? State::Error : State::Disabled,
                          error[0] ? error : nullptr);
        return !error[0];
    }
    if (!aircannect::TlsMemory::status().installed) {
        set_sleephq_state(State::Error, "tls_allocator_install");
        return false;
    }

    set_sleephq_state(State::Working);
    portENTER_CRITICAL(&sleephq_status_mux);
    sleephq_status.files_seen = 0;
    sleephq_status.files_uploaded = 0;
    sleephq_status.files_skipped = 0;
    sleephq_status.bytes_uploaded = 0;
    sleephq_status.import_id = 0;
    sleephq_status.import_status[0] = 0;
    copy_text(sleephq_status.current_day,
              sizeof(sleephq_status.current_day),
              request.kind == RequestKind::PostTherapy
                  ? request.entry.therapy_day : "");
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();

    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    if (!catalog.ready) {
        set_sleephq_state(State::Error, "catalog_not_ready");
        return false;
    }

    RunContext run = {};
    run.generation = request.generation;
    run.operation.started_ms = millis();
    run.operation.last_progress_ms = run.operation.started_ms;
    run.operation.timeout_ms = SLEEPHQ_IDLE_TIMEOUT_MS;
    run.operation.timeout_policy = BackgroundOperationTimeoutPolicy::NoProgress;
    run.operation.should_abort = run_aborted;
    run.operation.ctx = &run;

    SleepHqSync::Progress progress = {};
    bool success = true;
    for (uint32_t i = catalog.entries; success && i > 0; i--) {
        EdfCatalog::Entry entry;
        if (!storage.run([&](fs::FS &) { return EdfCatalog::read(i - 1, entry); })) {
            copy_text(error, sizeof(error), "catalog_read");
            success = false;
            break;
        }
        if (!export_ready(entry)) continue;
        SleepHqSync::BacklogChanges changes = {};
        success = SleepHqSync::sync_session(
            storage, config, entry, run.operation, progress,
            publish_sleephq_progress, nullptr, error, sizeof(error), &changes);
        if (changes.entry) invalidate_backlog_entry(i - 1, SHQ_VALID);
        if (changes.other_entries) {
            for (uint32_t n = 0; n < backlog_entry_count; ++n)
                backlog_entries[n].flags &= ~SHQ_VALID;
            request_backlog_refresh();
        }
    }

    if (!success) {
        if (run_aborted(&run)) set_sleephq_state(State::Idle);
        else set_sleephq_state(State::Error,
                               error[0] ? error : "sleephq_sync_failed");
        return false;
    }

    portENTER_CRITICAL(&sleephq_status_mux);
    sleephq_status.last_sync_epoch = static_cast<uint32_t>(time(nullptr));
    sleephq_status.current_day[0] = 0;
    portEXIT_CRITICAL(&sleephq_status_mux);
    publish_change();
    set_sleephq_state(State::Idle);
    return true;
}

static void run_check(const Request &request) {
    const bool smb = request.kind == RequestKind::CheckSmb;
    char error[96] = {};
    bool success = false;
    set_check(smb, request.generation, CheckState::Working, request.config_revision);
    RunContext run = {};
    run.generation = request.generation;
    run.network_only = true;
    run.config_revision = request.config_revision;
    run.section = smb ? Config::Section::Smb : Config::Section::SleepHq;
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
    } else if (smb) {
        SmbConfig config = {};
        StorageSmbClient client;
        char remote[aircannect::AC_STORAGE_SMB_REMOTE_PATH_MAX];
        StorageSmbRemoteStat stat;
        success = snapshot_smb_config(config, error, sizeof(error), false) &&
            client.configure(config.endpoint, config.user, config.password, error, sizeof(error)) &&
            wait_resolve(client, run, error, sizeof(error)) &&
            wait_connect(client, run, error, sizeof(error)) &&
            client.make_remote_path("/", remote, sizeof(remote)) &&
            wait_stat(client, remote, stat, run, error, sizeof(error));
        if (success && (!stat.exists || !stat.directory)) {
            success = false;
            copy_text(error, sizeof(error), "directory_missing");
        }
        if (client.connected()) disconnect(client, run);
        else client.abort_connection();
    } else {
        SleepHqSync::Config config = {};
        if (!aircannect::TlsMemory::status().installed)
            copy_text(error, sizeof(error), "tls_allocator_install");
        else success = snapshot_sleephq_config(config, error, sizeof(error), false) &&
                SleepHqSync::check_account(config, run.operation, error, sizeof(error));
    }
    if (request.generation != __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE)) return;
    if (run_aborted(&run)) {
        success = false;
        copy_text(error, sizeof(error), "preempted");
    }
    set_check(smb, request.generation, success ? CheckState::Passed : CheckState::Failed,
              request.config_revision, success ? "" : error[0] ? error : "check_failed");
    __atomic_store_n(&check_pending, false, __ATOMIC_RELEASE);
    Log::logf(CAT_EXPORT, success ? LOG_INFO : LOG_ERROR, "[%s] Check %s%s%s\n",
              smb ? "SMB" : "SLEEPHQ", success ? "passed" : "failed",
              error[0] ? ": " : "", error);
}

static void log_backlog_result(const char *destination, const Backlog &previous,
                               const Backlog &current) {
    const auto failed = [](const Backlog &value) {
        return !value.known && value.error[0] &&
            strcmp(value.error, "endpoint_missing") && strcmp(value.error, "credentials_missing");
    };
    if (failed(current) && (!failed(previous) || strcmp(previous.error, current.error)))
        Log::logf(CAT_EXPORT, LOG_WARN, "[%s] Backlog failed generation=%lu: %s\n",
            destination, (unsigned long)current.catalog_generation, current.error);
    else if (current.known && failed(previous))
        Log::logf(CAT_EXPORT, LOG_INFO, "[%s] Backlog recovered files=%lu\n",
            destination, (unsigned long)current.files);
}

static void refresh_backlog() {
    EdfCatalog::Changes changes;
    const bool incremental = EdfCatalog::changes_since(backlog_catalog_generation, changes);
    const auto &catalog = changes.catalog;
    const uint32_t smb_revision = Config::revision(Config::Section::Smb);
    const uint32_t shq_revision = Config::revision(Config::Section::SleepHq);
    if (!__atomic_load_n(&backlog_refresh, __ATOMIC_ACQUIRE) &&
        catalog.generation == backlog_catalog_generation &&
        smb_revision == backlog_smb_revision && shq_revision == backlog_shq_revision) return;
    if (uxQueueMessagesWaiting(request_queue) || !catalog.ready ||
        !AirSenseState::device_standby() || !storage.begin()) return;
    __atomic_store_n(&backlog_refresh, false, __ATOMIC_RELEASE);
    const bool files_changed = __atomic_exchange_n(&backlog_files_changed, false, __ATOMIC_ACQ_REL);
    if (files_changed) roots_cached = false;

    Backlog smb = {}, shq = {};
    smb.config_revision = smb_revision;
    shq.config_revision = shq_revision;
    smb.catalog_generation = shq.catalog_generation = catalog.generation;
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
        backlog_smb_revision = smb_revision;
        backlog_shq_revision = shq_revision;
    } else {
        smb.known = shq.known = false;
        copy_text(smb.error, sizeof(smb.error), "backlog_alloc");
        copy_text(shq.error, sizeof(shq.error), "backlog_alloc");
    }

    for (uint32_t i = 0; i < catalog.entries && (smb.known || shq.known); ++i) {
        if (uxQueueMessagesWaiting(request_queue)) {
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
            if (export_ready(entry)) cached.flags &= ~WAITING;
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
        if (!storage.valid()) {
            cached.flags = 0;
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
            if (!smb.known) break;
            RootFileState state;
            smb.known = read_root_state(smb_config, path, buffer,
                SdStorage::READ_CHUNK_BYTES, state, smb.error, sizeof(smb.error));
            if (state.exists && !state.confirmed) ++pending;
        }
        aircannect::Memory::free(buffer);
        roots_cached = smb.known;
        if (roots_cached) {
            roots_pending = pending;
            roots_endpoint = endpoint_hash(smb_config);
            roots_catalog_generation = catalog.generation;
        }
    }
    if (smb.known) smb.files += roots_pending;
    if (!storage.valid() || Config::revision(Config::Section::Smb) != smb_revision ||
        Config::revision(Config::Section::SleepHq) != shq_revision) {
        // An interrupted marker read is not evidence that the marker is absent.
        for (uint32_t i = 0; i < backlog_entry_count; ++i)
            backlog_entries[i].flags = 0;
        storage.end();
        __atomic_store_n(&backlog_refresh, true, __ATOMIC_RELEASE);
        return;
    }
    storage.end();
    Backlog previous_smb, previous_shq;
    portENTER_CRITICAL(&status_mux);
    previous_smb = status.backlog;
    status.backlog = smb;
    portEXIT_CRITICAL(&status_mux);
    portENTER_CRITICAL(&sleephq_status_mux);
    previous_shq = sleephq_status.backlog;
    sleephq_status.backlog = shq;
    portEXIT_CRITICAL(&sleephq_status_mux);
    log_backlog_result("SMB", previous_smb, smb);
    log_backlog_result("SleepHQ", previous_shq, shq);
    publish_change();
}

template <typename T>
static void run_logged(const Request &request, const char *name,
                        bool (*run)(const Request &), void (*snapshot)(T &)) {
    const uint32_t started = millis();
    Log::logf(CAT_EXPORT, LOG_INFO, "[%s] Sync started reason=%s wait=%lums\n", name,
              request.kind == RequestKind::PostTherapy ? "post_therapy" :
              request.kind == RequestKind::StartupSmb ||
              request.kind == RequestKind::StartupSleepHq ? "startup" : "manual",
              (unsigned long)(started - request.queued_ms));
    bool success = run(request);
    T result;
    snapshot(result);
    if (!success && (generation_aborted(request.generation) || !storage.valid())) {
        Log::logf(CAT_EXPORT, LOG_INFO, "[%s] Sync interrupted by storage change or therapy\n", name);
    } else if (!success) {
        Log::logf(CAT_EXPORT, LOG_ERROR, "[%s] Sync failed: %s\n", name, result.last_error);
    } else if (result.state == State::Disabled) {
        Log::logf(CAT_EXPORT, LOG_DEBUG, "[%s] Sync skipped: disabled\n", name);
    } else {
        Log::logf(CAT_EXPORT, LOG_INFO,
                  "[%s] Sync complete uploaded=%u skipped=%u bytes=%llu elapsed=%ums\n",
                  name, result.files_uploaded, result.files_skipped,
                  (unsigned long long)result.bytes_uploaded, (unsigned)(millis() - started));
    }
    Log::logf(CAT_EXPORT, LOG_DEBUG, "[%s] Worker stack minimum free=%u/%u bytes\n",
              name, (unsigned)uxTaskGetStackHighWaterMark(nullptr),
              (unsigned)EXPORT_TASK_STACK);
}

static void export_task(void *) {
    while (true) {
        refresh_backlog();
        Request request;
        if (xQueueReceive(request_queue, &request, pdMS_TO_TICKS(1000)) != pdTRUE)
            continue;
        if (request.generation !=
            __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE)) {
            continue;
        }
        if (request.kind == RequestKind::CheckSmb || request.kind == RequestKind::CheckSleepHq) {
            run_check(request);
            continue;
        }
        while ((WiFi.status() != WL_CONNECTED ||
                AirSenseState::rop() < 0) &&
               !generation_aborted(request.generation)) {
            refresh_backlog();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (generation_aborted(request.generation)) continue;

        const uint32_t preparation = request.kind == RequestKind::PostTherapy
            ? EdfReport::prepare_before_export(request.entry.therapy_day) : 0;
        const uint32_t preparation_started = millis();
        bool admitted = false;
        while (!generation_aborted(request.generation)) {
            const bool preparing = EdfReport::preparation_pending(preparation) &&
                uint32_t(millis() - preparation_started) < 60000;
            if (!preparing && AirSenseState::device_standby() && storage.begin()) {
                admitted = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!admitted) continue;
        if (generation_aborted(request.generation)) { storage.end(); continue; }
        request_backlog_refresh();

        if (request.kind == RequestKind::ManualSmb || request.kind == RequestKind::StartupSmb) {
            run_logged(request, "SMB", run_smb, get_status);
        } else if (request.kind == RequestKind::ManualSleepHq || request.kind == RequestKind::StartupSleepHq) {
            run_logged(request, "SLEEPHQ", run_sleephq, get_sleephq_status);
        } else {
            const AirBridgeConfig &config = Config::get();
            if (config.smb_enabled && config.smb_auto_after_therapy)
                run_logged(request, "SMB", run_smb, get_status);
            if (!generation_aborted(request.generation) &&
                config.sleephq_enabled && config.sleephq_auto_after_therapy)
                run_logged(request, "SLEEPHQ", run_sleephq, get_sleephq_status);
        }
        storage.end();
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

static const char *enqueue_request(bool smb, bool check, bool startup = false);

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
        set_state(State::Error, "task_initialization_failed");
        set_sleephq_state(State::Error, "task_initialization_failed");
        Log::logf(CAT_EXPORT, LOG_ERROR, "Task initialization failed\n");
        return;
    }
    set_state(Config::get().smb_enabled ? State::Idle : State::Disabled);
    set_sleephq_state(Config::get().sleephq_enabled
                          ? State::Idle : State::Disabled);
    if (Config::get().smb_enabled &&
        Config::get().smb_auto_after_therapy) {
        (void)enqueue_request(true, false, true);
    }
    if (Config::get().sleephq_enabled &&
        Config::get().sleephq_auto_after_therapy) {
        (void)enqueue_request(false, false, true);
    }
}

void therapy_started() {
    const uint32_t generation = __atomic_add_fetch(&abort_generation, 1, __ATOMIC_ACQ_REL);
    if (request_queue) xQueueReset(request_queue);
    request_backlog_refresh();
    if (__atomic_exchange_n(&check_pending, false, __ATOMIC_ACQ_REL)) {
        Status smb;
        SleepHqStatus shq;
        get_status(smb);
        get_sleephq_status(shq);
        if (smb.check.state == CheckState::Pending || smb.check.state == CheckState::Working)
            set_check(true, generation, CheckState::Failed, smb.check.config_revision, "preempted");
        if (shq.check.state == CheckState::Pending || shq.check.state == CheckState::Working)
            set_check(false, generation, CheckState::Failed, shq.check.config_revision, "preempted");
    }
    set_state(Config::get().smb_enabled ? State::Idle : State::Disabled);
    set_sleephq_state(Config::get().sleephq_enabled
                          ? State::Idle : State::Disabled);
}

bool request_post_therapy(const EdfCatalog::Entry &entry) {
    const AirBridgeConfig &config = Config::get();
    const bool smb = config.smb_enabled && config.smb_auto_after_therapy;
    const bool sleephq = config.sleephq_enabled &&
                         config.sleephq_auto_after_therapy;
    if (!request_queue || AirSenseState::rop() == 1 ||
        (!smb && !sleephq)) {
        return false;
    }
    Request request = {RequestKind::PostTherapy, entry,
        __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE)};
    request.queued_ms = millis();
    if (xQueueSend(request_queue, &request, 0) != pdTRUE) return false;
    if (smb) set_state(State::Pending);
    if (sleephq) set_sleephq_state(State::Pending);
    return true;
}

static const char *enqueue_request(bool smb, bool check, bool startup) {
    if (!request_queue) return "unavailable";
    if (AirSenseState::rop() == 1) return "not_idle";
    if (__atomic_load_n(&check_pending, __ATOMIC_ACQUIRE)) return "busy";
    if (check && __atomic_exchange_n(&check_pending, true, __ATOMIC_ACQ_REL))
        return "busy";
    Request request = {};
    request.kind = check ? (smb ? RequestKind::CheckSmb : RequestKind::CheckSleepHq)
                         : startup ? (smb ? RequestKind::StartupSmb : RequestKind::StartupSleepHq)
                                   : (smb ? RequestKind::ManualSmb : RequestKind::ManualSleepHq);
    request.generation =
        __atomic_load_n(&abort_generation, __ATOMIC_ACQUIRE);
    request.config_revision = Config::revision(smb ? Config::Section::Smb : Config::Section::SleepHq);
    request.queued_ms = millis();
    if (check) set_check(smb, request.generation, CheckState::Pending, request.config_revision);
    else if (smb) set_state(State::Pending);
    else set_sleephq_state(State::Pending);
    if (xQueueSend(request_queue, &request, 0) != pdTRUE) {
        if (check) {
            set_check(smb, request.generation, CheckState::NotRun, request.config_revision);
            __atomic_store_n(&check_pending, false, __ATOMIC_RELEASE);
        } else if (smb) set_state(State::Error, "queue_full");
        else set_sleephq_state(State::Error, "queue_full");
        return "queue_full";
    }
    return nullptr;
}

static bool request_manual(bool smb, bool check, const char **error) {
    const char *reason = action_blocked(smb, check);
    if (!reason) reason = enqueue_request(smb, check);
    if (error) *error = reason;
    return !reason;
}

bool request_manual_smb(bool check, const char **error) {
    return request_manual(true, check, error);
}

bool request_manual_sleephq(bool check, const char **error) {
    return request_manual(false, check, error);
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
        __atomic_load_n(&backlog_refresh, __ATOMIC_ACQUIRE)) out = {};
}

uint32_t revision() {
    return __atomic_load_n(&status_revision, __ATOMIC_ACQUIRE);
}

PublicationStamp publication_stamp() {
    return {revision(), Config::revision(Config::Section::Smb),
        Config::revision(Config::Section::SleepHq), EdfCatalog::revision(),
        WiFiSetup::revision(), Arbiter::get_state(), AirSenseState::rop(),
        SdStorage::mounted(), __atomic_load_n(&backlog_refresh, __ATOMIC_ACQUIRE),
        __atomic_load_n(&check_pending, __ATOMIC_ACQUIRE)};
}

bool busy() {
    const State smb = __atomic_load_n(&status.state, __ATOMIC_ACQUIRE);
    const State shq = __atomic_load_n(&sleephq_status.state, __ATOMIC_ACQUIRE);
    return smb == State::Pending || smb == State::Working ||
           shq == State::Pending || shq == State::Working;
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

const char *action_blocked(bool smb, bool check) {
    const auto &cfg = Config::get();
    if (!check && !(smb ? cfg.smb_enabled : cfg.sleephq_enabled)) return "disabled";
    if (!SdStorage::mounted()) return "sd_unavailable";
    bool configured = smb ? !cfg.smb_endpoint.isEmpty() :
        !cfg.sleephq_client_id.isEmpty() && !cfg.sleephq_client_secret.isEmpty();
    if (!configured) return "not_configured";
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

const char *action_blocked(bool, bool) { return "unsupported"; }
const char *backlog_state(const Backlog &, bool) { return "unsupported"; }

const char *state_name(State) { return "unsupported"; }
void init() {}
uint32_t revision() { return 0; }
PublicationStamp publication_stamp() { return {}; }
bool busy() { return false; }
void therapy_started() {}
bool request_post_therapy(const EdfCatalog::Entry &) { return false; }
bool request_manual_smb(bool, const char **error) {
    if (error) *error = "unsupported";
    return false;
}
bool request_manual_sleephq(bool, const char **error) {
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
