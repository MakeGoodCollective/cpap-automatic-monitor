#include "sleephq_sync.h"
#include "sd_storage.h"
#include <FS.h>

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "crc.h"
#include "debug_log.h"
#include "sleephq_client.h"
#include "string_util.h"

namespace SleepHqSync {
namespace {

using aircannect::BackgroundOperationControl;
using aircannect::BackgroundOperationStop;
using aircannect::SleepHqClient;
using aircannect::SleepHqImportInfo;
using aircannect::SleepHqImportStatusKind;
using aircannect::SleepHqUploadRequest;
using aircannect::SleepHqUploadResult;
using aircannect::copy_cstr;
using aircannect::sleephq_classify_import_status;

constexpr uint8_t JOURNAL_VERSION = 1;
constexpr size_t JOURNAL_SIZE = 64;
constexpr size_t JOURNAL_CRC_OFFSET = JOURNAL_SIZE - 4;
constexpr size_t MARKER_SIZE = 20;
constexpr size_t FILE_COUNT = 13;
constexpr uint8_t PHASE_UPLOADING = 1;
constexpr uint8_t PHASE_PROCESSING = 2;
constexpr uint32_t IMPORT_POLL_INTERVAL_MS = 5000;
constexpr uint32_t IMPORT_POLL_TIMEOUT_MS = 15 * 60 * 1000UL;

struct Journal {
    uint8_t phase;
    uint8_t flags;
    uint16_t uploaded_mask;
    uint32_t import_id;
    uint32_t team_id;
    uint32_t finalized_epoch;
    uint32_t str_revision;
    char therapy_day[9];
    char file_prefix[16];
};

enum class JournalLoad : uint8_t {
    Unavailable,
    Missing,
    Valid,
    Invalid,
};

struct StatePaths {
    char directory[48];
    char journal[80];
    char journal_part[88];
    char journal_backup[88];
};

struct FileSpec {
    char local_path[128];
    char remote_path[64];
    char name[80];
    bool required;
};

struct FileReader {
    SdStorage::Session *storage;
    SdStorage::Reader file;
    char path[128];
    uint64_t size;

    bool open() {
        if (file) file.close();
        if (!storage) return false;
        if (!file.open(*storage, path) || file.size() != size) {
            if (file) file.close();
            return false;
        }
        return true;
    }

    static bool read(void *context, uint8_t *out, size_t length,
                     size_t &read_size) {
        FileReader *reader = static_cast<FileReader *>(context);
        read_size = 0;
        if (!reader || !reader->file || !out) return false;
        read_size = reader->file.read(out, length);
        return read_size == length;
    }

    static bool reset(void *context) {
        FileReader *reader = static_cast<FileReader *>(context);
        return reader && reader->open();
    }
};

static void set_error(char *out, size_t out_size, const char *error) {
    copy_cstr(out, out_size, error ? error : "sleephq_sync_failed");
}

static bool operation_allows(const BackgroundOperationControl &operation,
                             char *error, size_t error_size) {
    const BackgroundOperationStop stop = operation.stop_reason(millis());
    if (stop == BackgroundOperationStop::None) return true;
    set_error(error, error_size,
              aircannect::background_operation_stop_error(stop));
    return false;
}

static void publish(const Progress &progress, ProgressCallback callback,
                    void *context) {
    if (callback) callback(context, progress);
}

static uint32_t config_hash(const Config &config) {
    uint32_t crc = crc32_ieee_initial();
    const char *values[] = {
        config.client_id, config.team_id, config.device_id,
    };
    for (const char *value : values) {
        crc = crc32_ieee_update(
            crc, reinterpret_cast<const uint8_t *>(value), strlen(value));
        const uint8_t separator = '\n';
        crc = crc32_ieee_update(crc, &separator, 1);
    }
    return crc32_ieee_finish(crc);
}

static bool build_state_paths(const Config &config, StatePaths &paths) {
    const uint32_t hash = config_hash(config);
    const int directory_length = snprintf(
        paths.directory, sizeof(paths.directory), "/airbridge/shq_%08lx",
        static_cast<unsigned long>(hash));
    const int journal_length = snprintf(
        paths.journal, sizeof(paths.journal), "%s/inflight.bin",
        paths.directory);
    const int part_length = snprintf(
        paths.journal_part, sizeof(paths.journal_part), "%s.part",
        paths.journal);
    const int backup_length = snprintf(
        paths.journal_backup, sizeof(paths.journal_backup), "%s.bak",
        paths.journal);
    return directory_length > 0 &&
           static_cast<size_t>(directory_length) < sizeof(paths.directory) &&
           journal_length > 0 &&
           static_cast<size_t>(journal_length) < sizeof(paths.journal) &&
           part_length > 0 &&
           static_cast<size_t>(part_length) < sizeof(paths.journal_part) &&
           backup_length > 0 &&
           static_cast<size_t>(backup_length) < sizeof(paths.journal_backup);
}

static bool ensure_state_directory(fs::FS &storage,
                                   const StatePaths &paths) {
    return storage.exists(paths.directory) || storage.mkdir(paths.directory);
}

static void recover_journal(fs::FS &storage, const StatePaths &paths) {
    storage.remove(paths.journal_part);
    if (!storage.exists(paths.journal) &&
        storage.exists(paths.journal_backup)) {
        (void)storage.rename(paths.journal_backup, paths.journal);
    } else if (storage.exists(paths.journal) &&
               storage.exists(paths.journal_backup)) {
        storage.remove(paths.journal_backup);
    }
}

static bool replace_file(fs::FS &storage, const char *path,
                         const char *part, const char *backup,
                         const uint8_t *data, size_t size) {
    storage.remove(part);
    fs::File file = storage.open(part, FILE_WRITE);
    const bool written = file && file.write(data, size) == size;
    if (file) {
        file.flush();
        file.close();
    }
    if (!written) {
        storage.remove(part);
        return false;
    }

    storage.remove(backup);
    if (storage.exists(path) && !storage.rename(path, backup)) {
        storage.remove(part);
        return false;
    }
    if (!storage.rename(part, path)) {
        (void)storage.rename(backup, path);
        storage.remove(part);
        return false;
    }
    storage.remove(backup);
    return true;
}

static void encode_journal(const Journal &journal, uint8_t *data) {
    memset(data, 0, JOURNAL_SIZE);
    memcpy(data, "ABSQ", 4);
    data[4] = JOURNAL_VERSION;
    data[5] = journal.phase;
    data[6] = journal.flags;
    SdStorage::put_le16(data + 8, journal.uploaded_mask);
    SdStorage::put_le16(data + 10, FILE_COUNT);
    SdStorage::put_le32(data + 12, journal.import_id);
    SdStorage::put_le32(data + 16, journal.team_id);
    SdStorage::put_le32(data + 20, journal.finalized_epoch);
    memcpy(data + 24, journal.therapy_day, 8);
    memcpy(data + 32, journal.file_prefix, 15);
    SdStorage::put_le32(data + 48, journal.str_revision);
    SdStorage::put_le32(data + JOURNAL_CRC_OFFSET,
             crc32_ieee(data, JOURNAL_CRC_OFFSET));
}

static bool valid_digits(const char *text, size_t length) {
    for (size_t i = 0; i < length; i++)
        if (text[i] < '0' || text[i] > '9') return false;
    return true;
}

static bool decode_journal(const uint8_t *data, Journal &journal) {
    if (memcmp(data, "ABSQ", 4) != 0 || data[4] != JOURNAL_VERSION ||
        (data[5] != PHASE_UPLOADING && data[5] != PHASE_PROCESSING) ||
        SdStorage::get_le16(data + 10) != FILE_COUNT || !SdStorage::get_le32(data + 12) ||
        !SdStorage::get_le32(data + 16) ||
        SdStorage::get_le32(data + JOURNAL_CRC_OFFSET) !=
            crc32_ieee(data, JOURNAL_CRC_OFFSET) ||
        !valid_digits(reinterpret_cast<const char *>(data + 24), 8) ||
        !valid_digits(reinterpret_cast<const char *>(data + 32), 8) ||
        data[40] != '_' ||
        !valid_digits(reinterpret_cast<const char *>(data + 41), 6)) {
        return false;
    }

    memset(&journal, 0, sizeof(journal));
    journal.phase = data[5];
    journal.flags = data[6];
    journal.uploaded_mask = SdStorage::get_le16(data + 8);
    journal.import_id = SdStorage::get_le32(data + 12);
    journal.team_id = SdStorage::get_le32(data + 16);
    journal.finalized_epoch = SdStorage::get_le32(data + 20);
    journal.str_revision = SdStorage::get_le32(data + 48);
    memcpy(journal.therapy_day, data + 24, 8);
    memcpy(journal.file_prefix, data + 32, 15);
    return true;
}

static JournalLoad load_journal(SdStorage::Session &access, const StatePaths &paths,
                                Journal &journal, bool recover = true) {
    JournalLoad result = JournalLoad::Missing;
    const bool loaded = access.run([&](fs::FS &storage) {
        if (recover) recover_journal(storage, paths);
        const char *path = !recover && !storage.exists(paths.journal)
            ? paths.journal_backup : paths.journal;
        if (!storage.exists(path)) return true;
        uint8_t data[JOURNAL_SIZE];
        fs::File file = storage.open(path, FILE_READ);
        if (!file) return false;
        bool valid = false;
        if (file.size() == sizeof(data)) {
            if (file.read(data, sizeof(data)) != sizeof(data)) return false;
            valid = decode_journal(data, journal);
        }
        result = valid ? JournalLoad::Valid : JournalLoad::Invalid;
        return true;
    });
    return loaded ? result : JournalLoad::Unavailable;
}

static bool write_journal(SdStorage::Session &access, const StatePaths &paths,
                          const Journal &journal) {
    uint8_t data[JOURNAL_SIZE];
    encode_journal(journal, data);
    return access.run([&](fs::FS &storage) {
        return ensure_state_directory(storage, paths) &&
               replace_file(storage, paths.journal, paths.journal_part,
                            paths.journal_backup, data, sizeof(data));
    });
}

static bool remove_journal(SdStorage::Session &access, const StatePaths &paths) {
    return access.run([&](fs::FS &storage) {
        // Remove recovery copies first so a failed cleanup cannot resurrect one.
        for (const char *path : {paths.journal_backup, paths.journal_part, paths.journal})
            if (storage.exists(path) && !storage.remove(path)) return false;
        return true;
    });
}

static bool marker_path(const StatePaths &paths,
                        const EdfCatalog::Entry &entry,
                        char *path, size_t path_size,
                        char *part = nullptr, size_t part_size = 0,
                        char *backup = nullptr, size_t backup_size = 0) {
    const int path_length = snprintf(path, path_size, "%s/%s.done",
                                     paths.directory, entry.file_prefix);
    if (path_length <= 0 || static_cast<size_t>(path_length) >= path_size)
        return false;
    if (part) {
        const int part_length = snprintf(part, part_size, "%s.part", path);
        if (part_length <= 0 ||
            static_cast<size_t>(part_length) >= part_size) return false;
    }
    if (backup) {
        const int backup_length = snprintf(backup, backup_size, "%s.bak",
                                           path);
        if (backup_length <= 0 ||
            static_cast<size_t>(backup_length) >= backup_size) return false;
    }
    return true;
}

static void encode_marker(const EdfCatalog::Entry &entry, uint8_t *data) {
    memset(data, 0, MARKER_SIZE);
    memcpy(data, "ABSD", 4);
    data[4] = JOURNAL_VERSION;
    data[5] = entry.flags;
    SdStorage::put_le32(data + 8, entry.finalized_epoch);
    SdStorage::put_le32(data + 12, entry.str_revision);
    SdStorage::put_le32(data + 16, crc32_ieee(data, 16));
}

static bool marker_valid(SdStorage::Session &storage, const StatePaths &paths,
                         const EdfCatalog::Entry &entry, bool include_str = true) {
    char path[80];
    if (!marker_path(paths, entry, path, sizeof(path))) return false;
    uint8_t data[MARKER_SIZE];
    SdStorage::Reader file;
    if (!file.open(storage, path)) return false;
    const size_t size = file.size();
    const bool valid = (size == sizeof(data) || size == 16) &&
        file.read(data, size) == size &&
        memcmp(data, "ABSD", 4) == 0 && data[4] == JOURNAL_VERSION &&
        data[5] == entry.flags && SdStorage::get_le32(data + 8) == entry.finalized_epoch &&
        SdStorage::get_le32(data + size - 4) == crc32_ieee(data, size - 4) &&
        (!include_str || (size == 16 ? 0 : SdStorage::get_le32(data + 12)) ==
                         entry.str_revision);
    if (file) file.close();
    return valid;
}

static bool write_marker(SdStorage::Session &access, const StatePaths &paths,
                         const EdfCatalog::Entry &entry) {
    char path[80];
    char part[88];
    char backup[88];
    if (!marker_path(paths, entry, path, sizeof(path), part, sizeof(part),
                     backup, sizeof(backup))) return false;
    uint8_t data[MARKER_SIZE];
    encode_marker(entry, data);
    return access.run([&](fs::FS &storage) {
        return ensure_state_directory(storage, paths) &&
               replace_file(storage, path, part, backup, data, sizeof(data));
    });
}

static EdfCatalog::Entry journal_entry(const Journal &journal) {
    EdfCatalog::Entry entry = {};
    copy_cstr(entry.therapy_day, sizeof(entry.therapy_day),
              journal.therapy_day);
    copy_cstr(entry.file_prefix, sizeof(entry.file_prefix),
              journal.file_prefix);
    entry.flags = journal.flags;
    entry.finalized_epoch = journal.finalized_epoch;
    entry.str_revision = journal.str_revision;
    return entry;
}

static bool build_file_spec(size_t index, const Journal &journal,
                            FileSpec &file) {
    memset(&file, 0, sizeof(file));
    if (index == 0) {
        copy_cstr(file.local_path, sizeof(file.local_path),
                  "/Identification.tgt");
        copy_cstr(file.name, sizeof(file.name), "Identification.tgt");
        copy_cstr(file.remote_path, sizeof(file.remote_path), "./");
        file.required = true;
        return true;
    }
    if (index == 1) {
        copy_cstr(file.local_path, sizeof(file.local_path),
                  "/Identification.crc");
        copy_cstr(file.name, sizeof(file.name), "Identification.crc");
        copy_cstr(file.remote_path, sizeof(file.remote_path), "./");
        file.required = true;
        return true;
    }
    if (index == 2) {
        copy_cstr(file.local_path, sizeof(file.local_path), "/STR.edf");
        copy_cstr(file.name, sizeof(file.name), "STR.edf");
        copy_cstr(file.remote_path, sizeof(file.remote_path), "./");
        file.required = (journal.flags & EdfCatalog::ENTRY_STR_READY) != 0;
        return true;
    }
    if (index >= FILE_COUNT) return false;

    static const char *const suffixes[] = {
        "BRP", "PLD", "SAD", "EVE", "CSL",
    };
    const size_t session_index = index - 3;
    const char *suffix = suffixes[session_index / 2];
    const char *extension = (session_index & 1u) ? "crc" : "edf";
    const int path_length = snprintf(
        file.local_path, sizeof(file.local_path), "/DATALOG/%s/%s_%s.%s",
        journal.therapy_day, journal.file_prefix, suffix, extension);
    const int name_length = snprintf(file.name, sizeof(file.name), "%s_%s.%s",
                                     journal.file_prefix, suffix, extension);
    const int remote_length = snprintf(
        file.remote_path, sizeof(file.remote_path), "./DATALOG/%s/",
        journal.therapy_day);
    file.required = true;
    return path_length > 0 &&
           static_cast<size_t>(path_length) < sizeof(file.local_path) &&
           name_length > 0 &&
           static_cast<size_t>(name_length) < sizeof(file.name) &&
           remote_length > 0 &&
           static_cast<size_t>(remote_length) < sizeof(file.remote_path);
}

static bool delay_with_abort(BackgroundOperationControl &operation,
                             uint32_t delay_ms,
                             char *error, size_t error_size) {
    const uint32_t started = millis();
    while (static_cast<uint32_t>(millis() - started) < delay_ms) {
        if (!operation_allows(operation, error, error_size)) return false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return true;
}

static bool status_needs_process(const char *status) {
    return !status || !status[0] || strcasecmp(status, "created") == 0 ||
           strcasecmp(status, "new") == 0 ||
           strcasecmp(status, "ready") == 0 ||
           strcasecmp(status, "uploaded") == 0 ||
           strcasecmp(status, "uploading") == 0;
}

static bool finish_import(SdStorage::Session &storage, const StatePaths &paths,
                          const Journal &journal, Progress &progress,
                          ProgressCallback callback, void *callback_context,
                          char *error, size_t error_size) {
    const EdfCatalog::Entry entry = journal_entry(journal);
    if (!write_marker(storage, paths, entry)) {
        set_error(error, error_size, "done_marker_write");
        return false;
    }
    if (!remove_journal(storage, paths)) {
        set_error(error, error_size, "inflight_remove");
        return false;
    }
    progress.current_day[0] = 0;
    publish(progress, callback, callback_context);
    return true;
}

static bool fetch_import(SdStorage::Session &storage, const StatePaths &paths,
                         SleepHqClient &client, const Journal &journal,
                         BackgroundOperationControl &operation,
                         SleepHqImportInfo &info, char *error, size_t error_size) {
    if (client.get_import(journal.import_id, info, &operation)) return true;
    const bool missing = client.last_http_status() == 404 &&
        strcmp(client.last_error(), "http_404") == 0 &&
        strncmp(client.last_request(), "GET ", 4) == 0;
    if (!missing) {
        set_error(error, error_size, client.last_error());
        return false;
    }
    if (!remove_journal(storage, paths)) {
        set_error(error, error_size, "inflight_remove");
        return false;
    }
    // Only the remote handle is invalidated; EDF files and done markers stay.
    set_error(error, error_size, "import_missing");
    return false;
}

static bool poll_import(SdStorage::Session &storage, const StatePaths &paths,
                        SleepHqClient &client, Journal &journal,
                        BackgroundOperationControl &operation,
                        Progress &progress, ProgressCallback callback,
                        void *callback_context,
                        char *error, size_t error_size) {
    const uint32_t started = millis();
    bool process_requested = false;
    while (true) {
        progress.phase = "import_status";
        progress.file[0] = 0;
        if (!operation_allows(operation, error, error_size)) return false;

        SleepHqImportInfo info;
        if (!fetch_import(storage, paths, client, journal, operation, info,
                          error, error_size)) return false;
        operation.note_progress(millis());
        progress.import_id = journal.import_id;
        copy_cstr(progress.import_status, sizeof(progress.import_status),
                  info.status);
        publish(progress, callback, callback_context);

        const SleepHqImportStatusKind kind =
            sleephq_classify_import_status(info.status);
        if (kind == SleepHqImportStatusKind::Success)
            return finish_import(storage, paths, journal, progress, callback,
                                 callback_context, error, error_size);
        if (kind == SleepHqImportStatusKind::Failure) {
            Log::logf(CAT_EXPORT, LOG_WARN, "[SLEEPHQ] Import %lu rejected: %.80s\n",
                      (unsigned long)journal.import_id, info.failed_reason);
            set_error(error, error_size, remove_journal(storage, paths)
                ? "import_failed" : "inflight_remove");
            return false;
        }

        if (!process_requested && status_needs_process(info.status)) {
            progress.phase = "process_import";
            if (!client.process_import(journal.import_id, &operation)) {
                set_error(error, error_size, client.last_error());
                return false;
            }
            operation.note_progress(millis());
            process_requested = true;
        }

        if (static_cast<uint32_t>(millis() - started) >=
            IMPORT_POLL_TIMEOUT_MS) {
            set_error(error, error_size, "import_process_timeout");
            return false;
        }
        if (!delay_with_abort(operation, IMPORT_POLL_INTERVAL_MS,
                              error, error_size)) return false;
    }
}

static bool run_journal(SdStorage::Session &storage, const StatePaths &paths,
                        SleepHqClient &client, Journal &journal,
                        BackgroundOperationControl &operation,
                        Progress &progress, ProgressCallback callback,
                        void *callback_context,
                        char *error, size_t error_size) {
    copy_cstr(progress.current_day, sizeof(progress.current_day),
              journal.therapy_day);
    progress.import_id = journal.import_id;
    publish(progress, callback, callback_context);

    if (journal.phase == PHASE_UPLOADING) {
        progress.phase = "upload";
        for (size_t index = 0; index < FILE_COUNT; index++) {
            const uint16_t bit = static_cast<uint16_t>(1u << index);
            if (journal.uploaded_mask & bit) continue;
            if (!operation_allows(operation, error, error_size)) return false;

            FileSpec spec;
            if (!build_file_spec(index, journal, spec)) {
                set_error(error, error_size, "local_path_build");
                return false;
            }
            copy_cstr(progress.file, sizeof(progress.file), spec.local_path);
            progress.files_seen++;
            publish(progress, callback, callback_context);

            SdStorage::Reader input;
            if (!input.open(storage, spec.local_path)) {
                if (spec.required) {
                    snprintf(error, error_size, "local_missing:%s",
                             spec.local_path);
                    return false;
                }
                journal.uploaded_mask |= bit;
                progress.files_skipped++;
                if (!write_journal(storage, paths, journal)) {
                    set_error(error, error_size, "inflight_write");
                    return false;
                }
                continue;
            }
            const uint64_t file_size = input.size();
            input.close();

            FileReader reader = {};
            reader.storage = &storage;
            reader.size = file_size;
            copy_cstr(reader.path, sizeof(reader.path), spec.local_path);
            if (!reader.open()) {
                set_error(error, error_size, "local_open");
                return false;
            }

            SleepHqUploadRequest request;
            request.import_id = journal.import_id;
            request.name = spec.name;
            request.path = spec.remote_path;
            request.size = file_size;
            request.read = FileReader::read;
            request.reset = FileReader::reset;
            request.ctx = &reader;
            request.operation = &operation;
            SleepHqUploadResult result;
            const bool uploaded = client.upload_file(request, result);
            reader.file.close();
            if (!uploaded) {
                set_error(error, error_size, client.last_error());
                return false;
            }

            journal.uploaded_mask |= bit;
            progress.files_uploaded++;
            progress.bytes_uploaded += result.bytes;
            operation.note_progress(millis());
            if (!write_journal(storage, paths, journal)) {
                set_error(error, error_size, "inflight_write");
                return false;
            }
            publish(progress, callback, callback_context);
            Log::logf(CAT_EXPORT, LOG_DEBUG,
                      "[SLEEPHQ] uploaded %s bytes=%llu import=%lu\n",
                      spec.local_path,
                      static_cast<unsigned long long>(result.bytes),
                      static_cast<unsigned long>(journal.import_id));
        }

        journal.phase = PHASE_PROCESSING;
        if (!write_journal(storage, paths, journal)) {
            set_error(error, error_size, "inflight_write");
            return false;
        }
    }

    return poll_import(storage, paths, client, journal, operation,
                       progress, callback, callback_context,
                       error, error_size);
}

static bool same_entry(const Journal &journal,
                       const EdfCatalog::Entry &entry) {
    return strcmp(journal.file_prefix, entry.file_prefix) == 0 &&
           journal.finalized_epoch == entry.finalized_epoch &&
           journal.str_revision == entry.str_revision &&
           journal.flags == entry.flags;
}

}  // namespace

bool configured(const Config &config) {
    return config.client_id[0] && config.client_secret[0];
}

bool check_account(const Config &config, BackgroundOperationControl &operation,
                   char *error, size_t error_size) {
    uint32_t team_id = 0;
    struct Selection { uint32_t id; bool found; } selection = {};
    if ((config.team_id[0] && (!aircannect::parse_uint32_decimal(config.team_id, team_id) || !team_id)) ||
        (config.device_id[0] && (!aircannect::parse_uint32_decimal(config.device_id, selection.id) || !selection.id))) {
        set_error(error, error_size, "invalid_team_or_device_id");
        return false;
    }
    SleepHqClient client;
    bool success = client.configure(config) &&
        client.resolve_team_id(team_id, &operation);
    bool more = true;
    for (uint32_t page = 1; success && more && !selection.found; ++page) {
        size_t count = 0;
        success = client.list_team_machines(team_id, page, selection.id ? 25 : 1,
            [](void *ctx, const aircannect::SleepHqMachine &machine) {
                auto &selected = *static_cast<Selection *>(ctx);
                if (machine.id == selected.id) selected.found = true;
                return true;
            }, &selection, count, more, &operation);
        if (!selection.id) break;
    }
    if (!success) set_error(error, error_size, client.last_error());
    else if (selection.id && !selection.found) {
        success = false;
        set_error(error, error_size, "device_not_in_team");
    }
    client.disconnect();
    return success;
}

bool complete(SdStorage::Session &storage, const Config &config,
              const EdfCatalog::Entry &entry) {
    StatePaths paths = {};
    return configured(config) && build_state_paths(config, paths) &&
           marker_valid(storage, paths, entry);
}

bool pending_files(SdStorage::Session &storage, const Config &config,
                   const EdfCatalog::Entry &entry, uint32_t &files,
                   bool &incomplete, char *error, size_t error_size) {
    files = 0;
    incomplete = false;
    StatePaths paths = {};
    if (!configured(config) || !build_state_paths(config, paths)) {
        set_error(error, error_size, "not_configured");
        return false;
    }
    if (marker_valid(storage, paths, entry)) return true;
    incomplete = true;
    Journal journal = {};
    const auto loaded = load_journal(storage, paths, journal, false);
    if (loaded == JournalLoad::Unavailable || loaded == JournalLoad::Invalid) {
        set_error(error, error_size, loaded == JournalLoad::Invalid
            ? "inflight_invalid" : "storage_unavailable");
        return false;
    }
    uint16_t uploaded = 0;
    if (loaded == JournalLoad::Valid && strcmp(journal.file_prefix, entry.file_prefix) == 0) {
        if (!same_entry(journal, entry)) {
            set_error(error, error_size, "inflight_revision_changed");
            return false;
        }
        uploaded = journal.uploaded_mask;
    } else if (marker_valid(storage, paths, entry, false)) {
        uploaded = ((1u << FILE_COUNT) - 1) & ~7u;
    }
    for (size_t i = 0; i < FILE_COUNT; ++i)
        if (!(uploaded & (1u << i))) ++files;
    return true;
}

Run::Run(const Config &config) : config_(config) {
    client_.configure(config_);
}

bool Run::sync_session(SdStorage::Session &storage,
                  const EdfCatalog::Entry &entry,
                  BackgroundOperationControl &operation,
                  Progress &progress, ProgressCallback callback,
                  void *callback_context, char *error, size_t error_size,
                  BacklogChanges *changes) {
    if (changes) *changes = {};
    const Config &config = config_;
    SleepHqClient &client = client_;
    progress.phase = "journal";
    progress.file[0] = 0;
    if (!operation_allows(operation, error, error_size)) return false;
    if (!configured(config)) {
        set_error(error, error_size, "not_configured");
        return false;
    }

    StatePaths paths = {};
    if (!build_state_paths(config, paths) ||
        !storage.run([&](fs::FS &fs) { return ensure_state_directory(fs, paths); })) {
        set_error(error, error_size, "state_directory");
        return false;
    }

    Journal journal = {};
    const JournalLoad loaded = load_journal(storage, paths, journal);
    if (loaded == JournalLoad::Unavailable) {
        set_error(error, error_size, "storage_unavailable");
        return false;
    }
    if (loaded == JournalLoad::Invalid) {
        set_error(error, error_size, "inflight_invalid");
        return false;
    } else if (loaded == JournalLoad::Valid) {
        const bool requested_entry = same_entry(journal, entry);
        if (changes) {
            changes->entry = requested_entry;
            changes->other_entries = !requested_entry;
        }
        progress.import_id = journal.import_id;
        progress.phase = "resume_import";
        progress.import_status[0] = 0;
        if (journal.phase == PHASE_UPLOADING) {
            SleepHqImportInfo info;
            if (!fetch_import(storage, paths, client, journal, operation, info,
                              error, error_size)) return false;
            const auto kind = sleephq_classify_import_status(info.status);
            const bool all_uploaded = journal.uploaded_mask == (1u << FILE_COUNT) - 1;
            if (kind == SleepHqImportStatusKind::Failure ||
                (!status_needs_process(info.status) && !all_uploaded)) {
                const char *reason = kind == SleepHqImportStatusKind::Failure
                    ? "import_failed" : "import_closed_before_upload";
                set_error(error, error_size, remove_journal(storage, paths)
                    ? reason : "inflight_remove");
                return false;
            }
        }
        if (!run_journal(storage, paths, client, journal, operation,
                         progress, callback, callback_context,
                         error, error_size)) {
            return false;
        }
        if (requested_entry) {
            return true;
        }
    }

    if (marker_valid(storage, paths, entry)) {
        progress.files_skipped += FILE_COUNT;
        publish(progress, callback, callback_context);
        return true;
    }

    progress.phase = "resolve_team";
    progress.import_id = 0;
    progress.import_status[0] = 0;
    if (!team_id_ && !client.resolve_team_id(team_id_, &operation)) {
        set_error(error, error_size, client.last_error());
        return false;
    }
    operation.note_progress(millis());

    SleepHqImportInfo import;
    progress.phase = "create_import";
    if (!client.create_import(team_id_, import, &operation) || !import.id) {
        set_error(error, error_size,
                  client.last_error()[0] ? client.last_error()
                                         : "import_id_missing");
        return false;
    }
    operation.note_progress(millis());

    journal = {};
    copy_cstr(progress.import_status, sizeof(progress.import_status), import.status);
    // A new STR generation does not invalidate the immutable detailed files.
    if (marker_valid(storage, paths, entry, false)) {
        journal.uploaded_mask = ((1u << FILE_COUNT) - 1) & ~7u;
        progress.files_skipped += FILE_COUNT - 3;
    }
    journal.phase = PHASE_UPLOADING;
    journal.flags = entry.flags;
    journal.import_id = import.id;
    journal.team_id = team_id_;
    journal.finalized_epoch = entry.finalized_epoch;
    journal.str_revision = entry.str_revision;
    copy_cstr(journal.therapy_day, sizeof(journal.therapy_day),
              entry.therapy_day);
    copy_cstr(journal.file_prefix, sizeof(journal.file_prefix),
              entry.file_prefix);
    if (changes) changes->entry = true;
    if (!write_journal(storage, paths, journal)) {
        set_error(error, error_size, "inflight_write");
        return false;
    }

    Log::logf(CAT_EXPORT, LOG_DEBUG,
              "[SLEEPHQ] import created id=%lu day=%s\n",
              static_cast<unsigned long>(journal.import_id),
              journal.therapy_day);
    const bool success = run_journal(
        storage, paths, client, journal, operation, progress,
        callback, callback_context, error, error_size);
    return success;
}

}  // namespace SleepHqSync
