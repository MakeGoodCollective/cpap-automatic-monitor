#include "sleephq_sync.h"
#include "sd_storage.h"
#include <FS.h>

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <esp_rom_md5.h>
#include <stdlib.h>
#include <new>
#include <memory>

#include "crc.h"
#include "hex_util.h"
#include "debug_log.h"
#include "sleephq_client.h"
#include "string_util.h"
#include "memory_manager.h"
#include "large_text_buffer.h"

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
    if (!storage.exists(paths.journal) &&
        storage.exists(paths.journal_backup)) {
        (void)storage.rename(paths.journal_backup, paths.journal);
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

static bool day_confirmed(SdStorage::Session &storage, const Config &config,
                          const char *day);

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
           marker_valid(storage, paths, entry) && day_confirmed(storage, config, entry.therapy_day);
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
    if (complete(storage, config, entry)) return true;
    incomplete = true;
    Journal journal = {};
    const auto loaded = load_journal(storage, paths, journal, false);
    if (loaded == JournalLoad::Unavailable || loaded == JournalLoad::Invalid) {
        set_error(error, error_size, loaded == JournalLoad::Invalid
            ? "inflight_invalid" : "storage_unavailable");
        return false;
    }
    files = marker_valid(storage, paths, entry) ? 0 :
        marker_valid(storage, paths, entry, false) ? 3 : FILE_COUNT;
    return true;
}

Run::Run(const Config &config) : config_(config) {
    client_.configure(config_);
}

bool Run::resume(SdStorage::Session &storage, BackgroundOperationControl &operation,
                 Progress &progress, ProgressCallback callback, void *context,
                 char *error, size_t error_size, BacklogChanges *changes) {
    if (changes) *changes = {};
    StatePaths paths;
    if (!build_state_paths(config_, paths)) return false;
    Journal legacy = {};
    const auto loaded = load_journal(storage, paths, legacy);
    if (loaded == JournalLoad::Missing) return true;
    if (loaded != JournalLoad::Valid) {
        set_error(error, error_size, loaded == JournalLoad::Invalid
            ? "inflight_invalid" : "storage_unavailable");
        return false;
    }
    return sync_day(storage, legacy.therapy_day, false, operation, progress,
                    callback, context, error, error_size, changes);
}


namespace {

using aircannect::Memory::free;
using aircannect::SleepHqRemoteFile;

// One day's inventory; large or unusual days require PSRAM, not internal heap.
struct DayFile {
    char name[80] = {};
    char hash[33] = {};
    char day[9] = {};
    uint64_t size = 0;
    bool root = false;
    bool local = false;
    bool present = false;
    bool available = false;
};

struct DayFiles {
    DayFile *items = nullptr;
    size_t count = 0, capacity = 0;
    ~DayFiles() { free(items); }

    DayFile *find(const char *name, const char *hash = nullptr) {
        for (size_t i = 0; i < count; ++i)
            if (!strcmp(items[i].name, name) &&
                (!hash || !strcmp(items[i].hash, hash))) return &items[i];
        return nullptr;
    }

    bool append(const DayFile &file) {
        if (count == capacity) {
            const size_t next = capacity ? capacity * 2 : 16;
            if (next > 4096) return false;
            void *grown = aircannect::Memory::realloc_large(
                items, next * sizeof(DayFile), next * sizeof(DayFile) <= 8192);
            if (!grown) return false;
            items = static_cast<DayFile *>(grown);
            capacity = next;
        }
        items[count++] = file;
        return true;
    }
};

struct DaySnapshot {
    EdfCatalog::Entry *entries = nullptr;
    uint32_t count = 0;
    DayFiles files;
    ~DaySnapshot() { free(entries); }
};

struct DayState {
    uint32_t import_id = 0, team_id = 0, fingerprint = 0;
    uint8_t phase = 0; // 0 confirmed, 1 preparing files, 2 process submitted/unknown
    char processed_at[32] = {};
};

static bool day_paths(const Config &config, const char *day, StatePaths &paths) {
    if (!day || strlen(day) != 8 || !valid_digits(day, 8) ||
        !build_state_paths(config, paths)) return false;
    snprintf(paths.journal, sizeof(paths.journal), "%s/day_%s.bin", paths.directory, day);
    snprintf(paths.journal_part, sizeof(paths.journal_part), "%s.part", paths.journal);
    snprintf(paths.journal_backup, sizeof(paths.journal_backup), "%s.bak", paths.journal);
    return true;
}

static JournalLoad load_day(SdStorage::Session &storage, const StatePaths &paths,
                            const char *day, DayState &state, bool recover = true) {
    JournalLoad result = JournalLoad::Missing;
    const bool ok = storage.run([&](fs::FS &fs) {
        if (recover) recover_journal(fs, paths);
        const char *path = !recover && !fs.exists(paths.journal)
            ? paths.journal_backup : paths.journal;
        if (!fs.exists(path)) return true;
        fs::File f = fs.open(path, FILE_READ);
        uint8_t b[64];
        const bool valid = f && f.size() == sizeof(b) &&
            f.read(b, sizeof(b)) == sizeof(b) &&
            !memcmp(b, "ABHD", 4) && b[4] == 1 && b[5] <= 2 &&
            !memcmp(b + 52, day, 8) && b[51] == 0 &&
            SdStorage::get_le32(b + 60) == crc32_ieee(b, 60);
        result = valid ? JournalLoad::Valid : JournalLoad::Invalid;
        if (valid) {
            state.phase = b[5];
            state.import_id = SdStorage::get_le32(b + 8);
            state.team_id = SdStorage::get_le32(b + 12);
            state.fingerprint = SdStorage::get_le32(b + 16);
            memcpy(state.processed_at, b + 20, sizeof(state.processed_at));
        }
        return true;
    });
    return ok ? result : JournalLoad::Unavailable;
}

static bool day_confirmed(SdStorage::Session &storage, const Config &config,
                          const char *day) {
    StatePaths paths;
    DayState state;
    return day_paths(config, day, paths) &&
        load_day(storage, paths, day, state, false) == JournalLoad::Valid &&
        state.phase == 0 && state.import_id && state.team_id;
}

static bool save_day(SdStorage::Session &storage, const StatePaths &paths,
                      const char *day, const DayState &state) {
    uint8_t b[64] = {};
    memcpy(b, "ABHD", 4);
    b[4] = 1;
    b[5] = state.phase;
    SdStorage::put_le32(b + 8, state.import_id);
    SdStorage::put_le32(b + 12, state.team_id);
    SdStorage::put_le32(b + 16, state.fingerprint);
    memcpy(b + 20, state.processed_at, sizeof(state.processed_at));
    memcpy(b + 52, day, 8);
    SdStorage::put_le32(b + 60, crc32_ieee(b, 60));
    return storage.run([&](fs::FS &fs) {
        return ensure_state_directory(fs, paths) &&
            replace_file(fs, paths.journal, paths.journal_part,
                         paths.journal_backup, b, sizeof(b));
    });
}

static void file_path(const char *day, const DayFile &file, char *out, size_t size) {
    if (file.root) snprintf(out, size, "/%s", file.name);
    else snprintf(out, size, "/DATALOG/%s/%s", day, file.name);
}

static bool hash_file(SdStorage::Session &storage, const char *day, DayFile &file,
                      BackgroundOperationControl &operation, char *error, size_t error_size) {
    char path[128];
    file_path(day, file, path, sizeof(path));
    SdStorage::Reader reader;
    if (!reader.open(storage, path)) {
        snprintf(error, error_size, "local_missing:%s", path);
        return false;
    }
    file.size = reader.size();
    md5_context_t md5;
    esp_rom_md5_init(&md5);
    std::unique_ptr<uint8_t, decltype(&aircannect::Memory::free)> buffer(
        static_cast<uint8_t *>(aircannect::Memory::alloc_large(SdStorage::READ_CHUNK_BYTES)),
        aircannect::Memory::free);
    if (!buffer) {
        set_error(error, error_size, "hash_buffer_alloc");
        return false;
    }
    uint64_t remaining = file.size;
    while (remaining) {
        if (!operation_allows(operation, error, error_size)) return false;
        const size_t n = remaining < SdStorage::READ_CHUNK_BYTES ? remaining : SdStorage::READ_CHUNK_BYTES;
        if (reader.read(buffer.get(), n) != n) {
            set_error(error, error_size, "source_read_failed");
            return false;
        }
        esp_rom_md5_update(&md5, buffer.get(), n);
        remaining -= n;
        operation.note_progress(millis());
    }
    reader.close();
    esp_rom_md5_update(&md5, reinterpret_cast<const uint8_t *>(file.name), strlen(file.name));
    uint8_t digest[16];
    esp_rom_md5_final(digest, &md5);
    return aircannect::hex_encode(digest, sizeof(digest), file.hash,
                                  sizeof(file.hash), aircannect::HexCase::Lower);
}

static bool snapshot_day(SdStorage::Session &storage, const char *day,
                          DaySnapshot &snapshot, char *error, size_t error_size) {
    if (!storage.run([&](fs::FS &) {
            return EdfCatalog::snapshot_day(day, snapshot.entries, snapshot.count);
        })) {
        set_error(error, error_size, storage.valid() ? "catalog_read" : "storage_unavailable");
        return false;
    }
    if (!snapshot.count) {
        set_error(error, error_size, "day_not_found");
        return false;
    }
    qsort(snapshot.entries, snapshot.count, sizeof(EdfCatalog::Entry),
        [](const void *a, const void *b) {
            return strcmp(static_cast<const EdfCatalog::Entry *>(a)->file_prefix,
                          static_cast<const EdfCatalog::Entry *>(b)->file_prefix);
        });
    for (uint32_t i = 0; i < snapshot.count; ++i) {
        const auto &entry = snapshot.entries[i];
        const uint8_t ready = EdfCatalog::ENTRY_LIVE_COMPLETE |
            EdfCatalog::ENTRY_IDENTIFICATION_READY | EdfCatalog::ENTRY_STR_READY;
        if ((entry.flags & ready) != ready) {
            set_error(error, error_size, "day_not_ready");
            return false;
        }
        Journal legacy = {};
        copy_cstr(legacy.therapy_day, sizeof(legacy.therapy_day), day);
        copy_cstr(legacy.file_prefix, sizeof(legacy.file_prefix), entry.file_prefix);
        legacy.flags = entry.flags;
        for (size_t index = i ? 3 : 0; index < FILE_COUNT; ++index) {
            FileSpec spec;
            if (!build_file_spec(index, legacy, spec)) return false;
            DayFile file;
            copy_cstr(file.name, sizeof(file.name), spec.name);
            file.root = index < 3;
            file.local = true;
            if (!snapshot.files.append(file)) {
                set_error(error, error_size, "day_inventory_alloc");
                return false;
            }
        }
    }
    return true;
}

struct RemoteDay {
    const char *day;
    DayFiles files;
    bool other_day = false;
    bool has_day = false;
    bool overflow = false;
    bool invalid = false;

    static bool receive(void *ctx, const SleepHqRemoteFile &remote) {
        auto &self = *static_cast<RemoteDay *>(ctx);
        const char *path = remote.path;
        while (*path == '.' || *path == '/') ++path;
        const bool root = !*path;
        const bool root_file = root && (!strcmp(remote.name, "Identification.tgt") ||
            !strcmp(remote.name, "Identification.crc") || !strcmp(remote.name, "STR.edf"));
        const bool detail = strlen(path) == 17 && !strncmp(path, "DATALOG/", 8) &&
            valid_digits(path + 8, 8) && path[16] == '/';
        const bool selected = detail && (!self.day || !strncmp(path + 8, self.day, 8));
        if (!strncmp(path, "DATALOG/", 8) && !selected) self.other_day = true;
        if (!root_file && !selected) return true;
        if (selected) self.has_day = true;
        const bool invalid = !remote.name[0] || strlen(remote.name) >= sizeof(DayFile::name) ||
            strpbrk(remote.name, "/\\") || !strcmp(remote.name, ".") ||
            !strcmp(remote.name, "..") || !remote.id ||
            !aircannect::hex_text_valid(remote.content_hash, 32);
        if (invalid && (self.day || root)) { self.invalid = true; return false; }
        DayFile file;
        // Keep a rejected day's presence in a whole-account scan, but only
        // reject the run if that day is actually selected for reconciliation.
        if (!invalid) {
            copy_cstr(file.name, sizeof(file.name), remote.name);
            copy_cstr(file.hash, sizeof(file.hash), remote.content_hash);
        }
        file.size = remote.size;
        file.root = root;
        file.available = !invalid;
        if (detail) memcpy(file.day, path + 8, 8);
        if (!self.files.append(file)) { self.overflow = true; return false; }
        return true;
    }
};

static bool read_remote(SleepHqClient &client, uint32_t id, RemoteDay &remote,
                         BackgroundOperationControl &operation) {
    bool more = true;
    for (uint32_t page = 1; more; ++page) {
        size_t count = 0;
        if (!client.list_import_files(id, page, RemoteDay::receive, &remote,
                                      count, more, &operation)) return false;
        operation.note_progress(millis());
    }
    return true;
}

static bool same_device(RemoteDay &remote, DayFiles &local) {
    const DayFile *ident = local.find("Identification.tgt");
    for (size_t i = 0; i < remote.files.count; ++i) {
        const auto &file = remote.files.items[i];
        if (ident && file.root && !strcmp(file.name, ident->name) &&
            !strcmp(file.hash, ident->hash)) return true;
    }
    return false;
}

static bool merge_remote(DayFiles &desired, const DayFiles &remote,
                          char *error, size_t error_size) {
    for (size_t i = 0; i < remote.count; ++i) {
        const auto &file = remote.items[i];
        DayFile *known = desired.find(file.name);
        if (known && !strcmp(known->hash, file.hash) && known->size == file.size)
            known->available = true;
        if (file.root) continue;
        if (known && known->local) continue; // Local snapshot selects the current version.
        if (known && strcmp(known->hash, file.hash)) {
            set_error(error, error_size, "remote_file_conflict");
            return false;
        }
        if (!known && !desired.append(file)) {
            set_error(error, error_size, "day_inventory_alloc");
            return false;
        }
    }
    return true;
}

static bool missing_import(const SleepHqClient &client) {
    const char *request = client.last_request();
    const char *id = request + (strlen(request) < 20 ? strlen(request) : 20);
    return client.last_http_status() == 404 &&
        !strcmp(client.last_error(), "http_404") &&
        !strncmp(request, "GET /api/v1/imports/", 20) &&
        id[0] && valid_digits(id, strlen(id));
}

static bool wait_import(SleepHqClient &client, uint32_t id,
                         BackgroundOperationControl &operation, SleepHqImportInfo &info,
                         const char *previous, Progress &progress, ProgressCallback callback,
                         void *context, char *error, size_t error_size) {
    const uint32_t started = millis();
    for (;;) {
        if (!operation_allows(operation, error, error_size)) return false;
        if (!client.get_import(id, info, &operation)) {
            set_error(error, error_size, client.last_error());
            return false;
        }
        operation.note_progress(millis());
        copy_cstr(progress.import_status, sizeof(progress.import_status), info.status);
        publish(progress, callback, context);
        const auto kind = aircannect::sleephq_classify_import_status(info.status);
        if (kind == SleepHqImportStatusKind::Failure) {
            if (!previous) return true; // An idle failed import can be repaired.
            if (strcmp(previous, info.processed_at)) {
                set_error(error, error_size, "import_failed");
                return false;
            }
        }
        if (kind == SleepHqImportStatusKind::Success &&
            (!previous || (info.processed_at[0] && strcmp(previous, info.processed_at))))
            return true;
        if (!previous && status_needs_process(info.status)) return true;
        if (millis() - started >= IMPORT_POLL_TIMEOUT_MS) {
            set_error(error, error_size, "import_process_timeout");
            return false;
        }
        if (!delay_with_abort(operation, IMPORT_POLL_INTERVAL_MS, error, error_size)) return false;
    }
}

static uint32_t manifest_fingerprint(DayFiles &files) {
    qsort(files.items, files.count, sizeof(DayFile), [](const void *a, const void *b) {
        return strcmp(static_cast<const DayFile *>(a)->name, static_cast<const DayFile *>(b)->name);
    });
    uint32_t crc = crc32_ieee_initial();
    for (size_t i = 0; i < files.count; ++i) {
        const auto &f = files.items[i];
        crc = crc32_ieee_update(crc, reinterpret_cast<const uint8_t *>(f.name), strlen(f.name) + 1);
        crc = crc32_ieee_update(crc, reinterpret_cast<const uint8_t *>(f.hash), 32);
    }
    return crc32_ieee_finish(crc);
}

static bool verify_source(const SdStorage::Session &storage,
                           BackgroundOperationControl &operation,
                           char *error, size_t error_size) {
    if (!operation_allows(operation, error, error_size)) return false;
    // The captured SD lease excludes writers; recorder acquisition revokes it.
    if (!storage.valid()) {
        set_error(error, error_size, "source_changed");
        return false;
    }
    return true;
}

}  // namespace

// One bounded, parsed inventory for the entire catalog pass. These records
// describe observed objects, never prove current target membership or success.
struct Run::Inventory {
    struct Import { uint32_t id, first, count; bool other_day; };
    aircannect::LargeTextBuffer imports{4096 * sizeof(Import), 2048};
    aircannect::LargeTextBuffer files{16384 * sizeof(DayFile), 8192};
    aircannect::LargeTextBuffer stored{16 * sizeof(DayFile), 4096};
    bool ready = false;

    bool load(SleepHqClient &client, uint32_t team, BackgroundOperationControl &operation,
              char *error, size_t error_size) {
        if (ready) return true;
        const uint32_t started = millis();
        // An incomplete scan is not a reusable inventory after a failed attempt.
        imports.clear(); files.clear();
        struct Page { uint32_t ids[16]; size_t count; uint32_t team; };
        bool more = true;
        for (uint32_t page = 1; more; ++page) {
            Page list = {{}, 0, team};
            size_t count = 0;
            if (!client.list_imports(team, page,
                [](void *ctx, const SleepHqImportInfo &item) {
                    auto &p = *static_cast<Page *>(ctx);
                    if (item.team_id != p.team) return true;
                    if (p.count == 16) return false;
                    p.ids[p.count++] = item.id;
                    return true;
                }, &list, count, more, &operation)) {
                set_error(error, error_size, client.last_error());
                return false;
            }
            for (size_t i = 0; i < list.count; ++i) {
                RemoteDay remote{nullptr};
                if (!read_remote(client, list.ids[i], remote, operation)) {
                    if (missing_import(client)) continue;
                    set_error(error, error_size, remote.overflow ? "remote_inventory_alloc" :
                        remote.invalid ? "remote_inventory_invalid" : client.last_error());
                    return false;
                }
                Import import{list.ids[i], uint32_t(files.length() / sizeof(DayFile)),
                              uint32_t(remote.files.count), remote.other_day};
                if (!files.append(reinterpret_cast<const char *>(remote.files.items),
                                  remote.files.count * sizeof(DayFile)) ||
                    !imports.append(reinterpret_cast<const char *>(&import), sizeof(import))) {
                    set_error(error, error_size, files.overflowed() || imports.overflowed()
                        ? "remote_inventory_limit" : "remote_inventory_alloc");
                    return false;
                }
            }
        }
        ready = true;
        Log::logf(CAT_EXPORT, LOG_DEBUG, "[SLEEPHQ] Discovered imports=%u files=%u elapsed=%lums\n",
                  unsigned(imports.length() / sizeof(Import)), unsigned(files.length() / sizeof(DayFile)),
                  (unsigned long)(millis() - started));
        return true;
    }

    bool select(size_t offset, const char *day, RemoteDay &out, uint32_t &id) const {
        Import import;
        memcpy(&import, imports.c_str() + offset, sizeof(import));
        id = import.id;
        out.other_day = import.other_day;
        for (size_t i = 0; i < import.count; ++i) {
            DayFile file;
            memcpy(&file, files.c_str() + (import.first + i) * sizeof(file), sizeof(file));
            if (!file.root && strcmp(file.day, day)) { out.other_day = true; continue; }
            if (!file.root) out.has_day = true;
            if (!file.available) { out.invalid = true; return false; }
            if (!out.files.append(file)) return false;
        }
        return true;
    }

    bool known(const DayFile &file) {
        for (const auto *list : {&stored, &files})
            for (size_t offset = 0; offset < list->length(); offset += sizeof(DayFile)) {
                DayFile cached;
                memcpy(&cached, list->c_str() + offset, sizeof(cached));
                if (cached.available && cached.size == file.size && !strcmp(cached.name, file.name) &&
                    !strcmp(cached.hash, file.hash)) return true;
            }
        return false;
    }
};

Run::~Run() {
    if (inventory_) {
        inventory_->~Inventory();
        free(inventory_);
    }
}

bool Run::sync_day(SdStorage::Session &storage, const char *day, bool repair,
                  BackgroundOperationControl &operation,
                  Progress &progress, ProgressCallback callback, void *context,
                  char *error, size_t error_size, BacklogChanges *changes) {
    if (changes) *changes = {};
    if (!operation_allows(operation, error, error_size)) return false;
    StatePaths paths, legacy_paths;
    if (!configured(config_) || !day_paths(config_, day, paths) ||
        !build_state_paths(config_, legacy_paths)) {
        set_error(error, error_size, "invalid_day_or_config");
        return false;
    }
    const SdStorage::Session source = storage;
    DaySnapshot snapshot;
    if (!snapshot_day(storage, day, snapshot, error, error_size)) return false;
    DayState state;
    const auto loaded = load_day(storage, paths, day, state);
    Journal legacy = {};
    const auto old = load_journal(storage, legacy_paths, legacy);
    if (loaded == JournalLoad::Invalid || old == JournalLoad::Invalid) {
        set_error(error, error_size, "inflight_invalid");
        return false;
    }
    if (loaded == JournalLoad::Unavailable || old == JournalLoad::Unavailable) {
        set_error(error, error_size, "storage_unavailable");
        return false;
    }
    const bool migrating = old == JournalLoad::Valid && !strcmp(legacy.therapy_day, day);
    const bool unfinished = loaded == JournalLoad::Valid && state.phase != 0;
    const bool reconcile_objects = repair || migrating || unfinished;
    bool confirmed = loaded == JournalLoad::Valid && state.phase == 0 && state.import_id;
    for (uint32_t i = 0; confirmed && i < snapshot.count; ++i)
        confirmed = marker_valid(storage, legacy_paths, snapshot.entries[i]);
    if (confirmed && !repair && !migrating) return true;
    const uint32_t uploaded_before = progress.files_uploaded;
    const uint32_t present_before = progress.files_skipped;
    const uint32_t attached_before = progress.files_attached;
    if (!inventory_) {
        void *memory = aircannect::Memory::alloc_large(sizeof(Inventory));
        if (!memory) { set_error(error, error_size, "remote_inventory_alloc"); return false; }
        inventory_ = new (memory) Inventory;
    }

    if (state.phase == 0) state.phase = 1;
    if (!save_day(storage, paths, day, state)) {
        set_error(error, error_size, "inflight_write");
        return false;
    }

    if (changes) changes->other_entries = true;
    copy_cstr(progress.current_day, sizeof(progress.current_day), day);
    progress.phase = "day_inventory";
    publish(progress, callback, context);
    for (size_t i = 0; i < snapshot.files.count; ++i)
        if (!hash_file(storage, day, snapshot.files.items[i], operation, error, error_size))
            return false;
    if (!team_id_ && !client_.resolve_team_id(team_id_, &operation)) {
        set_error(error, error_size, client_.last_error());
        return false;
    }
    if (state.import_id && state.team_id != team_id_) {
        set_error(error, error_size, "import_team_changed");
        return false;
    }
    if (!state.import_id && migrating && legacy.team_id == team_id_) {
        state.import_id = legacy.import_id;
        state.phase = 1;
    }
    SleepHqImportInfo info;
    if (state.import_id) {
        if (!client_.get_import(state.import_id, info, &operation)) {
            if (!missing_import(client_)) {
                set_error(error, error_size, client_.last_error());
                return false;
            }
            state.import_id = 0;
            state.phase = 1;
        } else {
            RemoteDay remote{day};
            if (info.team_id != team_id_ ||
                !read_remote(client_, state.import_id, remote, operation)) {
                set_error(error, error_size, info.team_id != team_id_
                    ? "import_team_changed" : client_.last_error());
                return false;
            }
            // Never automatically rewrite a multi-day or another machine's import.
            if (remote.other_day || (remote.files.find("Identification.tgt") &&
                                    !same_device(remote, snapshot.files))) {
                state.import_id = 0;
                state.phase = 1;
            } else if (!merge_remote(snapshot.files, remote.files, error, error_size)) return false;
        }
    }

    // Find a day destination or recover remote state; confirmed days skip discovery.
    if (!state.import_id || reconcile_objects) {
        progress.phase = "discover_imports";
        if (!inventory_->load(client_, team_id_, operation, error, error_size)) return false;
        for (size_t offset = 0; offset < inventory_->imports.length(); offset += sizeof(Inventory::Import)) {
            RemoteDay remote{day};
            uint32_t id;
            if (!inventory_->select(offset, day, remote, id)) {
                set_error(error, error_size, remote.invalid ? "remote_inventory_invalid" : "day_inventory_alloc");
                return false;
            }
            if (!remote.has_day || !same_device(remote, snapshot.files)) continue;
            if (!merge_remote(snapshot.files, remote.files, error, error_size)) return false;
            if (!state.import_id && !remote.other_day) state.import_id = id;
        }
    }
    if (!state.import_id) {
        progress.phase = "create_import";
        if (!client_.create_import(team_id_, info, &operation)) {
            set_error(error, error_size, client_.last_error());
            return false;
        }
        state.import_id = info.id;
        state.phase = 1;
    }
    state.team_id = team_id_;
    progress.import_id = state.import_id;
    publish(progress, callback, context);
    // Persist the destination before adding any files. A lost ACK is recovered
    // from its remote inventory, rather than from an optimistic local bit mask.
    if (!save_day(storage, paths, day, state)) {
        set_error(error, error_size, "inflight_write");
        return false;
    }
    if (migrating && !remove_journal(storage, legacy_paths)) {
        set_error(error, error_size, "inflight_remove");
        return false;
    }
    progress.phase = "import_status";
    if (!wait_import(client_, state.import_id, operation, info, nullptr,
                     progress, callback, context, error, error_size)) return false;

    RemoteDay target{day};
    if (!read_remote(client_, state.import_id, target, operation)) {
        set_error(error, error_size, client_.last_error());
        return false;
    }
    if (target.other_day || (target.files.find("Identification.tgt") &&
                            !same_device(target, snapshot.files))) {
        set_error(error, error_size, "remote_inventory_changed");
        return false;
    }
    if (!merge_remote(snapshot.files, target.files, error, error_size)) return false;
    bool changed = false;
    for (size_t i = 0; i < snapshot.files.count; ++i) {
        auto &f = snapshot.files.items[i];
        const auto *remote = target.files.find(f.name, f.hash);
        f.present = remote && remote->size == f.size && remote->root == f.root;
        f.available |= f.present;
        changed |= !f.present;
    }
    const uint32_t fingerprint = manifest_fingerprint(snapshot.files);
    const bool recovered = state.phase == 2 && state.fingerprint == fingerprint &&
        sleephq_classify_import_status(info.status) == SleepHqImportStatusKind::Success &&
        info.processed_at[0] && strcmp(info.processed_at, state.processed_at) && !changed;
    if (!recovered) {
        // Reattach the desired STR last even if an older association with this
        // hash exists alongside other STR revisions in the remote import.
        if (auto *str = snapshot.files.find("STR.edf")) str->present = false;
        state.phase = 1;
        state.fingerprint = fingerprint;
        if (!save_day(storage, paths, day, state)) {
            set_error(error, error_size, "inflight_write");
            return false;
        }
        for (size_t i = 0; i < snapshot.files.count; ++i) {
            auto &f = snapshot.files.items[i];
            if (!operation_allows(operation, error, error_size)) return false;
            ++progress.files_seen;
            if (f.present) { ++progress.files_skipped; continue; }
            char remote_path[32];
            snprintf(remote_path, sizeof(remote_path), f.root ? "./" : "./DATALOG/%s/", day);
            file_path(day, f, progress.file, sizeof(progress.file));
            progress.phase = "attach";
            publish(progress, callback, context);
            aircannect::SleepHqAttachRequest attach;
            attach.import_id = state.import_id;
            attach.name = f.name;
            attach.path = remote_path;
            attach.content_hash = f.hash;
            SleepHqUploadResult result;
            struct HashMatch { const DayFile *file; bool found; } match{&f, f.available || inventory_->known(f)};
            // Normal day appends upload unknown objects without a per-file search.
            // Recovery can still find accepted objects missing their association.
            bool more = reconcile_objects && !match.found;
            for (uint32_t page = 1; more && !match.found; ++page) {
                size_t count = 0;
                if (!client_.list_team_files(team_id_, page, 100,
                    [](void *ctx, const SleepHqRemoteFile &remote) {
                        auto &m = *static_cast<HashMatch *>(ctx);
                        if (!strcmp(remote.name, m.file->name) &&
                            !strcmp(remote.content_hash, m.file->hash) && remote.size == m.file->size)
                            m.found = true;
                        return true;
                    }, &match, count, more, &operation, f.name)) {
                    set_error(error, error_size, client_.last_error());
                    return false;
                }
                operation.note_progress(millis());
            }
            if (match.found) {
                if (!client_.attach_file(attach, result, &operation)) {
                    set_error(error, error_size, client_.last_error());
                    return false;
                }
                ++progress.files_attached;
            } else {
                FileReader reader;
                reader.storage = &storage;
                reader.size = f.size;
                file_path(day, f, reader.path, sizeof(reader.path));
                if (!reader.open()) { set_error(error, error_size, "source_changed"); return false; }
                SleepHqUploadRequest upload;
                upload.import_id = state.import_id;
                upload.name = f.name;
                upload.path = remote_path;
                upload.content_hash = f.hash;
                upload.size = f.size;
                upload.read = FileReader::read;
                upload.reset = FileReader::reset;
                upload.ctx = &reader;
                upload.operation = &operation;
                progress.phase = "upload";
                const bool ok = client_.upload_file(upload, result);
                reader.file.close();
                if (!ok) { set_error(error, error_size, client_.last_error()); return false; }
                ++progress.files_uploaded;
                progress.bytes_uploaded += result.bytes;
            }
            // Only the shared roots can occur in subsequent local days.
            f.available = true;
            if (f.root && !inventory_->known(f) &&
                !inventory_->stored.append(reinterpret_cast<const char *>(&f), sizeof(f))) {
                set_error(error, error_size, "remote_inventory_alloc");
                return false;
            }
            operation.note_progress(millis());
            publish(progress, callback, context);
        }
        target.files.count = 0;
        target.other_day = target.has_day = false;
        if (!read_remote(client_, state.import_id, target, operation)) {
            set_error(error, error_size, client_.last_error());
            return false;
        }
        for (size_t i = 0; i < snapshot.files.count; ++i) {
            const auto &file = snapshot.files.items[i];
            const auto *remote = target.files.find(file.name, file.hash);
            if (target.other_day || !remote || remote->size != file.size ||
                remote->root != file.root) {
                set_error(error, error_size, "remote_inventory_changed");
                return false;
            }
        }
        if (!verify_source(source, operation, error, error_size)) return false;
        // Capture the server's generation, not the ESP wall clock.
        if (!client_.get_import(state.import_id, info, &operation)) {
            set_error(error, error_size, client_.last_error());
            return false;
        }
        copy_cstr(state.processed_at, sizeof(state.processed_at), info.processed_at);
        state.phase = 2;
        if (!save_day(storage, paths, day, state)) {
            set_error(error, error_size, "inflight_write");
            return false;
        }
        progress.phase = "process_import";
        if (!client_.process_import(state.import_id, &operation)) {
            set_error(error, error_size, client_.last_error());
            return false;
        }
        if (!wait_import(client_, state.import_id, operation, info, state.processed_at,
                         progress, callback, context, error, error_size)) return false;
    }
    if (!verify_source(source, operation, error, error_size)) return false;
    for (uint32_t i = 0; i < snapshot.count; ++i) {
        if (!write_marker(storage, legacy_paths, snapshot.entries[i])) {
            set_error(error, error_size, "done_marker_write");
            return false;
        }
    }
    state.phase = 0;
    state.fingerprint = fingerprint;
    copy_cstr(state.processed_at, sizeof(state.processed_at), info.processed_at);
    if (!save_day(storage, paths, day, state)) {
        set_error(error, error_size, "inflight_write");
        return false;
    }
    Log::logf(CAT_EXPORT, LOG_INFO,
              "[SLEEPHQ] Day %s synchronized sessions=%lu import=%lu uploaded=%u attached=%u present=%u\n",
              day, (unsigned long)snapshot.count, (unsigned long)state.import_id,
              progress.files_uploaded - uploaded_before, progress.files_attached - attached_before,
              progress.files_skipped - present_before);
    progress.file[0] = 0;
    progress.current_day[0] = 0;
    publish(progress, callback, context);
    return true;
}

}  // namespace SleepHqSync
