#include "storage_browser.h"
#include "board.h"

#if AB_STORAGE_HAS_SDCARD
#include <Arduino.h>
#include <FS.h>
#include <algorithm>
#include <atomic>
#include <new>
#include <string.h>
#include <time.h>
#include "crc.h"
#include "json_util.h"
#include "large_text_buffer.h"
#include "memory_manager.h"
#include "sd_storage.h"
#include "debug_log.h"
#include "export_sync.h"
#include "airsense_state.h"

namespace StorageBrowser {
namespace {
using aircannect::Memory::free;
constexpr size_t RING_BYTES = 16 * 1024;
constexpr size_t INTERNAL_RING_BYTES = 4096;
constexpr size_t INTERNAL_COPY_BYTES = 1024;
static_assert(SdStorage::READ_CHUNK_BYTES <= UINT16_MAX, "ZIP stored block length");
constexpr uint32_t CONSUMER_TIMEOUT_MS = 30000;
std::atomic<unsigned> active_operations{0};
MutationStatus mutation = {};
portMUX_TYPE mutation_mux = portMUX_INITIALIZER_UNLOCKED;

bool mutating(Kind kind) { return kind == Kind::Rename || kind == Kind::Delete; }

const char *kind_name(Kind kind) {
    switch (kind) {
    case Kind::List: return "list";
    case Kind::File: return "file";
    case Kind::Archive: return "zip";
    case Kind::Rename: return "rename";
    case Kind::Delete: return "delete";
    case Kind::Immutable: return "artifact";
    }
    return "unknown";
}

bool valid_path(const char *path) {
    if (!path || path[0] != '/') return false;
    if (!path[1]) return true;
    const char *segment = path + 1;
    for (const char *p = segment;; p++) {
        if (*p && (uint8_t(*p) < 32 || *p == '\\' || *p == 127)) return false;
        if (!*p || *p == '/') {
            const size_t size = p - segment;
            if (!size || (size == 1 && segment[0] == '.') ||
                (size == 2 && segment[0] == '.' && segment[1] == '.')) return false;
            if (!*p) return true;
            segment = p + 1;
        }
    }
}

bool child_path(const char *parent, const char *name, size_t length, char (&out)[256]) {
    if (!length || memchr(name, '/', length) || memchr(name, '\n', length)) return false;
    const int written = snprintf(out, sizeof(out), "%s%s%.*s", parent,
        strcmp(parent, "/") ? "/" : "", int(length), name);
    return written > 0 && size_t(written) < sizeof(out) && valid_path(out);
}

bool rename_target(const Request &request, char (&out)[256]) {
    char parent[256];
    strcpy(parent, request.path);
    char *slash = strrchr(parent, '/');
    if (!slash || !slash[1]) return false;
    if (slash == parent) slash[1] = 0;
    else *slash = 0;
    return child_path(parent, request.selection, strlen(request.selection), out);
}

bool valid_request(const Request &request) {
    if (!memchr(request.path, 0, sizeof(request.path)) || !valid_path(request.path) ||
        !memchr(request.selection, 0, sizeof(request.selection))) return false;
    if (request.range_count > 8 || (request.kind != Kind::Immutable && request.range_count)) return false;
    uint64_t bytes = 0;
    for (unsigned i = 0; i < request.range_count; ++i) {
        const auto &range = request.ranges[i];
        if (range.last < range.first || (i && range.first <= request.ranges[i - 1].last)) return false;
        bytes += uint64_t(range.last) - range.first + 1;
    }
    if (bytes > 256 * 1024) return false;
    char path[256];
    if (request.kind == Kind::Rename) return rename_target(request, path);
    if (request.kind == Kind::Delete && !strcmp(request.path, "/") && !*request.selection)
        return false;
    const char *name = request.selection;
    while (*name) {
        const char *end = strchr(name, '\n');
        const size_t length = end ? size_t(end - name) : strlen(name);
        if (!child_path(request.path, name, length, path) || (end && !end[1])) return false;
        name = end ? end + 1 : name + length;
    }
    return true;
}

struct ZipEntry {
    char path[256];
    uint32_t size;
    uint32_t crc;
    uint32_t offset;
    uint32_t modified;
    bool directory;
};

class Job final : public Transfer {
public:
    Request request = {};
    Ready ready;
    Released released = nullptr;
    SdStorage::Session session;
    SdStorage::Reader input;
    uint8_t *ring = nullptr;
    size_t capacity = 0;
    uint8_t *copy_buffer = nullptr;
    size_t copy_capacity = 0;
    std::atomic<uint32_t> produced{0}, consumed{0};
    std::atomic<bool> cancelled{false}, done{false}, error{false};
    bool registered = false;
    uint32_t started_ms = millis();
    const char *failure = nullptr;
    log_level_t failure_level = LOG_WARN;
    char failure_path[256] = {};
    ZipEntry *entries = nullptr;
    size_t entry_count = 0, entry_capacity = 0;
    uint64_t representation = 0;
    uint64_t representation_size() const override { return representation; }

    ~Job() override {
        // Releasing an HTTP response also happens on success. Only unread
        // producer output proves that its consumer stopped early.
        if (done.load() && !error.load() && produced.load() != consumed.load())
            report(LOG_DEBUG, "response_released_early");
        free(entries);
        free(ring);
        free(copy_buffer);
        if (registered) active_operations.fetch_sub(1);
    }

    bool failed() const override { return error.load(); }
    bool finished() const override { return done.load() && produced.load() == consumed.load(); }
    void cancel() override { cancelled.store(true); }

    void report(log_level_t level, const char *result) const {
        Log::logf(CAT_STORAGE, level, "%s %s made=%u out=%u ms=%u path=%s\n",
            kind_name(request.kind), result, unsigned(produced.load()),
            unsigned(consumed.load()), unsigned(uint32_t(millis() - started_ms)),
            *failure_path ? failure_path : request.path);
    }

    bool fail(const char *reason, const char *path = nullptr) {
        if (!failure) {
            failure = reason;
            if (cancelled.load()) {
                failure = "cancelled";
                failure_level = LOG_DEBUG;
            } else if (!session.valid()) {
                failure = "storage_revoked";
                failure_level = LOG_DEBUG;
            }
            if (path) snprintf(failure_path, sizeof(failure_path), "%s", path);
        }
        return false;
    }

    bool admit_reader(const char *path) {
        while (!cancelled.load() && SdStorage::mounted() && SdStorage::local_access_allowed() &&
               AirSenseState::local_background_allowed()) {
            if (session.begin(SdStorage::Access::ReadOnly, path)) return true;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        return fail("storage_wait_interrupted", path);
    }

    void close_input() {
        input.close();
        // Only the current file is protected. Keep the admission, not its old
        // scope, while draining output or waiting for the next file's Writer.
        if (!mutating(request.kind))
            while (!cancelled.load() && session.valid() && !session.protect_read(nullptr))
                vTaskDelay(pdMS_TO_TICKS(10));
    }

    bool open_input(const char *path, bool directory = false) {
        close_input();
        if (!mutating(request.kind)) {
            for (;;) {
                if (!checkpoint()) return false;
                if (session.protect_read(directory ? nullptr : path)) break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        return input.open(session, path, directory);
    }

    bool checkpoint() {
        return (!cancelled.load() && session.valid()) || fail("interrupted");
    }

    bool next_input(SdStorage::Reader::Entry &entry, bool &end) {
        return checkpoint() && input.next(entry, end);
    }

    size_t read(uint8_t *out, size_t length) override {
        const uint32_t tail = consumed.load();
        length = std::min<size_t>(length, produced.load() - tail);
        const size_t first = std::min<size_t>(length, capacity - tail % capacity);
        memcpy(out, ring + tail % capacity, first);
        memcpy(out + first, ring, length - first);
        consumed.store(tail + length);
        return length;
    }

    bool emit(const uint8_t *data, size_t length) {
        uint32_t progress = millis();
        while (length) {
            if (!checkpoint()) return false;
            const uint32_t head = produced.load();
            const size_t room = capacity - (head - consumed.load());
            if (!room) {
                if (uint32_t(millis() - progress) >= CONSUMER_TIMEOUT_MS)
                    return fail("consumer_timeout");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            const size_t count = std::min(length, std::min<size_t>(room, capacity - head % capacity));
            memcpy(ring + head % capacity, data, count);
            produced.store(head + count);
            data += count;
            length -= count;
            progress = millis();
        }
        return true;
    }

    bool add(const char *path, uint64_t size, bool directory, int64_t modified = 0) {
        if ((request.kind != Kind::Delete && size > UINT32_MAX) ||
            entry_count == UINT16_MAX || strlen(path) >= 256)
            return fail("entry_limit", path);
        if (entry_count == entry_capacity) {
            const size_t next = std::min<size_t>(UINT16_MAX, entry_capacity ? entry_capacity * 2 : 16);
            void *grown = aircannect::Memory::realloc_large(entries,
                next * sizeof(ZipEntry), next * sizeof(ZipEntry) <= 16384);
            if (!grown) return fail("entries_no_memory", path);
            entries = static_cast<ZipEntry *>(grown);
            entry_capacity = next;
        }
        ZipEntry &entry = entries[entry_count++];
        entry = {};
        strcpy(entry.path, path);
        entry.size = size;
        entry.directory = directory;
        time_t stamp = modified;
        struct tm date = {};
        entry.modified = 33u << 16;
        if (modified > 0 && localtime_r(&stamp, &date) && date.tm_year >= 80 && date.tm_year <= 207)
            entry.modified = uint32_t(date.tm_year - 80) << 25 | uint32_t(date.tm_mon + 1) << 21 |
                uint32_t(date.tm_mday) << 16 | uint32_t(date.tm_hour) << 11 |
                uint32_t(date.tm_min) << 5 | uint32_t(date.tm_sec / 2);
        return true;
    }
};

bool list(Job &job, aircannect::LargeTextBuffer &json) {
    if (!job.open_input(job.request.path, true)) return job.fail("directory_open_failed");
    json = "{\"ok\":true";
    aircannect::json_add_string(json, "path", job.request.path);
    json += ",\"entries\":[";
    uint32_t seen = 0, count = 0;
    bool end = false;
    SdStorage::Reader::Entry entry;
    while (!end) {
        if (!job.next_input(entry, end)) return job.fail("directory_read_failed");
        if (end) break;
        if (seen++ < job.request.offset) continue;
        if (count == 32) break;
        if (count++) json += ',';
        json += '{';
        aircannect::json_add_string(json, "name", entry.name, false);
        aircannect::json_add_uint64(json, "size", entry.size);
        aircannect::json_add_uint64(json, "modified", entry.modified > 0 ? entry.modified : 0);
        aircannect::json_add_bool(json, "directory", entry.directory);
        json += '}';
    }
    json += ']';
    aircannect::json_add_bool(json, "more", !end);
    json += '}';
    job.close_input();
    return !json.overflowed() || job.fail("listing_no_memory");
}

bool collect(Job &job) {
    const char *selection = job.request.selection;
    if (!*selection) {
        if (job.open_input(job.request.path)) {
            const bool added = job.add(job.request.path, job.input.size(), false, job.input.modified());
            job.close_input();
            return added;
        }
        if (!job.add(job.request.path, 0, true)) return false;
    } else {
        while (*selection) {
            const char *end = strchr(selection, '\n');
            const size_t length = end ? size_t(end - selection) : strlen(selection);
            char path[256];
            if (!child_path(job.request.path, selection, length, path)) return job.fail("invalid_child_path");
            const bool regular = job.open_input(path);
            if (!job.add(path, regular ? job.input.size() : 0, !regular,
                         regular ? job.input.modified() : 0)) return false;
            job.close_input();
            selection = end ? end + 1 : selection + length;
        }
    }
    // Breadth-first traversal holds one directory handle, not one per level.
    for (size_t i = 0; i < job.entry_count; i++) {
        if (!job.entries[i].directory) continue;
        char parent[256];
        strcpy(parent, job.entries[i].path);
        if (!job.open_input(parent, true)) return job.fail("directory_open_failed", parent);
        bool end = false;
        SdStorage::Reader::Entry entry;
        while (!end) {
            if (!job.next_input(entry, end)) return job.fail("directory_read_failed", parent);
            if (end) break;
            char path[256];
            if (!child_path(parent, entry.name, strlen(entry.name), path))
                return job.fail("invalid_child_path", parent);
            if (!job.add(path, entry.size, entry.directory, entry.modified)) return false;
        }
        job.close_input();
    }
    return true;
}

const char *mutate(Job &job, uint32_t &changed) {
    if (job.cancelled.load() || !job.session.valid()) return "cancelled";
    if (job.request.kind == Kind::Rename) {
        char destination[256];
        if (!rename_target(job.request, destination)) return "invalid_name";
        const char *error = "storage_busy";
        const bool renamed = job.session.mutate(job.request.path, destination, [&](fs::FS &fs) {
            if (job.cancelled.load()) { error = "cancelled"; return false; }
            if (!fs.exists(job.request.path)) { error = "not_found"; return false; }
            if (fs.exists(destination)) { error = "destination_exists"; return false; }
            error = "rename_failed";
            if (!fs.rename(job.request.path, destination)) return false;
            SdStorage::notify_files_changed();
            return true;
        });
        if (!renamed) return error;
        changed = 1;
        return nullptr;
    }
    // Collect before deleting: malformed selections and allocation failures
    // must not leave a half-deleted tree. Children follow parents in this list.
    if (!collect(job)) return "selection_unavailable";
    const char *error = nullptr;
    for (size_t i = job.entry_count; i > 0; i--) {
        if (job.cancelled.load() || !job.session.valid()) { error = "cancelled"; break; }
        const ZipEntry &entry = job.entries[i - 1];
        const bool removed = job.session.mutate(entry.path, [&](fs::FS &fs) {
            if (job.cancelled.load()) return false;
            if (!(entry.directory ? fs.rmdir(entry.path) : fs.remove(entry.path))) return false;
            SdStorage::notify_files_changed();
            return true;
        });
        if (!removed) { error = job.session.valid() ? "delete_failed" : "cancelled"; break; }
        changed++;
    }
    if (changed) job.session.run([](fs::FS &) { SdStorage::refresh_usage(); return true; });
    return error;
}

uint64_t zip_data_size(uint32_t size, size_t block_bytes) {
    return uint64_t(size) + 5 * (size ? (uint64_t(size) + block_bytes - 1) / block_bytes : 1);
}

bool copy_file(Job &job, uint32_t *checksum = nullptr,
               const char *path = nullptr, uint64_t limit = UINT64_MAX) {
    auto &file = job.input;
    uint8_t *buffer = job.copy_buffer;
    uint64_t remaining = std::min(file.size(), limit);
    uint32_t crc = crc32_ieee_initial();
    do {
        const size_t count = std::min<uint64_t>(job.copy_capacity, remaining);
        if (checksum) {
            // Raw DEFLATE stored block: one pass, no compression workspace.
            uint8_t block[5];
            block[0] = remaining <= count ? 1 : 0;
            SdStorage::put_le16(block + 1, count);
            SdStorage::put_le16(block + 3, uint16_t(~count));
            if (!job.emit(block, sizeof(block))) return false;
        }
        if (!job.checkpoint() || file.read(buffer, count) != count) return job.fail("file_read_failed", path);
        if (!job.emit(buffer, count)) return false;
        if (checksum) crc = crc32_ieee_update(crc, buffer, count);
        remaining -= count;
    } while (remaining);
    if (checksum) *checksum = crc32_ieee_finish(crc);
    return true;
}

size_t range_header(const Job &job, unsigned index, char (&out)[160]) {
    const auto &range = job.request.ranges[index];
    return snprintf(out, sizeof(out), "--ab-range\r\nContent-Type: application/octet-stream\r\n"
        "Content-Range: bytes %lu-%lu/%llu\r\n\r\n", static_cast<unsigned long>(range.first),
        static_cast<unsigned long>(range.last), static_cast<unsigned long long>(job.representation));
}

bool immutable_file(const std::shared_ptr<Job> &transfer) {
    auto &job = *transfer;
    if (!job.open_input(job.request.path)) {
        job.ready(404, "artifact_unavailable", nullptr, 0);
        return job.fail("file_open_failed");
    }
    auto &file = job.input;
    job.representation = file.size();
    uint64_t length = 0;
    char header[160];
    for (unsigned i = 0; i < job.request.range_count; ++i) {
        const auto &range = job.request.ranges[i];
        if (range.last >= file.size()) {
            job.ready(416, "range_unsatisfiable", nullptr, 0);
            return job.fail("range_unsatisfiable");
        }
        length += uint64_t(range.last) - range.first + 1;
        if (job.request.range_count > 1) length += range_header(job, i, header) + 2;
    }
    if (!job.request.range_count) length = file.size();
    else if (job.request.range_count > 1) length += sizeof("--ab-range--\r\n") - 1;
    if (job.request.unchanged || job.request.head) {
        job.ready(job.request.unchanged ? 304 : 200, nullptr, nullptr, file.size());
        return true;
    }
    job.ready(job.request.range_count ? 206 : 200, nullptr, transfer, length);
    if (!job.request.range_count) return copy_file(job);
    for (unsigned i = 0; i < job.request.range_count; ++i) {
        const auto &range = job.request.ranges[i];
        if (job.request.range_count > 1 &&
            !job.emit(reinterpret_cast<const uint8_t *>(header), range_header(job, i, header))) return false;
        if (!file.seek(range.first) || !copy_file(job, nullptr, nullptr,
            uint64_t(range.last) - range.first + 1)) return job.fail("range_read_failed");
        if (job.request.range_count > 1 && !job.emit(reinterpret_cast<const uint8_t *>("\r\n"), 2)) return false;
    }
    return job.request.range_count == 1 || job.emit(reinterpret_cast<const uint8_t *>("--ab-range--\r\n"), 14);
}

bool archive(Job &job) {
    using SdStorage::put_le16;
    using SdStorage::put_le32;
    uint64_t position = 0;
    uint16_t files = 0;
    for (size_t i = 0; i < job.entry_count; i++) {
        ZipEntry &entry = job.entries[i];
        if (entry.directory) continue;
        const char *name = entry.path + 1;
        const size_t length = strlen(name);
        const uint64_t compressed_size = zip_data_size(entry.size, job.copy_capacity);
        if (position + 30 + length + compressed_size + 16 > UINT32_MAX)
            return job.fail("zip_size_limit", entry.path);
        entry.offset = position;
        uint8_t header[30] = {};
        put_le32(header, 0x04034b50);
        put_le16(header + 4, 20);
        put_le16(header + 6, 0x0808); // UTF-8 names, trailing data descriptor.
        put_le16(header + 8, 8);
        put_le32(header + 10, entry.modified);
        put_le16(header + 26, length);
        if (!job.emit(header, sizeof(header)) ||
            !job.emit(reinterpret_cast<const uint8_t *>(name), length)) return false;
        if (!job.open_input(entry.path)) return job.fail("file_open_failed", entry.path);
        auto &file = job.input;
        if (file.size() != entry.size) return job.fail("file_size_changed", entry.path);
        if (!copy_file(job, &entry.crc, entry.path)) return false;
        job.close_input();
        uint8_t descriptor[16];
        put_le32(descriptor, 0x08074b50);
        put_le32(descriptor + 4, entry.crc);
        put_le32(descriptor + 8, compressed_size);
        put_le32(descriptor + 12, entry.size);
        if (!job.emit(descriptor, sizeof(descriptor))) return false;
        position += sizeof(header) + length + compressed_size + sizeof(descriptor);
        files++;
    }
    const uint32_t central_offset = position;
    for (size_t i = 0; i < job.entry_count; i++) {
        const ZipEntry &entry = job.entries[i];
        if (entry.directory) continue;
        const char *name = entry.path + 1;
        const size_t length = strlen(name);
        if (position + 46 + length + 22 > UINT32_MAX) return job.fail("zip_size_limit", entry.path);
        uint8_t header[46] = {};
        put_le32(header, 0x02014b50);
        put_le16(header + 4, 20);
        put_le16(header + 6, 20);
        put_le16(header + 8, 0x0808);
        put_le16(header + 10, 8);
        put_le32(header + 12, entry.modified);
        put_le32(header + 16, entry.crc);
        put_le32(header + 20, zip_data_size(entry.size, job.copy_capacity));
        put_le32(header + 24, entry.size);
        put_le16(header + 28, length);
        put_le32(header + 42, entry.offset);
        if (!job.emit(header, sizeof(header)) ||
            !job.emit(reinterpret_cast<const uint8_t *>(name), length)) return false;
        position += sizeof(header) + length;
    }
    uint8_t end[22] = {};
    put_le32(end, 0x06054b50);
    put_le16(end + 8, files);
    put_le16(end + 10, files);
    put_le32(end + 12, position - central_offset);
    put_le32(end + 16, central_offset);
    return job.emit(end, sizeof(end));
}

void produce(void *context) {
    auto job = std::move(*static_cast<std::shared_ptr<Job> *>(context));
    delete static_cast<std::shared_ptr<Job> *>(context);
    bool success = false;
    job->report(LOG_DEBUG, "started");
    const SdStorage::Access access = mutating(job->request.kind)
        ? SdStorage::Access::Exclusive : SdStorage::Access::ReadOnly;
    const char *read_root = access == SdStorage::Access::ReadOnly &&
        (job->request.kind == Kind::File || job->request.kind == Kind::Immutable)
        ? job->request.path : nullptr;
    bool admitted = access == SdStorage::Access::ReadOnly ? job->admit_reader(read_root)
        : job->session.begin(access);
    // Mutation admission is bounded. Downloads wait behind existing background
    // work until admitted or disconnected, never on the AsyncTCP callback.
    while (access == SdStorage::Access::Exclusive && !admitted && !job->cancelled.load() &&
           SdStorage::local_access_allowed() && uint32_t(millis() - job->started_ms) < 2000) {
        vTaskDelay(pdMS_TO_TICKS(10));
        admitted = job->session.begin(access);
    }
    if (admitted && job->request.kind == Kind::Immutable)
        Log::logf(CAT_STORAGE, LOG_DEBUG, "artifact admitted wait=%ums path=%s\n",
            unsigned(uint32_t(millis() - job->started_ms)), job->request.path);
    if (mutating(job->request.kind)) {
        uint32_t changed = 0;
        const char *error = admitted ? mutate(*job, changed) : "storage_busy";
        if (changed) ExportSync::request_backlog_refresh(true);
        success = !error;
        portENTER_CRITICAL(&mutation_mux);
        mutation.active = false;
        mutation.succeeded = success;
        mutation.changed = changed;
        snprintf(mutation.error, sizeof(mutation.error), "%s", error ? error : "");
        portEXIT_CRITICAL(&mutation_mux);
        log_level_t level = success ? LOG_INFO : LOG_WARN;
        if (!success) {
            if (job->failure) level = job->failure_level;
            else if (!admitted || job->cancelled.load() || !job->session.valid()) level = LOG_DEBUG;
        }
        Log::logf(CAT_STORAGE, level,
            "%s %s: %s (%u changed)\n",
            job->request.kind == Kind::Rename ? "rename" : "delete",
            job->request.path, job->failure ? job->failure : error ? error : "done", changed);
        job->ready(success ? 200 : 409, error, nullptr, changed);
    } else if (!admitted) {
        job->report(LOG_DEBUG, "storage_busy");
        job->ready(409, "storage_busy", nullptr, 0);
    } else if (job->request.kind == Kind::List) {
        aircannect::LargeTextBuffer json;
        if (list(*job, json)) {
            job->ready(200, nullptr, job, json.length());
            success = job->emit(reinterpret_cast<const uint8_t *>(json.c_str()), json.length());
        } else job->ready(409, "listing_failed", nullptr, 0);
    } else if (job->request.kind == Kind::Immutable) {
        success = immutable_file(job);
    } else if (job->request.kind == Kind::File) {
        if (job->open_input(job->request.path)) {
            auto &file = job->input;
            job->ready(200, nullptr, job, file.size());
            success = copy_file(*job);
        } else {
            job->fail("file_open_failed");
            job->ready(409, "file_unavailable", nullptr, 0);
        }
    } else if (collect(*job)) {
        job->ready(200, nullptr, job, 0);
        success = archive(*job);
    } else job->ready(409, "archive_unavailable", nullptr, 0);
    job->ready = nullptr;
    if (!mutating(job->request.kind) && admitted)
        job->report(success ? LOG_DEBUG : job->failure_level,
            success ? "producer_done" : job->failure);
    job->close_input();
    job->session.end();
    if (job->released) job->released();
    free(job->copy_buffer);
    job->copy_buffer = nullptr;
    job->error.store(!success);
    job->done.store(true);
    job.reset();
    vTaskDeleteWithCaps(nullptr);
}
}  // namespace

StartResult start(const Request &request, Ready ready, std::weak_ptr<Transfer> &active,
                  Released released) {
    if (!valid_request(request)) return StartResult::BadRequest;
    if (!SdStorage::local_access_allowed() || !SdStorage::mounted()) return StartResult::Unavailable;
    // Two readers may coexist, including a response outliving its producer.
    // Mutations keep a single browser job until their exclusive lease finishes.
    const unsigned limit = mutating(request.kind) ? 1 : 2;
    unsigned active_count = active_operations.load();
    for (;;) {
        if (active_count >= limit) {
            Log::logf(CAT_STORAGE, LOG_DEBUG, "%s storage_busy path=%s\n", kind_name(request.kind), request.path);
            return StartResult::Busy;
        }
        if (active_operations.compare_exchange_weak(active_count, active_count + 1)) break;
    }
    void *memory = aircannect::Memory::alloc_large(sizeof(Job));
    if (!memory) {
        Log::logf(CAT_STORAGE, LOG_WARN, "%s job_no_memory path=%s\n", kind_name(request.kind), request.path);
        active_operations.fetch_sub(1);
        return StartResult::Unavailable;
    }
    auto job = std::shared_ptr<Job>(new(memory) Job, [](Job *value) {
        value->~Job();
        free(value);
    });
    job->registered = true;
    job->request = request;
    job->released = released;
    active = job;
    job->ready = std::move(ready);
    if (!mutating(request.kind)) {
        job->capacity = RING_BYTES;
        job->ring = static_cast<uint8_t *>(aircannect::Memory::alloc_large(RING_BYTES, false));
        if (!job->ring) {
            job->capacity = INTERNAL_RING_BYTES;
            job->ring = static_cast<uint8_t *>(aircannect::Memory::alloc_large(INTERNAL_RING_BYTES));
        }
        if (!job->ring) { job->report(LOG_WARN, "ring_no_memory"); return StartResult::Unavailable; }
    }
    if (request.kind == Kind::File || request.kind == Kind::Archive || request.kind == Kind::Immutable) {
        job->copy_capacity = SdStorage::READ_CHUNK_BYTES;
        job->copy_buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(job->copy_capacity, false));
        if (!job->copy_buffer) {
            job->copy_capacity = INTERNAL_COPY_BYTES;
            job->copy_buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(job->copy_capacity));
        }
        if (!job->copy_buffer) { job->report(LOG_WARN, "copy_no_memory"); return StartResult::Unavailable; }
    }
    auto *context = new(std::nothrow) std::shared_ptr<Job>(job);
    if (!context) { job->report(LOG_WARN, "context_no_memory"); return StartResult::Unavailable; }
    if (mutating(request.kind)) {
        portENTER_CRITICAL(&mutation_mux);
        mutation = {};
        mutation.active = true;
        portEXIT_CRITICAL(&mutation_mux);
    }
    BaseType_t created = pdFAIL;
    if (aircannect::Memory::psram_available())
        created = xTaskCreatePinnedToCoreWithCaps(produce, "sd_browser", 6144,
            context, 1, nullptr, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (created != pdPASS)
        created = xTaskCreatePinnedToCoreWithCaps(produce, "sd_browser", 6144,
            context, 1, nullptr, 0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (created != pdPASS) {
        job->report(LOG_WARN, "worker_unavailable");
        delete context;
        if (mutating(request.kind)) {
            portENTER_CRITICAL(&mutation_mux);
            mutation.active = false;
            strcpy(mutation.error, "worker_unavailable");
            portEXIT_CRITICAL(&mutation_mux);
        }
        return StartResult::Unavailable;
    }
    return StartResult::Started;
}

void mutation_status(MutationStatus &out) {
    portENTER_CRITICAL(&mutation_mux);
    out = mutation;
    portEXIT_CRITICAL(&mutation_mux);
}
}  // namespace StorageBrowser
#endif
