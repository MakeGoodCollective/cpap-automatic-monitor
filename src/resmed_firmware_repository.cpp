#include "resmed_firmware_repository.h"
#include "board.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#if AB_STORAGE_HAS_SDCARD
#include <Arduino.h>
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <new>
#include "memory_manager.h"
#include "sd_storage.h"
#endif

namespace ResmedFirmwareRepository {

bool valid_filename(const char *name) {
    if (!name || !*name || *name == '.' || strlen(name) > 192) return false;
    const size_t length = strlen(name);
    if (name[length - 1] == '.' || name[length - 1] == ' ') return false;
    for (const unsigned char *p = reinterpret_cast<const unsigned char *>(name); *p; ++p)
        if (*p < 32 || *p >= 127 || strchr("/\\:*?\"<>|", *p)) return false;
    return length < 5 || strcasecmp(name + length - 5, ".part");
}

bool bootloader_path(const char *bid, char *out, size_t capacity) {
    if (!bid || !*bid || strlen(bid) > 31 || !out || !capacity) return false;
    for (const char *p = bid; *p; ++p)
        if (!((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') ||
              (*p >= '0' && *p <= '9') || *p == '-' || *p == '_')) return false;
    const int written = snprintf(out, capacity, "%s/bootloaders/%s/patched.bin", ROOT, bid);
    return written > 0 && size_t(written) < capacity;
}

bool valid_path(const char *path) {
    if (!path || strlen(path) >= MAX_PATH) return false;
    const size_t root_length = strlen(ROOT);
    if (strncmp(path, ROOT, root_length) || path[root_length] != '/') return false;
    const char *name = path + root_length + 1;
    if (!strchr(name, '/')) return valid_filename(name);
    constexpr const char *prefix = "bootloaders/";
    if (strncmp(name, prefix, strlen(prefix))) {
        for (;;) {
            const char *slash = strchr(name, '/');
            const size_t length = slash ? size_t(slash - name) : strlen(name);
            char component[193];
            if (!length || length >= sizeof(component)) return false;
            memcpy(component, name, length);
            component[length] = 0;
            if (!valid_filename(component)) return false;
            if (!slash) return true;
            name = slash + 1;
        }
    }
    name += strlen(prefix);
    const char *slash = strchr(name, '/');
    if (!slash || size_t(slash - name) > 31) return false;
    char bid[32] = {}, expected[MAX_PATH];
    memcpy(bid, name, slash - name);
    return bootloader_path(bid, expected, sizeof(expected)) && !strcmp(path, expected);
}

#if AB_STORAGE_HAS_SDCARD
namespace {
SemaphoreHandle_t mutex = nullptr;
TaskHandle_t task = nullptr;
Status status;
std::shared_ptr<const Snapshot> published;
std::weak_ptr<const Snapshot> retired;
uint32_t requested = 1, completed = 0;
bool last_available = false;
constexpr size_t INTERNAL_BYTES = 4096;
constexpr size_t MAX_PACKED_BYTES = MAX_ENTRIES * (MAX_PATH + 40);
constexpr const char *BOOTLOADERS = "/airbridge/resmed-firmware/bootloaders";
}

struct Builder {
    std::shared_ptr<Snapshot> result{new (std::nothrow) Snapshot};
    size_t used = 0, capacity = 0;

    void truncate() { if (result) result->truncated_ = true; }

    bool append(const char *path, uint64_t size, int64_t modified, bool bootloader) {
        if (!result) return false;
        if (result->count_ == MAX_ENTRIES) { result->truncated_ = true; return true; }
        const size_t bytes = (sizeof(Snapshot::Entry) + strlen(path) + 1 + 7) & ~size_t(7);
        if (used + bytes > capacity) {
            size_t wanted = capacity ? capacity * 2 : 512;
            if (wanted < used + bytes) wanted = used + bytes;
            if (wanted > MAX_PACKED_BYTES) wanted = MAX_PACKED_BYTES;
            void *grown = aircannect::Memory::realloc_large(result->packed_, wanted,
                                                           wanted <= INTERNAL_BYTES);
            if (!grown) return false;
            result->packed_ = grown;
            capacity = wanted;
        }
        auto *data = static_cast<uint8_t *>(result->packed_);
        auto *entry = new (data + used) Snapshot::Entry{};
        entry->size = size;
        entry->modified = modified;
        entry->path = used + sizeof(Snapshot::Entry);
        entry->next = used + bytes;
        entry->bootloader = bootloader;
        strcpy(reinterpret_cast<char *>(data + entry->path), path);
        used += bytes;
        ++result->count_;
        return true;
    }

    void finish(uint32_t revision, bool truncated) {
        if (!result) return;
        result->revision_ = revision;
        result->truncated_ |= truncated;
        if (used && used < capacity) {
            void *shrunk = aircannect::Memory::realloc_large(result->packed_, used,
                                                           used <= INTERNAL_BYTES);
            if (shrunk) result->packed_ = shrunk;
        }
    }
};

Snapshot::~Snapshot() { aircannect::Memory::free(packed_); }

bool Snapshot::entry(size_t index, EntryView &out) const {
    out = {};
    if (index >= count_) return false;
    const auto *data = static_cast<const uint8_t *>(packed_);
    const Entry *entry = reinterpret_cast<const Entry *>(data);
    while (index--) entry = reinterpret_cast<const Entry *>(data + entry->next);
    out.path = reinterpret_cast<const char *>(data + entry->path);
    out.filename = strrchr(out.path, '/') + 1;
    out.size = entry->size;
    out.modified = entry->modified;
    out.bootloader = entry->bootloader;
    return true;
}

bool Snapshot::contains(const char *path) const {
    EntryView view;
    for (size_t i = 0; path && entry(i, view); ++i)
        if (!strcmp(path, view.path)) return true;
    return false;
}

namespace {
// Only queued paths, not open directories or image bytes. Popped nodes release
// their allocation immediately; the scan's entry limit bounds the whole walk.
class Directories {
    struct Node { Node *next; size_t bytes; char path[1]; };
    Node *first_ = nullptr, *last_ = nullptr;
    size_t bytes_ = 0;
public:
    ~Directories() {
        while (first_) {
            Node *next = first_->next;
            aircannect::Memory::free(first_);
            first_ = next;
        }
    }
    bool add(const char *path) {
        const size_t bytes = sizeof(Node) + strlen(path);
        auto *node = static_cast<Node *>(aircannect::Memory::alloc_large(
            bytes, bytes_ + bytes <= INTERNAL_BYTES));
        if (!node) return false;
        node->next = nullptr;
        node->bytes = bytes;
        strcpy(node->path, path);
        if (last_) last_->next = node;
        else first_ = node;
        last_ = node;
        bytes_ += bytes;
        return true;
    }
    bool take(char (&path)[MAX_PATH]) {
        if (!first_) return false;
        Node *node = first_;
        strcpy(path, node->path);
        first_ = node->next;
        if (!first_) last_ = nullptr;
        bytes_ -= node->bytes;
        aircannect::Memory::free(node);
        return true;
    }
};

bool ensure_directories(SdStorage::Session &session) {
    return session.mutate(ROOT, [](fs::FS &fs) {
        for (const char *path : {"/airbridge", ROOT, BOOTLOADERS}) {
            if (!fs.exists(path) && !fs.mkdir(path)) return false;
            fs::File dir = fs.open(path);
            if (!dir || !dir.isDirectory()) return false;
        }
        return true;
    });
}

const char *scan(Builder &builder, uint32_t files, SdStorage::Session &session) {
    // Keep the pending FIFO ticket across retries so background peers yield.
    if (!session.begin()) return "storage_busy";
    const char *error = nullptr;
    {
        SdStorage::Reader directory;
        Directories pending;
        char parent[MAX_PATH];
        unsigned visited = 0;
        if (!ensure_directories(session)) error = "directory_failed";
        if (!error && !pending.add(ROOT)) error = "catalog_no_memory";
        while (!error && pending.take(parent)) {
            directory.close();
            if (!directory.open(session, parent, true)) {
                error = "scan_open_failed";
                break;
            }
            for (;;) {
                SdStorage::Reader::Entry item{};
                bool end = false;
                if (session.yield_requested()) { error = "storage_busy"; break; }
                if (visited == MAX_SCAN_ENTRIES) { builder.truncate(); break; }
                if (!directory.next(item, end)) { error = "scan_failed"; break; }
                if (end) break;
                ++visited;
                const char *name = strrchr(item.name, '/');
                name = name ? name + 1 : item.name;
                if (!valid_filename(name)) continue;
                char path[MAX_PATH];
                const int length = snprintf(path, sizeof(path), "%s/%s", parent, name);
                if (length < 0 || size_t(length) >= sizeof(path)) {
                    builder.truncate();
                    continue;
                }
                if (item.directory) {
                    if (!pending.add(path)) { error = "catalog_no_memory"; break; }
                    continue;
                }
                if (!valid_path(path)) continue;
                const size_t bootloaders_length = strlen(BOOTLOADERS);
                const bool bootloader = !strncmp(path, BOOTLOADERS, bootloaders_length) &&
                                        path[bootloaders_length] == '/';
                if (!builder.append(path, item.size, item.modified, bootloader)) {
                    error = "catalog_no_memory";
                    break;
                }
                vTaskDelay(1);
            }
            if (visited == MAX_SCAN_ENTRIES) break;
        }
    }
    if (!session.valid() || files != SdStorage::files_revision()) error = "storage_changed";
    session.end();
    return error;
}

void worker(void *) {
    SdStorage::Session session;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xSemaphoreTake(mutex, portMAX_DELAY);
        const uint32_t target = requested, files = status.files_revision;
        if (!status.available || completed == target || !retired.expired()) {
            xSemaphoreGive(mutex);
            session.end();
            continue;
        }
        status.refreshing = true;
        xSemaphoreGive(mutex);

        Builder builder;
        const char *error = builder.result ? scan(builder, files, session) : "catalog_no_memory";
        if (!builder.result) session.end();
        xSemaphoreTake(mutex, portMAX_DELAY);
        status.refreshing = false;
        if (!error && target == requested && status.available) {
            ++status.revision;
            builder.finish(status.revision, false);
            retired = published;
            published = std::move(builder.result);
            status.ready = true;
            status.entries = published->size();
            status.truncated = published->truncated();
            status.error[0] = 0;
            completed = target;
        } else {
            const char *reason = error ? error : "storage_changed";
            // Retrying the same unavailable SD lease is not a new publication.
            if (status.ready || strcmp(status.error, reason)) ++status.revision;
            status.ready = false;
            snprintf(status.error, sizeof(status.error), "%s", reason);
            // Admission/preemption retries are driven by main poll, not HTTP.
            if (error && strcmp(error, "storage_busy") && strcmp(error, "storage_changed"))
                completed = target;
        }
        xSemaphoreGive(mutex);
    }
}
}

void init() {
    if (mutex) return;
    mutex = xSemaphoreCreateMutex();
    if (!mutex) return;
    status.supported = true;
    if (xTaskCreate(worker, "fw_catalog", 4096, nullptr, 1, &task) != pdPASS)
        snprintf(status.error, sizeof(status.error), "worker_no_memory");
}

void poll() {
    if (!mutex) return;
    const bool available = SdStorage::mounted() && SdStorage::local_access_allowed();
    const uint32_t files = SdStorage::files_revision();
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool changed = last_available != available || files != status.files_revision;
    if (changed) {
        last_available = available;
        status.available = available;
        status.files_revision = files;
        status.ready = false;
        ++requested;
        ++status.revision;
        if (!available) snprintf(status.error, sizeof(status.error), "storage_unavailable");
    }
    const bool wake = !status.refreshing && (changed || (available && completed != requested));
    xSemaphoreGive(mutex);
    if (wake && task) xTaskNotifyGive(task);
}

void request_refresh() {
    if (!mutex) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    ++requested;
    status.ready = false;
    ++status.revision;
    xSemaphoreGive(mutex);
    if (task) xTaskNotifyGive(task);
}

void get_status(Status &out) {
    out = {};
    if (!mutex) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    out = status;
    xSemaphoreGive(mutex);
}

std::shared_ptr<const Snapshot> snapshot() {
    Status ignored;
    return snapshot(ignored);
}

std::shared_ptr<const Snapshot> snapshot(Status &out) {
    out = {};
    if (!mutex) return {};
    xSemaphoreTake(mutex, portMAX_DELAY);
    out = status;
    auto result = status.ready ? published : std::shared_ptr<const Snapshot>{};
    xSemaphoreGive(mutex);
    return result;
}

bool notify_published(const char *path) {
    if (!valid_path(path)) return false;
    SdStorage::notify_files_changed();
    request_refresh();
    return true;
}

#else
Snapshot::~Snapshot() = default;
bool Snapshot::entry(size_t, EntryView &out) const { out = {}; return false; }
bool Snapshot::contains(const char *) const { return false; }
void init() {}
void poll() {}
void request_refresh() {}
void get_status(Status &out) { out = {}; }
std::shared_ptr<const Snapshot> snapshot() { return {}; }
std::shared_ptr<const Snapshot> snapshot(Status &out) { out = {}; return {}; }
bool notify_published(const char *) { return false; }
#endif

}  // namespace ResmedFirmwareRepository
