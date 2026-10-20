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
#include <new>
#include "memory_manager.h"
#include "sd_storage.h"
#include "storage_browser.h"
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


bool valid_directory(const char *path) {
    if (!path || strlen(path) >= MAX_PATH) return false;
    if (!strcmp(path, ROOT)) return true;
    const size_t root_length = strlen(ROOT);
    if (strncmp(path, ROOT, root_length) || path[root_length] != '/') return false;
    const char *name = path + root_length + 1;
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

bool is_bootloader(const char *path) {
    constexpr const char *prefix = "/airbridge/resmed-firmware/bootloaders/";
    return path && !strncmp(path, prefix, strlen(prefix));
}

bool valid_path(const char *path) {
    if (!path || strlen(path) >= MAX_PATH) return false;
    const size_t root_length = strlen(ROOT);
    if (strncmp(path, ROOT, root_length) || path[root_length] != '/') return false;
    const char *name = path + root_length + 1;
    if (!strchr(name, '/')) return valid_filename(name);
    constexpr const char *prefix = "bootloaders/";
    if (strncmp(name, prefix, strlen(prefix))) return valid_directory(path);
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
Status status;
std::shared_ptr<const Snapshot> published;
constexpr size_t INTERNAL_BYTES = 4096;
constexpr size_t MAX_PACKED_BYTES = PAGE_SIZE * (MAX_PATH + 40);
}

struct Builder {
    std::shared_ptr<Snapshot> result{new (std::nothrow) Snapshot};
    size_t used = 0, capacity = 0;

    void select(const char *path, uint32_t offset) {
        if (result) { strcpy(result->path_, path); result->offset_ = offset; }
    }
    void finish_page(bool more) { if (result) result->more_ = more; }

    bool append(const char *path, uint64_t size, int64_t modified, bool bootloader,
                bool directory = false) {
        if (!result) return false;
        if (result->count_ == PAGE_SIZE) return false;
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
        entry->directory = directory;
        strcpy(reinterpret_cast<char *>(data + entry->path), path);
        used += bytes;
        ++result->count_;
        return true;
    }

    void finish(uint32_t revision) {
        if (!result) return;
        result->revision_ = revision;
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
    out.directory = entry->directory;
    return true;
}

bool Snapshot::contains(const char *path) const {
    EntryView view;
    for (size_t i = 0; path && entry(i, view); ++i)
        if (!view.directory && !strcmp(path, view.path)) return true;
    return false;
}

void init() {
    if (!mutex) mutex = xSemaphoreCreateMutex();
    if (mutex) status.supported = true;
}

void poll() {
    if (!mutex) return;
    const bool available = SdStorage::mounted() && SdStorage::local_access_allowed();
    const uint32_t files = SdStorage::files_revision(SdStorage::FileScope::Firmware);
    xSemaphoreTake(mutex, portMAX_DELAY);
    status.supported = true;
    if (status.available != available || files != status.files_revision) {
        status.available = available;
        status.files_revision = files;
        status.ready = false;
        published.reset();
        ++status.revision;
        snprintf(status.error, sizeof(status.error), "%s", available ? "" : "storage_unavailable");
    }
    xSemaphoreGive(mutex);
}

void request_refresh() {
    if (!mutex) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    published.reset();
    status.ready = false;
    ++status.revision;
    xSemaphoreGive(mutex);
}

bool request_listing(const char *path, uint32_t offset) {
    if (!mutex || !valid_directory(path)) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (status.refreshing || (published && status.ready &&
        published->offset() == offset && !strcmp(published->path(), path))) {
        xSemaphoreGive(mutex);
        return true;
    }
    status.refreshing = true;
    status.ready = false;
    status.error[0] = 0;
    published.reset();
    const uint32_t revision = status.revision;
    xSemaphoreGive(mutex);

    auto builder = std::shared_ptr<Builder>(new(std::nothrow) Builder);
    if (!builder || !builder->result) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        status.refreshing = false;
        snprintf(status.error, sizeof(status.error), "listing_no_memory");
        xSemaphoreGive(mutex);
        return false;
    }
    builder->select(path, offset);
    StorageBrowser::Request request = {};
    request.kind = StorageBrowser::Kind::List;
    request.firmware_repository = true;
    request.offset = offset;
    strcpy(request.path, path);
    std::weak_ptr<StorageBrowser::Transfer> active;
    auto complete = [builder, revision](int, const char *error,
            std::shared_ptr<StorageBrowser::Transfer>, uint64_t) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        status.refreshing = false;
        if (!error && revision == status.revision && builder->result) {
            builder->finish(revision);
            published = builder->result;
            status.ready = true;
            status.entries = published->size();
        } else {
            snprintf(status.error, sizeof(status.error), "%s", error ? error : "storage_changed");
        }
        xSemaphoreGive(mutex);
    };
    const auto result = StorageBrowser::start(request, complete, active, nullptr,
        [builder](const char *entry, uint64_t size, int64_t modified, bool directory, bool more) {
            if (!entry) { builder->finish_page(more); return bool(builder->result); }
            return builder->append(entry, size, modified, !directory && is_bootloader(entry), directory);
        });
    if (result != StorageBrowser::StartResult::Started)
        complete(503, result == StorageBrowser::StartResult::Busy ? "storage_busy" : "storage_unavailable", {}, 0);
    return result == StorageBrowser::StartResult::Started;
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
    SdStorage::notify_files_changed(path);
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
bool request_listing(const char *, uint32_t) { return false; }
std::shared_ptr<const Snapshot> snapshot() { return {}; }
std::shared_ptr<const Snapshot> snapshot(Status &out) { out = {}; return {}; }
bool notify_published(const char *) { return false; }
#endif

}  // namespace ResmedFirmwareRepository
