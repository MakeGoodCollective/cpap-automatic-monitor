#pragma once

#include <memory>
#include <stddef.h>
#include <stdint.h>
#include "storage_browser.h"

namespace ResmedFirmwareRepository {

constexpr const char *ROOT = "/airbridge/resmed-firmware";
constexpr size_t MAX_PATH = 256;
constexpr size_t PAGE_SIZE = StorageBrowser::LIST_PAGE_SIZE;

struct EntryView {
    const char *path = nullptr;
    const char *filename = nullptr;
    uint64_t size = 0;
    int64_t modified = 0;
    bool bootloader = false;
    bool directory = false;
};

class Snapshot {
public:
    ~Snapshot();
    size_t size() const { return count_; }
    uint32_t revision() const { return revision_; }
    bool more() const { return more_; }
    const char *path() const { return path_; }
    uint32_t offset() const { return offset_; }
    bool entry(size_t index, EntryView &out) const;
    bool contains(const char *path) const;

private:
    friend struct Builder;
    Snapshot() = default;
    Snapshot(const Snapshot &) = delete;
    Snapshot &operator=(const Snapshot &) = delete;
    struct Entry {
        uint64_t size;
        int64_t modified;
        uint32_t path;
        uint32_t next;
        bool bootloader;
        bool directory;
    };
    void *packed_ = nullptr;
    size_t count_ = 0;
    uint32_t revision_ = 0;
    bool more_ = false;
    char path_[MAX_PATH] = {};
    uint32_t offset_ = 0;
};

struct Status {
    bool supported = false;
    bool available = false;
    bool refreshing = false;
    bool ready = false;
    uint32_t revision = 0;
    uint32_t files_revision = 0;
    uint16_t entries = 0;
    char error[48] = {};
};

// poll publishes card availability and file revisions; it does not scan SD.
void init();
void poll();
void request_refresh();
void get_status(Status &out);
// CLI retains one requested directory page. HTTP uses StorageBrowser directly
// so clients navigate independently. Repeating a completed request reuses it.
bool request_listing(const char *path, uint32_t offset = 0);
std::shared_ptr<const Snapshot> snapshot();
// Pins entries and copies their publication status in one owner lock. A null
// result is paired with not-ready status; never join two independent reads.
std::shared_ptr<const Snapshot> snapshot(Status &out);

// Exact absolute file identities within ROOT, including ordinary subdirectories.
// The bootloaders subtree permits only <BID>/patched.bin.
// No traversal, hidden components or .part files.
bool valid_filename(const char *filename);
bool valid_directory(const char *path);
bool is_bootloader(const char *path);
bool valid_path(const char *path);
bool bootloader_path(const char *bid, char *out, size_t capacity);
// Operation dumps/upload workers call only AFTER their atomic publication.
// Rename/delete/download continue to use StorageBrowser, not this owner.
bool notify_published(const char *path);

}  // namespace ResmedFirmwareRepository
