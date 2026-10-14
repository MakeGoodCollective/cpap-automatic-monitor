#pragma once

#include <stdint.h>
#include <stddef.h>
#include <type_traits>

namespace fs {
class FS;
class File;
}

namespace SdStorage {

constexpr size_t READ_CHUNK_BYTES = 4096;
constexpr size_t WRITE_CHUNK_BYTES = 4096;

enum class Mode : uint8_t { Local, ToUsb, Usb, ToLocal };

struct Status {
    bool supported;
    bool mounted;
    uint64_t card_bytes;
    uint64_t total_bytes;
    uint64_t used_bytes;
    char error[48];
    Mode mode = Mode::Local;
    bool usb_supported = false;
};

// Binary storage records use little-endian fields regardless of alignment.
inline uint16_t get_le16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(data[1]) << 8;
}

inline uint32_t get_le32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0]) |
           static_cast<uint32_t>(data[1]) << 8 |
           static_cast<uint32_t>(data[2]) << 16 |
           static_cast<uint32_t>(data[3]) << 24;
}

inline void put_le16(uint8_t *data, uint16_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}

inline void put_le32(uint8_t *data, uint32_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value >> 16);
    data[3] = static_cast<uint8_t>(value >> 24);
}

bool write_exact(fs::File &file, const uint8_t *data, size_t size);

// Boot coordinator only, before init or any worker/network/recorder starts.
// Replaces the card partition table and creates one empty FAT32 partition.
// Logical format, not secure erase. Leaves the card unmounted; false on failure
// or unsupported builds. Once init is attempted, formatting is forbidden.
bool factory_format();
void init();
bool mounted();
fs::FS *filesystem();
void refresh_usage();
void get_status(Status &out);
const char *state_name(const Status &status);

// Admission only; the SD worker drains admitted I/O before changing media.
// Disabling USB also works with a connected host; its unwritten cache is lost.
bool request_usb(bool enabled, const char **error = nullptr);
bool local_access_allowed();

// USB transport callbacks. Buffer and completion remain owned until done.
bool usb_transfer(bool write, uint32_t sector, uint8_t *buffer, size_t size,
                  void (*done)(bool));
void usb_ejected();

// Explicit rename/delete steps, including partial jobs, and successful mounts.
// Recorder/catalog commits and private cache writes have separate revisions.
uint32_t files_revision();
void notify_files_changed();

// Recorder/recovery only. Waits for current local I/O, never for a network
// consumer; acquisition invalidates all previously admitted background work.
bool acquire();
// Maintenance never revokes an auxiliary session. reserve_next keeps the next
// free slot for this caller after a miss, through all steps of its maintenance
// pass. finish_maintenance releases an admitted pass; an ungranted reservation
// remains for retry. cancel_acquire abandons it even before admission.
// Existing sessions remain valid so cooperative workers can checkpoint first.
bool try_acquire(bool reserve_next = false);
void finish_maintenance();
void cancel_acquire();
bool maintenance_pending();
// Nonblocking admission hint for the reserving owner's existing work loop.
// A true result still requires try_acquire; it does not take the card.
bool maintenance_ready();
void release();

class Session {
public:
    // One auxiliary operation at a time. Copies borrow the token; the owner
    // must end it after closing its Readers/Writer. Recorder acquisition revokes it.
    bool begin();
    void end();
    bool valid() const;

    // Blocking task API, not for HTTP/RX callbacks. The operation must finish
    // one bounded filesystem step and must not retain File objects or do UART
    // or network work. Caller context remains alive until completion.
    bool run(bool (*operation)(fs::FS &, void *), void *context) const;
    template <typename F> bool run(F &&operation) const {
        return run([](fs::FS &fs, void *context) {
            return (*static_cast<typename std::remove_reference<F>::type *>(context))(fs);
        }, &operation);
    }

private:
    uint32_t generation_ = 0;
};

class Reader {
public:
    struct Entry {
        char name[256];
        uint64_t size;
        int64_t modified;
        bool directory;
    };
    Reader() = default;
    ~Reader() { close(); }
    Reader(const Reader &) = delete;
    Reader &operator=(const Reader &) = delete;
    bool open(const Session &session, const char *path, bool directory = false);
    bool next(Entry &entry, bool &end);
    // Absolute file offset through UINT32_MAX, including EOF; failure keeps offset.
    bool seek(uint64_t offset);
    size_t read(uint8_t *data, size_t length);
    void close();
    uint64_t size() const { return size_; }
    int64_t modified() const { return modified_; }
    explicit operator bool() const { return handle_ != 0; }

private:
    Session session_;
    uint32_t handle_ = 0;
    uint64_t size_ = 0;
    int64_t modified_ = 0;
    uint64_t offset_ = 0;
};

// One sequential output alongside the two readers, all held by the SD worker.
// For replaceable files: create/truncate a temporary path, finish, then publish
// via Session::run. Revocation closes it before recorder/USB access begins.
class Writer {
public:
    Writer() = default;
    ~Writer() { close(); }
    Writer(const Writer &) = delete;
    Writer &operator=(const Writer &) = delete;
    // Nonzero append_at resumes a private build only at its exact flushed size.
    bool open(const Session &session, const char *path, uint64_t append_at = 0);
    bool write(const uint8_t *data, size_t length);
    // Make the completed prefix readable without closing the sequential writer.
    // Only private, unpublished build files may be read while still growing.
    bool flush(uint64_t expected_size);
    // Flush, check the complete length and close, including on failure.
    bool finish(uint64_t expected_size);
    void close();

private:
    Session session_;
    uint32_t handle_ = 0;
    uint64_t size_ = 0;
};

}  // namespace SdStorage
