#pragma once

#include <stddef.h>
#include <stdint.h>
#include <functional>
#include <memory>

namespace StorageBrowser {

enum class Kind { List, File, Archive, Rename, Delete, Immutable };
enum class StartResult { Started, BadRequest, Busy, Unavailable };

struct Request {
    Kind kind;
    char path[256];
    char selection[2048]; // Child names separated by newlines, or new name for Rename.
    uint32_t offset;
    struct Range { uint32_t first = 0, last = 0; } ranges[8];
    uint8_t range_count = 0;
    bool head = false, unchanged = false;
};

class Transfer {
public:
    virtual ~Transfer() = default;
    virtual size_t read(uint8_t *out, size_t capacity) = 0;
    virtual bool failed() const = 0;
    virtual bool finished() const = 0;
    virtual void cancel() = 0;
    virtual uint64_t representation_size() const { return 0; }
};

using Ready = std::function<void(int code, const char *error,
    std::shared_ptr<Transfer> transfer, uint64_t size)>;
using Released = void (*)();

struct MutationStatus {
    bool active;
    bool succeeded;
    uint32_t changed;
    char error[48];
};

// At most two active read responses and four FIFO waiting descriptors. Waiting
// reads have no worker, transfer buffers or SD admission. Started includes
// queued work; activation failures are delivered through Ready. Cancellation
// removes a waiting descriptor immediately. Mutations are not queued.
// Ready runs outside the SD executor, including immediate activation failure.
// Size zero for archives denotes HTTP chunked transfer.
// Mutations return no Transfer; completion is also retained for the CLI.
// Released runs once after ending SD access, or without a session for work
// cancelled/failed before activation. It is not called unless start returns
// Started. An active slot outlives the producer until its response is released.
StartResult start(const Request &request, Ready ready, std::weak_ptr<Transfer> &active,
                  Released released = nullptr);
void mutation_status(MutationStatus &out);

}  // namespace StorageBrowser
