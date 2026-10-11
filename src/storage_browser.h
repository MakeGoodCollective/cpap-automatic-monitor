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

struct MutationStatus {
    bool active;
    bool succeeded;
    uint32_t changed;
    char error[48];
};

// One SD operation. Immutable reads admit one bounded waiting successor while
// the previous response is being released. Ready runs outside the SD executor.
// Size zero for archives denotes HTTP chunked transfer.
// Mutations return no Transfer; completion is also retained for the CLI.
StartResult start(const Request &request, Ready ready, std::weak_ptr<Transfer> &active);
void mutation_status(MutationStatus &out);

}  // namespace StorageBrowser
