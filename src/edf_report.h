#pragma once

#include <stddef.h>
#include <stdint.h>
#include "storage_browser.h"

// Local EDF ownership is independent of the UART/ClinicalJobs report path.
namespace EdfReport {

enum class State : uint8_t { Ready, WaitingStorage, Building, Blocked, Error };
enum class Action : uint8_t { Days, Summary, Series, History, Prepare };
enum class View : uint8_t { Day, Period };

constexpr size_t MAX_SESSION_COUNT = 8192;

struct Request {
    Action action = Action::Summary;
    View view = View::Day;
    uint16_t day = 0;       // Native epoch day; zero selects latest catalog day.
    uint16_t period = 1;    // Actual days: 1/7/30/90/180/365, ending at day.
    int64_t from_ms = 0, to_ms = 0;
    uint16_t px = 800;      // Shared aligned min/max buckets, at most 1600.
    uint32_t revision = 0;  // Dataset revision, NOT status.revision; zero accepts latest.
    // Summary session IDs are catalog indexes valid only for that data revision.
    // Bit n excludes session n from series values and events, never statistics.
    uint8_t excluded[MAX_SESSION_COUNT / 8] = {};

    bool excludes(size_t id) const {
        return id < MAX_SESSION_COUNT && (excluded[id / 8] & (1u << (id % 8)));
    }
    bool operator==(const Request &other) const;
};

struct Status {
    bool supported = false, mounted = false, available = false;
    State state = State::Blocked;
    uint32_t revision = 0, data_revision = 0, artifact_revision = 0;
    uint32_t catalog_generation = 0, files_revision = 0;
    uint32_t sessions = 0;
    char error[48] = {};
};

void init();
void tick();  // Nonblocking maintenance/wakeup; no filesystem or EDF parsing.
void get_status(Status &out);
uint32_t revision();
const char *state_name(State state);
bool valid(const Request &request);

// Immutable selectors, not user filesystem paths. The shared bounded file
// producer serves bytes; no report computation or SD access in HTTP callbacks.
StorageBrowser::StartResult open_artifact(const char *day, const char *digest,
    bool document, StorageBrowser::Request &ranges, StorageBrowser::Ready ready,
    std::weak_ptr<StorageBrowser::Transfer> &active);

// Two active jobs, coalesced identical requests, bounded PSRAM result retention. Admission
// and polling do no SD work. Completion changes status.revision only; source
// invalidation also changes status.data_revision. JSON carries both revisions.
bool submit(const Request &request, uint32_t &id, const char **error = nullptr);
uint32_t prepare_before_export(const char *therapy_day);
bool preparation_pending(uint32_t id);
void cancel(uint32_t id);
// Cancellation is best-effort for sole requests; coalesced jobs are not cancelled
// by one consumer. Pinned results remain valid until their last lease releases.

// Immutable payload pinned until reset/destruction, including after invalidation
// or cancellation. The HTTP owner releases its lease on completion/disconnect.
class Result {
public:
    Result() = default;
    ~Result();
    Result(const Result &) = delete;
    Result &operator=(const Result &) = delete;
    Result(Result &&other) noexcept;
    bool available() const { return id_ != 0; }
    size_t length() const { return length_; }
    size_t read(size_t offset, char *out, size_t capacity) const;
    void reset();

private:
    friend int poll(uint32_t id, Result &out);
    uint32_t id_ = 0;
    const char *data_ = nullptr;
    size_t length_ = 0;
};

// 202 pending; 200 payload; 400/409/410/422/503 terminal error payload or expired.
int poll(uint32_t id, Result &out);

}  // namespace EdfReport
