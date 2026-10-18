#pragma once

#include "report_artifact.h"
#include "report_edf.h"
#include "sd_storage.h"
#include "large_text_buffer.h"

namespace ReportArtifact {

struct Selection {
    uint16_t id;
    uint16_t signal;
    double scale;
    const char *label;
    const char *unit;
};

struct Publication {
    uint8_t digest[32] = {}, source[32] = {};
    uint32_t bytes = 0;
};

enum class Lookup : uint8_t { Missing, Ready, Invalid, Failed };

// Prefix: an owner-generated /airbridge/report/YYYYMMDD/session_KIND path.
// Fingerprint: cheap source size/sidecar identity, distinct from content SHA256.
// Calls require an owner worker with a valid storage session, never HTTP/RX.
Lookup lookup(const SdStorage::Session &session, const char *prefix,
              const uint8_t fingerprint[32], Publication &out);
bool fingerprint(const uint8_t *bytes, size_t length, uint8_t out[32]);

struct Document {
    uint8_t digest[32] = {}, key[32] = {};
    uint32_t bytes = 0, generation = 0;
};

// Lookup reads only CRC-protected heads. Read verifies the selected body once
// and may fall back to the other head, updating document on success. A non-null
// key forbids fallback to a publication of different sources.
Lookup document_lookup(const SdStorage::Session &session, const char *prefix,
                       const uint8_t *key, Document &out);
// Retention traverses both committed generations, never just the newest one.
Lookup document_slot(const SdStorage::Session &session, const char *prefix,
                     unsigned slot, Document &out);
bool document_read(const SdStorage::Session &session, const char *prefix,
                   Document &document, aircannect::LargeTextBuffer &out,
                   const uint8_t *key, bool fallback = true,
                   bool (*cancelled)(void *) = nullptr, void *context = nullptr);
bool document_publish(const SdStorage::Session &session, const char *prefix,
                      const uint8_t key[32], const uint8_t *data, size_t length,
                      Document &out);

// Root index only. Return the displaced head after a successful commit; the
// report owner retires it separately, excluding active byte-read admissions.
bool index_publish(const SdStorage::Session &session, const uint8_t key[32],
                   const uint8_t *data, size_t length, Document &out, Document &retired);
// Never removes content referenced by either index, dataset or rebuild head.
// Caller proves the candidate is an index. Its optional cancellation predicate
// rechecks owner pins inside the serialized mutation, without I/O or waits.
bool index_discard(const SdStorage::Session &session, const uint8_t digest[32],
                   bool (*cancelled)(void *) = nullptr, void *context = nullptr);
// Read an unreferenced root body once, verifying its content hash. Missing
// means protected by a head; Invalid means malformed, Failed means retry.
// The owner must still establish that this body is an index before removal.
Lookup index_candidate(const SdStorage::Session &session, const uint8_t digest[32],
                       uint32_t bytes, aircannect::LargeTextBuffer &out,
                       bool (*cancelled)(void *) = nullptr, void *context = nullptr);

// One source, one record traversal, all selected tracks. Scratch lives in PSRAM
// and is independent of duration. Base transposition and x4 passes read the
// private binary spool, never source EDF again. The same 4 KiB input buffer
// batches nearby spans; reduction checks cancellation at most 64 cells apart.
// No global state or new worker.
class Producer {
public:
    // The token is copied; cancellation context must outlive the producer.
    Producer(const SdStorage::Session &session, bool (*cancelled)(void *), void *context);
    ~Producer();
    Producer(const Producer &) = delete;
    Producer &operator=(const Producer &) = delete;
    bool begin(const char *prefix, const uint8_t fingerprint[32],
               const ReportEdf::Header &header, const Selection *tracks, size_t count);
    // Close every handle before the owner ends its session. Scratch, digests and
    // cursors stay in PSRAM; resume borrows a copy of the new session token.
    void pause();
    // Clears interruption only, never a permanent producer error.
    bool resume(const SdStorage::Session &session);
    uint32_t processed_records() const;
    // Logical phase and completed bytes; stable across pause/resume.
    uint64_t progress() const;
    // Retry the same record after interruption. Only a complete spool record
    // advances processed_records() and the source digest.
    bool record(const ReportEdf::Header &header, const uint8_t *data, size_t bytes);
    // False with error()=="interrupted" is resumable, including inside a tile
    // or verification pass. Successful finish is idempotent. Reboot rebuilds
    // the source spool; tile checkpoints follow level boundaries after 1 MiB
    // of new output, never after the final level. Lost tile output can span
    // that budget plus one complete level, which can itself be large.
    bool finish(Publication &out);
    const char *error() const;

private:
    struct Work;
    Work *work_ = nullptr;
};

}  // namespace ReportArtifact
