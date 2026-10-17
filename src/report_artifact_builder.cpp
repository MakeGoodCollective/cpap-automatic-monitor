#include "board.h"
#if AB_STORAGE_HAS_SDCARD
#include "report_artifact_builder.h"
#include "crc.h"
#include "hex_util.h"
#include "memory_manager.h"
#include <FS.h>
#include <mbedtls/sha256.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>

namespace ReportArtifact {
struct Digest {
    mbedtls_sha256_context context;
    bool ready;
    Digest() { mbedtls_sha256_init(&context); ready = mbedtls_sha256_starts(&context, 0) == 0; }
    ~Digest() { mbedtls_sha256_free(&context); }
    bool add(const uint8_t *data, size_t length) {
        return ready && mbedtls_sha256_update(&context, data, length) == 0;
    }
    bool finish(uint8_t out[32]) { return ready && mbedtls_sha256_finish(&context, out) == 0; }
    bool snapshot(uint8_t out[32]) const {
        Digest copy;
        mbedtls_sha256_clone(&copy.context, &context);
        return ready && copy.finish(out);
    }
};

namespace {
constexpr size_t PATH_BYTES = 192;
constexpr size_t REFERENCE_BYTES = 116;

bool named_path(const char *prefix, const char *suffix, char (&out)[PATH_BYTES]) {
    const int count = snprintf(out, sizeof(out), "%s.%s", prefix, suffix);
    return count > 0 && size_t(count) < sizeof(out);
}
bool reference_path(const char *prefix, const uint8_t key[32], char (&out)[PATH_BYTES]) {
    char text[65];
    return aircannect::hex_encode(key, 32, text, sizeof(text), aircannect::HexCase::Lower) &&
        snprintf(out, sizeof(out), "%s.%s.ref", prefix, text) < int(sizeof(out));
}
bool content_path(const char *prefix, const uint8_t hash[32], char (&out)[PATH_BYTES], bool document = false) {
    char text[65];
    const char *end = strrchr(prefix, '/');
    if (!end || !aircannect::hex_encode(hash, 32, text, sizeof(text), aircannect::HexCase::Lower)) return false;
    return snprintf(out, sizeof(out), "%.*s/%.*s%s%s.%s", int(end - prefix), prefix,
        document ? 2 : 0, text, document ? "/" : "", text, document ? "abp" : "abt") < int(sizeof(out));
}

bool exact_read(SdStorage::Reader &reader, uint64_t offset, uint8_t *out, size_t count) {
    if (!reader.seek(offset)) return false;
    for (size_t done = 0; done < count;) {
        const size_t take = std::min(count - done, SdStorage::READ_CHUNK_BYTES);
        if (reader.read(out + done, take) != take) return false;
        done += take;
    }
    return true;
}

void reference_bytes(const uint8_t key[32], const Publication &publication,
                     uint8_t (&bytes)[REFERENCE_BYTES]) {
    memset(bytes, 0, sizeof(bytes));
    memcpy(bytes, "ABAREF01", 8);
    SdStorage::put_le32(bytes + 8, publication.bytes);
    memcpy(bytes + 12, key, 32);
    memcpy(bytes + 44, publication.digest, 32);
    memcpy(bytes + 76, publication.source, 32);
    SdStorage::put_le32(bytes + 112, crc32_ieee(bytes, 112));
}
}  // namespace

bool fingerprint(const uint8_t *bytes, size_t length, uint8_t out[32]) {
    return mbedtls_sha256(bytes, length, out, 0) == 0;
}

Lookup lookup(const SdStorage::Session &session, const char *prefix,
              const uint8_t key[32], Publication &out) {
    out = {};
    char path[PATH_BYTES];
    if (!reference_path(prefix, key, path)) return Lookup::Failed;
    bool exists = false;
    if (!session.run([&](fs::FS &fs) { exists = fs.exists(path); return true; })) return Lookup::Failed;
    if (!exists) return Lookup::Missing;
    uint8_t bytes[REFERENCE_BYTES];
    SdStorage::Reader reader;
    if (!reader.open(session, path)) return Lookup::Failed;
    if (reader.size() != sizeof(bytes)) return Lookup::Invalid;
    if (!exact_read(reader, 0, bytes, sizeof(bytes))) return Lookup::Failed;
    if (memcmp(bytes, "ABAREF01", 8) || memcmp(bytes + 12, key, 32) ||
        SdStorage::get_le32(bytes + 108) ||
        crc32_ieee(bytes, 112) != SdStorage::get_le32(bytes + 112)) return Lookup::Invalid;
    out.bytes = SdStorage::get_le32(bytes + 8);
    memcpy(out.digest, bytes + 44, 32);
    memcpy(out.source, bytes + 76, 32);
    reader.close();
    if (!content_path(prefix, out.digest, path) || !reader.open(session, path)) return Lookup::Failed;
    if (reader.size() != out.bytes || !exact_read(reader, 0, bytes, HEADER_BYTES)) return Lookup::Invalid;
    if (memcmp(bytes, "ABTILE01", 8) || SdStorage::get_le16(bytes + 8) != VERSION ||
        SdStorage::get_le32(bytes + 12) != out.bytes || memcmp(bytes + 20, out.source, 32)) return Lookup::Invalid;
    return Lookup::Ready;
}

namespace {
struct DocumentScratch {
    char path[PATH_BYTES], part[PATH_BYTES];
    uint8_t block[SdStorage::READ_CHUNK_BYTES];
};
static_assert(sizeof(DocumentScratch) <= 4608, "bounded document scratch fallback");

bool document_path(const char *prefix, const uint8_t hash[32], char (&out)[PATH_BYTES]) {
    return content_path(prefix, hash, out, true);
}

bool document_content(const SdStorage::Session &session, const char *prefix,
                      const Document &document, aircannect::LargeTextBuffer &out,
                      DocumentScratch &scratch, bool (*cancelled)(void *) = nullptr, void *context = nullptr) {
    auto &path = scratch.path;
    auto &bytes = scratch.block;
    SdStorage::Reader reader;
    out.clear();
    if (!document_path(prefix, document.digest, path) || !reader.open(session, path) ||
        reader.size() != document.bytes || !out.reserve(document.bytes)) return false;
    Digest digest;
    uint8_t hash[32];
    for (size_t at = 0; at < document.bytes;) {
        if (cancelled && cancelled(context)) { out.clear(); return false; }
        const size_t count = std::min(sizeof(bytes), size_t(document.bytes - at));
        if (reader.read(bytes, count) != count || !digest.add(bytes, count) ||
            !out.append(reinterpret_cast<const char *>(bytes), count)) {
            out.clear();
            return false;
        }
        at += count;
    }
    const bool valid = digest.finish(hash) && !memcmp(hash, document.digest, 32);
    if (!valid) out.clear();
    return valid;
}

Lookup document_head(const SdStorage::Session &session, const char *prefix, unsigned slot,
                     Document &out, char (&path)[PATH_BYTES]) {
    snprintf(path, sizeof(path), "%s.head%u", prefix, slot);
    bool exists = false;
    if (!session.run([&](fs::FS &fs) { exists = fs.exists(path); return true; })) return Lookup::Failed;
    if (!exists) return Lookup::Missing;
    SdStorage::Reader reader;
    uint8_t bytes[84];
    if (!reader.open(session, path) || !exact_read(reader, 0, bytes, sizeof(bytes))) return Lookup::Failed;
    if (reader.size() != sizeof(bytes) || memcmp(bytes, "ABDOC001", 8) ||
        crc32_ieee(bytes, 80) != SdStorage::get_le32(bytes + 80)) return Lookup::Invalid;
    out.generation = SdStorage::get_le32(bytes + 8);
    out.bytes = SdStorage::get_le32(bytes + 12);
    if (!out.bytes || (out.generation & 1) != slot) return Lookup::Invalid;
    memcpy(out.key, bytes + 16, 32);
    memcpy(out.digest, bytes + 48, 32);
    reader.close();
    return Lookup::Ready;
}

Lookup lookup_document(const SdStorage::Session &session, const char *prefix,
                       const uint8_t *key, Document &out, char (&path)[PATH_BYTES],
                       Document *replaced = nullptr) {
    Document heads[2];
    const Lookup a = document_head(session, prefix, 0, heads[0], path);
    const Lookup b = document_head(session, prefix, 1, heads[1], path);
    int chosen = -1;
    for (unsigned i = 0; i < 2; ++i) {
        if ((i ? b : a) != Lookup::Ready || (key && memcmp(key, heads[i].key, 32))) continue;
        if (chosen < 0 || int32_t(heads[i].generation - heads[chosen].generation) > 0) chosen = i;
    }
    if (chosen >= 0) {
        out = heads[chosen];
        if (replaced && (chosen ? a : b) == Lookup::Ready) *replaced = heads[chosen ^ 1];
        return Lookup::Ready;
    }
    if (a == Lookup::Failed || b == Lookup::Failed) return Lookup::Failed;
    if (a == Lookup::Invalid || b == Lookup::Invalid) return Lookup::Invalid;
    return Lookup::Missing;
}
}  // namespace

Lookup document_lookup(const SdStorage::Session &session, const char *prefix,
                       const uint8_t *key, Document &out) {
    char path[PATH_BYTES];
    return lookup_document(session, prefix, key, out, path);
}

Lookup document_slot(const SdStorage::Session &session, const char *prefix,
                     unsigned slot, Document &out) {
    if (slot > 1) return Lookup::Invalid;
    char path[PATH_BYTES];
    return document_head(session, prefix, slot, out, path);
}

bool document_read(const SdStorage::Session &session, const char *prefix,
                   Document &document, aircannect::LargeTextBuffer &out,
                   const uint8_t *key, bool fallback, bool (*cancelled)(void *), void *context) {
    out.clear();
    if (key && memcmp(key, document.key, 32)) return false;
    void *memory = aircannect::Memory::alloc_large(sizeof(DocumentScratch));
    if (!memory) return false;
    std::unique_ptr<DocumentScratch, decltype(&aircannect::Memory::free)> scratch(
        new (memory) DocumentScratch, aircannect::Memory::free);
    if (document_content(session, prefix, document, out, *scratch, cancelled, context)) return true;
    if (!fallback || !session.valid() || out.overflowed() || (cancelled && cancelled(context))) return false;
    Document previous;
    if (document_head(session, prefix, unsigned((document.generation & 1) ^ 1), previous, scratch->path) != Lookup::Ready ||
        int32_t(previous.generation - document.generation) >= 0 ||
        (key && memcmp(key, previous.key, 32)) || !memcmp(previous.digest, document.digest, 32) ||
        !document_content(session, prefix, previous, out, *scratch, cancelled, context)) return false;
    document = previous;
    return true;
}

static bool publish_document(const SdStorage::Session &session, const char *prefix,
    const uint8_t key[32], const uint8_t *data, size_t length, Document &out, Document *retired) {
    if (retired) *retired = {};
    if (!length || length > UINT32_MAX) return false;
    void *memory = aircannect::Memory::alloc_large(sizeof(DocumentScratch));
    if (!memory) return false;
    std::unique_ptr<DocumentScratch, decltype(&aircannect::Memory::free)> scratch(
        new (memory) DocumentScratch, aircannect::Memory::free);
    // Only heads are needed to choose the next generation and inactive slot.
    Document previous, replaced;
    const Lookup old = lookup_document(session, prefix, nullptr, previous, scratch->path, &replaced);
    if (old == Lookup::Failed || old == Lookup::Invalid) return false;
    out = {};
    out.bytes = length;
    out.generation = old == Lookup::Ready ? previous.generation + 1 : 1;
    memcpy(out.key, key, 32);
    if (!fingerprint(data, length, out.digest)) return false;
    auto &path = scratch->path;
    auto &part = scratch->part;
    if (!document_path(prefix, out.digest, path) || !named_path(prefix, "document.part", part)) return false;
    bool exists = false;
    if (!session.mutate(part, path, [&](fs::FS &fs) {
        if ((!fs.exists("/airbridge") && !fs.mkdir("/airbridge")) ||
            (!fs.exists("/airbridge/report") && !fs.mkdir("/airbridge/report"))) return false;
        char directory[PATH_BYTES];
        snprintf(directory, sizeof(directory), "%s", prefix);
        *strrchr(directory, '/') = 0;
        if (!fs.exists(directory) && !fs.mkdir(directory)) return false;
        snprintf(directory, sizeof(directory), "%s", path);
        *strrchr(directory, '/') = 0;
        if (!fs.exists(directory) && !fs.mkdir(directory)) return false;
        exists = fs.exists(path);
        return true;
    })) return false;
    SdStorage::Writer writer;
    if (!exists) {
        if (!writer.open(session, part) || !writer.write(data, length) || !writer.finish(length)) return false;
    }
    // Verify persisted bytes before publishing a head, including reused content.
    SdStorage::Reader reader;
    if (!reader.open(session, exists ? path : part) || reader.size() != length) return false;
    auto &block = scratch->block;
    for (size_t at = 0; at < length;) {
        const size_t count = std::min(sizeof(block), length - at);
        if (reader.read(block, count) != count || memcmp(block, data + at, count)) return false;
        at += count;
    }
    reader.close();
    if (!exists && !session.mutate(part, path, [&](fs::FS &fs) { return fs.rename(part, path); })) return false;
    if (old == Lookup::Ready && !memcmp(previous.digest, out.digest, 32) &&
        !memcmp(previous.key, key, 32)) { out = previous; return true; }
    uint8_t head[84] = {};
    memcpy(head, "ABDOC001", 8);
    SdStorage::put_le32(head + 8, out.generation);
    SdStorage::put_le32(head + 12, out.bytes);
    memcpy(head + 16, key, 32);
    memcpy(head + 48, out.digest, 32);
    SdStorage::put_le32(head + 80, crc32_ieee(head, 80));
    if (!writer.open(session, part) || !writer.write(head, sizeof(head)) || !writer.finish(sizeof(head))) return false;
    if (!reader.open(session, part) || reader.size() != sizeof(head) ||
        reader.read(block, sizeof(head)) != sizeof(head) || memcmp(block, head, sizeof(head))) return false;
    reader.close();
    snprintf(path, sizeof(path), "%s.head%u", prefix, unsigned(out.generation & 1));
    const bool committed = session.mutate(part, path, [&](fs::FS &fs) {
        // Only the inactive head is replaced. The previous content/head survive.
        return (!fs.exists(path) || fs.remove(path)) && fs.rename(part, path);
    });
    if (committed && retired) *retired = replaced;
    return committed;
}

bool document_publish(const SdStorage::Session &session, const char *prefix,
                      const uint8_t key[32], const uint8_t *data, size_t length, Document &out) {
    return publish_document(session, prefix, key, data, length, out, nullptr);
}

bool index_publish(const SdStorage::Session &session, const uint8_t key[32],
                   const uint8_t *data, size_t length, Document &out, Document &retired) {
    return publish_document(session, "/airbridge/report/index", key, data, length, out, &retired);
}

static Lookup root_reference(const SdStorage::Session &session, const uint8_t digest[32]) {
    char path[PATH_BYTES];
    for (const char *prefix : {"/airbridge/report/index", "/airbridge/report/dataset", "/airbridge/report/rebuild"}) {
        for (unsigned slot = 0; slot < 2; ++slot) {
            Document retained;
            const auto state = document_head(session, prefix, slot, retained, path);
            if (state == Lookup::Failed || state == Lookup::Invalid) return state;
            if (state == Lookup::Ready && !memcmp(retained.digest, digest, 32)) return Lookup::Ready;
        }
    }
    return Lookup::Missing;
}

bool index_discard(const SdStorage::Session &session, const uint8_t digest[32],
                   bool (*cancelled)(void *), void *context) {
    const auto state = root_reference(session, digest);
    if (state == Lookup::Ready) return true;
    if (state != Lookup::Missing) return false;
    char path[PATH_BYTES];
    if (!document_path("/airbridge/report/index", digest, path)) return false;
    // Root documents can retain dependencies outside the document's own path.
    return session.mutate("/airbridge/report", [&](fs::FS &fs) {
        if (cancelled && cancelled(context)) return false;
        return !fs.exists(path) || fs.remove(path);
    });
}

Lookup index_candidate(const SdStorage::Session &session, const uint8_t digest[32],
                       uint32_t bytes, aircannect::LargeTextBuffer &out, bool (*cancelled)(void *), void *context) {
    const auto state = root_reference(session, digest);
    if (state == Lookup::Ready) return Lookup::Missing;
    if (state != Lookup::Missing) return Lookup::Failed;
    void *memory = aircannect::Memory::alloc_large(sizeof(DocumentScratch));
    if (!memory) return Lookup::Failed;
    std::unique_ptr<DocumentScratch, decltype(&aircannect::Memory::free)> scratch(
        new (memory) DocumentScratch, aircannect::Memory::free);
    Document candidate;
    candidate.bytes = bytes;
    memcpy(candidate.digest, digest, 32);
    if (document_content(session, "/airbridge/report/index", candidate, out, *scratch, cancelled, context)) return Lookup::Ready;
    return !session.valid() || out.overflowed() || (cancelled && cancelled(context)) ? Lookup::Failed : Lookup::Invalid;
}

struct Producer::Work {
    const SdStorage::Session &session;
    bool (*cancelled)(void *);
    void *context;
    const char *failure = nullptr;
    Metadata metadata;
    uint16_t signals[MAX_TRACKS] = {};
    uint32_t offsets[MAX_TRACKS] = {}, stride = 0, records = 0, written_records = 0;
    uint8_t key[32] = {};
    char prefix[96] = {}, spool_path[PATH_BYTES] = {}, part_path[PATH_BYTES] = {};
    SdStorage::Writer writer;
    Digest source, content;
    uint64_t made = 0;
    bool started = false, finished = false, published = false;
    uint8_t input[REDUCED_TILE_BYTES], output[MAX_METADATA_BYTES];

    Work(const SdStorage::Session &session, bool (*cancelled)(void *), void *context)
        : session(session), cancelled(cancelled), context(context) {}
    ~Work() {
        writer.close();
        // Only unpublished scratch paths are removed. A revoked session leaves
        // them for the next explicit attempt, never for a filesystem callback.
        if (!published) return;
        for (const char *path : {spool_path, part_path}) if (*path)
            session.mutate(path, [&](fs::FS &fs) { return !fs.exists(path) || fs.remove(path); });
        char scratch[PATH_BYTES];
        if (*prefix && named_path(prefix, "resume.part", scratch))
            session.mutate(scratch, [&](fs::FS &fs) { return !fs.exists(scratch) || fs.remove(scratch); });
    }
    bool fail(const char *reason) { if (!failure) failure = reason; return false; }
    bool active() {
        if (failure) return false;
        if (!session.valid() || (cancelled && cancelled(context))) return fail("interrupted");
        return true;
    }
    bool append(const uint8_t *bytes, size_t length) {
        if (!active() || !writer.write(bytes, length) || !content.add(bytes, length)) return fail("artifact_write");
        made += length;
        return true;
    }
    bool checkpoint() {
        if (!active() || !writer.finish(made)) return fail("checkpoint_flush");
        uint8_t bytes[48] = {};
        memcpy(bytes, "ABCHK001", 8);
        SdStorage::put_le32(bytes + 8, made);
        if (!content.snapshot(bytes + 16)) return fail("checkpoint_digest");
        char path[PATH_BYTES];
        Document document;
        if (!named_path(prefix, "checkpoint", path) ||
            !document_publish(session, path, metadata.source, bytes, sizeof(bytes), document) ||
            !writer.open(session, part_path, made)) return fail("checkpoint_write");
        return true;
    }
    bool restore() {
        char path[PATH_BYTES], scratch[PATH_BYTES];
        Document document;
        aircannect::LargeTextBuffer saved(128, 128);
        if (!named_path(prefix, "checkpoint", path) ||
            document_lookup(session, path, metadata.source, document) != Lookup::Ready ||
            !document_read(session, path, document, saved, metadata.source) || saved.length() != 48 ||
            memcmp(saved.c_str(), "ABCHK001", 8)) return false;
        const auto *bytes = reinterpret_cast<const uint8_t *>(saved.c_str());
        const uint32_t complete = SdStorage::get_le32(bytes + 8);
        bool boundary = complete == metadata.header_bytes;
        for (size_t c = 0; c < metadata.track_count; ++c)
            for (size_t l = 0; l < metadata.tracks[c].level_count; ++l) {
                const auto &level = metadata.tracks[c].levels[l];
                boundary |= complete == level.offset + level.bytes;
            }
        SdStorage::Reader reader;
        if (!boundary || complete > metadata.bytes || !reader.open(session, part_path) || reader.size() < complete) return false;
        const bool tail = reader.size() != complete;
        if (tail && (!named_path(prefix, "resume.part", scratch) || !writer.open(session, scratch))) return false;
        for (uint32_t at = 0; at < complete;) {
            const size_t take = std::min(sizeof(input), size_t(complete - at));
            if (!active() || reader.read(input, take) != take || !content.add(input, take) ||
                (tail && !writer.write(input, take))) return false;
            at += take;
        }
        uint8_t hash[32];
        if (!content.snapshot(hash) || memcmp(hash, bytes + 16, 32)) return false;
        reader.close();
        if (tail && (!writer.finish(complete) || !session.mutate(scratch, part_path, [&](fs::FS &fs) {
            return fs.remove(part_path) && fs.rename(scratch, part_path);
        }))) return false;
        made = complete;
        return writer.open(session, part_path, made);
    }
    bool base(SdStorage::Reader &spool, size_t c) {
        const auto &track = metadata.tracks[c];
        const auto &level = track.levels[0];
        if (made != level.offset) return fail("artifact_layout");
        for (uint32_t first = 0; first < track.count; first += TILE_SAMPLES) {
            if (!active()) return false;
            memset(output, 0, RAW_TILE_BYTES);
            const uint32_t end = std::min<uint32_t>(track.count, first + TILE_SAMPLES);
            for (uint32_t at = first; at < end;) {
                const uint32_t record = at / track.samples_per_record, slot = at % track.samples_per_record;
                const uint32_t take = std::min(end - at, track.samples_per_record - slot);
                const uint64_t begin = uint64_t(record) * stride + offsets[c];
                const size_t mask_begin = slot / 8, mask_bytes = (slot + take + 7) / 8 - mask_begin;
                if (!exact_read(spool, begin + slot * 2, input, take * 2) ||
                    !exact_read(spool, begin + track.samples_per_record * 2 + mask_begin,
                                input + TILE_SAMPLES * 2, mask_bytes)) return fail("spool_read");
                for (uint32_t n = 0; n < take; ++n) {
                    const uint32_t bit = slot + n - mask_begin * 8;
                    const bool valid = (input[TILE_SAMPLES * 2 + bit / 8] >> (bit % 8)) & 1;
                    put_raw(output, at + n - first, int16_t(SdStorage::get_le16(input + n * 2)), valid);
                }
                at += take;
            }
            if (!append(output, RAW_TILE_BYTES)) return false;
        }
        return true;
    }
    bool reduce(size_t c, size_t l) {
        const auto &track = metadata.tracks[c];
        const auto &previous = track.levels[l - 1], &level = track.levels[l];
        if (made != level.offset || !writer.flush(made)) return fail("artifact_flush");
        SdStorage::Reader reader;
        if (!reader.open(session, part_path)) return fail("artifact_open");
        const size_t input_bytes = l == 1 ? RAW_TILE_BYTES : REDUCED_TILE_BYTES;
        uint32_t loaded = UINT32_MAX;
        for (uint32_t first = 0; first < level.count; first += TILE_SAMPLES) {
            if (!active()) return false;
            memset(output, 0, REDUCED_TILE_BYTES);
            const uint32_t end = std::min<uint32_t>(level.count, first + TILE_SAMPLES);
            for (uint32_t at = first; at < end; ++at) {
                Cell cell;
                for (uint32_t p = at * 4; p < std::min(previous.count, at * 4 + 4); ++p) {
                    const uint32_t tile = p / TILE_SAMPLES;
                    if (loaded != tile) {
                        if (!exact_read(reader, previous.offset + uint64_t(tile) * input_bytes,
                                        input, input_bytes)) return fail("artifact_read");
                        loaded = tile;
                    }
                    const Cell next = l == 1 ? raw_cell(input, p % TILE_SAMPLES) : get_cell(input, p % TILE_SAMPLES);
                    if (next.valid > expected(track, previous, p) || !merge(cell, next)) return fail("artifact_cell");
                }
                put_cell(output, at - first, cell);
            }
            if (!append(output, REDUCED_TILE_BYTES)) return false;
        }
        return true;
    }
    bool verify(const char *path, const Publication &publication) {
        SdStorage::Reader reader;
        if (!reader.open(session, path) || reader.size() != publication.bytes) return fail("artifact_length");
        Digest digest;
        for (uint64_t at = 0; at < reader.size();) {
            const size_t take = std::min<uint64_t>(sizeof(input), reader.size() - at);
            if (!active() || reader.read(input, take) != take || !digest.add(input, take)) return fail("artifact_verify");
            at += take;
        }
        uint8_t value[32];
        return (digest.finish(value) && !memcmp(value, publication.digest, 32)) || fail("artifact_digest");
    }
    bool publish(const Publication &publication) {
        char final[PATH_BYTES], reference[PATH_BYTES];
        if (!content_path(prefix, publication.digest, final) || !reference_path(prefix, key, reference)) return fail("artifact_path");
        bool exists = false;
        if (!session.run([&](fs::FS &fs) { exists = fs.exists(final); return true; })) return fail("artifact_stat");
        if (exists) {
            if (!verify(final, publication)) return false;
            if (!session.mutate(part_path, [&](fs::FS &fs) { return fs.remove(part_path); })) return fail("artifact_remove_part");
        } else if (!session.mutate(part_path, final, [&](fs::FS &fs) { return fs.rename(part_path, final); })) return fail("artifact_publish");
        uint8_t ref[REFERENCE_BYTES];
        reference_bytes(key, publication, ref);
        if (!writer.open(session, part_path) || !writer.write(ref, sizeof(ref)) || !writer.finish(sizeof(ref))) return fail("reference_write");
        // Publish the reference last, without unlinking any old valid generation.
        if (!session.run([&](fs::FS &fs) { exists = fs.exists(reference); return true; })) return fail("reference_stat");
        if (exists) {
            SdStorage::Reader reader;
            if (!reader.open(session, reference) || reader.size() != sizeof(ref) ||
                !exact_read(reader, 0, input, sizeof(ref)) || memcmp(input, ref, sizeof(ref))) return fail("reference_conflict");
            return true;
        }
        return session.mutate(part_path, reference, [&](fs::FS &fs) { return fs.rename(part_path, reference); }) || fail("reference_publish");
    }
};

Producer::Producer(const SdStorage::Session &session, bool (*cancelled)(void *), void *context) {
    void *memory = aircannect::Memory::alloc_large(sizeof(Work), false);
    if (memory) work_ = new (memory) Work(session, cancelled, context);
}
Producer::~Producer() {
    if (work_) work_->~Work();
    aircannect::Memory::free(work_);
}
const char *Producer::error() const { return work_ ? work_->failure : "artifact_alloc"; }

bool Producer::begin(const char *prefix, const uint8_t key[32], const ReportEdf::Header &header,
                     const Selection *tracks, size_t count) {
    if (!work_) return false;
    auto &w = *work_;
    if (!w.active() || w.started) return w.fail("artifact_state");
    w.started = true;
    if (!std::isfinite(header.record_seconds) || header.record_seconds <= 0 ||
        header.record_seconds > 86400 || header.start_seconds < 0 ||
        header.start_seconds > 9007199254740991ll / 1000000) return w.fail("artifact_axis");
    if (!prefix || strlen(prefix) >= sizeof(w.prefix) || !tracks ||
        !count || count > MAX_TRACKS) return w.fail("artifact_tracks");
    snprintf(w.prefix, sizeof(w.prefix), "%s", prefix);
    memcpy(w.key, key, 32);
    w.metadata.track_count = count;
    w.records = header.records;
    for (size_t c = 0; c < count; ++c) {
        if (tracks[c].signal >= header.signal_count) return w.fail("artifact_signal");
        if (!tracks[c].label || !tracks[c].unit ||
            strlen(tracks[c].label) >= sizeof(w.metadata.tracks[c].label) ||
            strlen(tracks[c].unit) >= sizeof(w.metadata.tracks[c].unit)) return w.fail("artifact_label");
        const auto &signal = header.signals[tracks[c].signal];
        if (signal.annotation || signal.crc || !std::isfinite(tracks[c].scale) || tracks[c].scale <= 0) return w.fail("artifact_signal");
        auto &track = w.metadata.tracks[c];
        w.signals[c] = tracks[c].signal;
        w.offsets[c] = w.stride;
        track.id = tracks[c].id;
        const uint64_t samples = uint64_t(header.records) * signal.samples_per_record;
        if (samples > UINT32_MAX) return w.fail("artifact_samples");
        track.count = samples;
        track.start_ms = header.start_seconds * 1000;
        track.record_us = llround(header.record_seconds * 1000000);
        track.samples_per_record = signal.samples_per_record;
        track.digital_min = signal.digital_min; track.digital_max = signal.digital_max;
        track.physical_min = signal.physical_min * tracks[c].scale;
        track.physical_max = signal.physical_max * tracks[c].scale;
        snprintf(track.label, sizeof(track.label), "%s", tracks[c].label);
        snprintf(track.unit, sizeof(track.unit), "%s", tracks[c].unit);
        w.stride += signal.samples_per_record * 2 + (signal.samples_per_record + 7) / 8;
    }
    if (uint64_t(w.stride) * w.records > UINT32_MAX) return w.fail("spool_size");
    if (!layout(w.metadata) || !encode(w.metadata, w.output, sizeof(w.output)) ||
        !w.source.add(w.output, w.metadata.header_bytes)) return w.fail("artifact_layout");
    if (!named_path(prefix, "samples.part", w.spool_path) || !named_path(prefix, "tiles.part", w.part_path)) return w.fail("artifact_path");
    char directory[96];
    snprintf(directory, sizeof(directory), "%s", prefix);
    char *end = strrchr(directory, '/');
    if (!end) return w.fail("artifact_path");
    *end = 0;
    if (!w.session.mutate(w.spool_path, [&](fs::FS &fs) {
        return (fs.exists("/airbridge") || fs.mkdir("/airbridge")) &&
            (fs.exists("/airbridge/report") || fs.mkdir("/airbridge/report")) &&
            (fs.exists(directory) || fs.mkdir(directory));
    }) || !w.writer.open(w.session, w.spool_path)) return w.fail("spool_open");
    return true;
}

bool Producer::record(const ReportEdf::Header &header, const uint8_t *data, size_t bytes) {
    if (!work_ || !work_->active()) return false;
    auto &w = *work_;
    if (!w.started || w.finished || !data || w.written_records >= w.records ||
        bytes != header.record_bytes || !w.source.add(data, bytes)) return w.fail("artifact_record");
    for (size_t c = 0; c < w.metadata.track_count; ++c) {
        ReportEdf::RecordSamples samples;
        if (!ReportEdf::record_samples(header, data, bytes, w.signals[c], samples)) return w.fail("artifact_record");
        const size_t count = samples.signal->samples_per_record;
        for (size_t at = 0; at < count;) {
            const size_t take = std::min(count - at, sizeof(w.input) / 2);
            for (size_t n = 0; n < take; ++n) SdStorage::put_le16(w.input + n * 2, uint16_t(samples[at + n].raw));
            if (!w.writer.write(w.input, take * 2)) return w.fail("spool_write");
            at += take;
        }
        const size_t mask = (count + 7) / 8;
        if (mask > sizeof(w.input)) return w.fail("artifact_record_size");
        memset(w.input, 0, mask);
        for (size_t n = 0; n < count; ++n) if (samples[n].valid) w.input[n / 8] |= 1u << (n % 8);
        if (!w.writer.write(w.input, mask)) return w.fail("spool_write");
    }
    ++w.written_records;
    return true;
}

bool Producer::finish(Publication &out) {
    if (!work_ || !work_->active()) return false;
    auto &w = *work_;
    if (!w.started || w.finished) return w.fail("artifact_state");
    w.finished = true;
    if (w.written_records != w.records || !w.writer.finish(uint64_t(w.records) * w.stride) ||
        !w.source.finish(w.metadata.source)) return w.fail("spool_finish");
    SdStorage::Reader spool;
    if (!spool.open(w.session, w.spool_path) || !encode(w.metadata, w.output, sizeof(w.output))) return w.fail("artifact_open");
    if (!w.restore()) {
        if (!w.active()) return false;
        w.writer.close(); w.content.~Digest(); new (&w.content) Digest;
        w.made = 0;
        if (!w.writer.open(w.session, w.part_path) || !w.append(w.output, w.metadata.header_bytes) ||
            !w.checkpoint()) return w.fail("artifact_open");
    }
    for (size_t c = 0; c < w.metadata.track_count; ++c) {
        const auto &track = w.metadata.tracks[c];
        for (size_t l = 0; l < track.level_count; ++l) {
            const auto &level = track.levels[l];
            if (w.made >= level.offset + level.bytes) continue;
            if (!(l ? w.reduce(c, l) : w.base(spool, c)) || !w.checkpoint()) return false;
        }
    }
    spool.close();
    if (w.made != w.metadata.bytes || !w.writer.finish(w.made) || !w.content.finish(out.digest)) return w.fail("artifact_finish");
    out.bytes = w.metadata.bytes;
    memcpy(out.source, w.metadata.source, 32);
    w.published = w.verify(w.part_path, out) && w.publish(out);
    return w.published;
}

}  // namespace ReportArtifact
#endif
