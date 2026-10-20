#pragma once

#include <stddef.h>
#include <stdint.h>

namespace ResmedFirmwareImage {

constexpr uint32_t FLASH_BASE = 0x08000000;
constexpr uint32_t FULL_SIZE = 0x100000;
constexpr size_t RECOMMENDED_SCRATCH = 4096;

enum class Platform : uint8_t { Unknown, SX577_0200, SX585_0200 };
enum class Block : uint8_t { Unknown, FULL, BLX, CCX, CDX, CMX };
enum class Status : uint8_t {
    Ok, InvalidArgument, InvalidLength, UnknownTarget,
    UnsupportedPlatform, PlatformMismatch, ReadError,
};
enum class IntegrityPatch : uint8_t {
    NotInspected, NotDetected, Unsafe, Safe, Partial,
};
enum class DumpExtension : uint8_t { Unknown, NotDetected, Recognized };

struct Range {
    Block block;
    uint32_t image_offset;
    uint32_t flash_start;
    uint32_t size;
};

struct Profile {
    Platform platform;
    const char *bid;
    Range ranges[5];  // FULL, BLX, CCX, CDX, CMX; half-open ranges.
};

struct Reader {
    // Exact offset read: false includes short/partial reads. Caller owns the
    // immutable source and its SD/partition lease for the entire inspection.
    void *context = nullptr;
    uint64_t size = 0;
    bool (*read_at)(void *, uint64_t, uint8_t *, size_t) = nullptr;
};

struct Options {
    // Auto recognizes only exact FULL/BLX lengths with a supported BLX BID.
    // Other standalone blocks require both explicit target and platform.
    // target describes the source shape, not a slice to take from a FULL source.
    // An explicit platform fixes the expected CRC layout; embedded BID warnings
    // remain facts, not a policy rejection. BLX signatures use the embedded BID.
    Block target = Block::Unknown;
    Platform platform = Platform::Unknown;
};

struct BlockCheck {
    Range range = {};
    uint32_t source_offset = 0;  // In this Reader, not the canonical FULL.
    bool complete = false;
    uint16_t crc = 0xFFFF;  // CRC16-CCITT-FALSE of entire block, including tail.
    bool crc_ok = false;   // Complete and zero residue.
};

struct Result {
    Status status = Status::InvalidArgument;
    Block target = Block::Unknown;
    Platform platform = Platform::Unknown;
    Platform bid_platform = Platform::Unknown;
    uint64_t size = 0;
    char bid[32] = {};
    bool bid_present = false;
    bool bid_supported = false;
    bool bid_matches = false;  // Embedded BID agrees with selected profile.
    BlockCheck blocks[3];      // BLX, CCX, CDX as applicable, never CMX CRC.
    size_t block_count = 0;
    IntegrityPatch integrity_patch = IntegrityPatch::NotInspected;
    DumpExtension dump_extension = DumpExtension::Unknown;

    bool crc_ok() const;
};

const Profile *profile(Platform platform);
const Profile *profile_for_bid(const char *bid);
const Range *range(const Profile &profile, Block block);
const char *block_name(Block block);
Block parse_block(const char *name);

// Pure, read-only and allocation-free. Returns true only for Status::Ok:
// inspection completed, NOT permission to flash. CRC failure and unsafe patch
// remain facts for caller policy. No hashes, UART, filesystem or task ownership.
// CRC streams each constituent block once. Scratch is borrowed, capped at 4KiB;
// nullptr/0 uses a bounded 256-byte stack fallback. Other null/size pairs fail.
// On failure, completed blocks remain factual; incomplete blocks are not valid.
// Dump recognition is independent of integrity patches and is not a live probe.
bool inspect(const Reader &reader, Result &out, const Options &options = {},
             uint8_t *scratch = nullptr, size_t scratch_size = 0);

}  // namespace ResmedFirmwareImage
