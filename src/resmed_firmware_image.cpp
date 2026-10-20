#include "resmed_firmware_image.h"
#include "crc.h"

#include <string.h>

namespace ResmedFirmwareImage {
namespace {

// Airbreak resmed_flash.py BLOCK_MAPS and lib/as10_firmware.py PLATFORMS.
constexpr Profile PROFILES[] = {
    {Platform::SX577_0200, "SX577-0200", {
        {Block::FULL, 0, FLASH_BASE, FULL_SIZE},
        {Block::BLX, 0, FLASH_BASE, 0x04000},
        {Block::CCX, 0x04000, FLASH_BASE + 0x04000, 0x3C000},
        {Block::CDX, 0x40000, FLASH_BASE + 0x40000, 0xC0000},
        {Block::CMX, 0x04000, FLASH_BASE + 0x04000, 0xFC000},
    }},
    {Platform::SX585_0200, "SX585-0200", {
        {Block::FULL, 0, FLASH_BASE, FULL_SIZE},
        {Block::BLX, 0, FLASH_BASE, 0x04000},
        {Block::CCX, 0x04000, FLASH_BASE + 0x04000, 0x1C000},
        {Block::CDX, 0x20000, FLASH_BASE + 0x20000, 0xE0000},
        {Block::CMX, 0x04000, FLASH_BASE + 0x04000, 0xFC000},
    }},
};

constexpr uint32_t BID_OFFSET = 0x3F80;
constexpr Block CONSTITUENTS[] = {Block::BLX, Block::CCX, Block::CDX};
constexpr uint8_t UNSAFE_PATCH[] = {0xC0, 0x46};
constexpr uint8_t SAFE_BLX[] = {0x01, 0x20, 0xC0, 0x46};
constexpr uint8_t SAFE_APP[] = {0x00, 0x20, 0xC0, 0x46};

// Exact 214-byte blx_dump.c build, SX577-0200; no digest/signature heuristic.
// Reference: Airbreak 586f6902f589, arm-none-eabi-gcc 13.2.1, -Os.
// The SX585 build differs only at six branch halfwords listed below. These
// builds' payload_layout TSVs place them at 0x3DE0 / 0x3E78. Other compiler or
// allocator versions remain Unknown, even if they implement the same protocol.
constexpr uint8_t DUMP_BODY[] = {
    0x30,0xB5,0x15,0x46,0xAD,0xF5,0x41,0x7D,0x41,0xB1,0x0A,0x78,0x4F,0x2A,0x05,0xD1,
    0x4A,0x68,0x07,0x2A,0x02,0xD1,0x0A,0x7A,0x44,0x2A,0x06,0xD0,0x2A,0x46,0x0D,0xF5,
    0x41,0x7D,0xBD,0xE8,0x30,0x40,0xFC,0xF7,0x5A,0xBB,0x8B,0x7A,0x4C,0x7A,0x8A,0x7B,
    0x44,0xEA,0x03,0x24,0xCB,0x7A,0x44,0xEA,0x03,0x44,0x0B,0x7B,0x44,0xEA,0x03,0x64,
    0x4B,0x7B,0x8D,0xF8,0x05,0x40,0x43,0xEA,0x02,0x23,0x45,0x22,0x8D,0xF8,0x04,0x20,
    0x22,0x0A,0x8D,0xF8,0x06,0x20,0x22,0x0C,0x8D,0xF8,0x07,0x20,0x22,0x0E,0x8D,0xF8,
    0x08,0x20,0x1A,0x0A,0x8D,0xF8,0x0A,0x20,0x5A,0x1E,0xEF,0x2A,0x8D,0xF8,0x09,0x30,
    0x2B,0xD8,0xB4,0xF5,0x80,0x1F,0x28,0xD2,0xC4,0xF5,0x80,0x12,0x93,0x42,0x24,0xD8,
    0x64,0x22,0x8D,0xF8,0x04,0x20,0x0D,0xF1,0x0A,0x01,0x00,0x22,0x04,0xF1,0x00,0x64,
    0xA0,0x5C,0x01,0xF8,0x01,0x0F,0x01,0x32,0x93,0x42,0xF9,0xD8,0xDA,0x1D,0x92,0xB2,
    0x3F,0xAB,0x01,0xA9,0x4F,0x20,0xFD,0xF7,0xDD,0xFC,0x30,0xB1,0x28,0x46,0xFD,0xF7,
    0x1C,0xFC,0xBF,0x9A,0x3F,0xA9,0xFD,0xF7,0x7C,0xFB,0xFD,0xF7,0xCD,0xF9,0x04,0x30,
    0xFD,0xF7,0x29,0xF9,0x0D,0xF5,0x41,0x7D,0x30,0xBD,0x00,0x23,0x8D,0xF8,0x09,0x30,
    0x8D,0xF8,0x0A,0x30,0xE2,0xE7,
};

struct BlxFacts {
    uint8_t unsafe[2] = {};
    uint8_t safe[3][4] = {};
    uint8_t hook[4] = {};
    bool dump_matches = true;
};

// Copy only the overlap, including signatures split across tiny scratch reads.
void capture(uint32_t offset, const uint8_t *data, size_t count,
             uint32_t field_offset, uint8_t *field, size_t field_size) {
    const uint32_t begin = offset > field_offset ? offset : field_offset;
    const uint32_t data_end = offset + count;
    const uint32_t field_end = field_offset + field_size;
    const uint32_t end = data_end < field_end ? data_end : field_end;
    if (begin < end) {
        memcpy(field + begin - field_offset, data + begin - offset, end - begin);
    }
}

void collect_blx(BlxFacts &facts, Platform platform, uint32_t offset,
                 const uint8_t *data, size_t count) {
    const bool lumis = platform == Platform::SX585_0200;
    const uint32_t shift = lumis ? 0x60 : 0;
    capture(offset, data, count, 0xF0, facts.unsafe, sizeof(facts.unsafe));
    const uint32_t safe_offsets[] = {0x310E + shift, 0x313E + shift, 0x3130 + shift};
    for (size_t i = 0; i < 3; ++i) {
        capture(offset, data, count, safe_offsets[i], facts.safe[i], 4);
    }
    capture(offset, data, count, 0x300E + shift, facts.hook, sizeof(facts.hook));

    const uint32_t payload = lumis ? 0x3E78 : 0x3DE0;
    const uint32_t begin = offset > payload ? offset : payload;
    const uint32_t end = offset + count < payload + sizeof(DUMP_BODY)
        ? offset + count : payload + sizeof(DUMP_BODY);
    for (uint32_t pos = begin; pos < end; ++pos) {
        const size_t index = pos - payload;
        uint8_t expected = DUMP_BODY[index];
        if (lumis) {
            switch (index) {
                case 0x28: expected = 0x0E; break;
                case 0xA8: expected = 0xC1; break;
                case 0xB0: expected = 0x00; break;
                case 0xB8: expected = 0x60; break;
                case 0xBC: expected = 0xB1; break;
                case 0xC2: expected = 0x0D; break;
            }
        }
        if (data[pos - offset] != expected) facts.dump_matches = false;
    }
}

void finish_blx(const BlxFacts &facts, Platform platform, Result &out) {
    const unsigned safe_count =
        (memcmp(facts.safe[0], SAFE_BLX, 4) == 0) +
        (memcmp(facts.safe[1], SAFE_APP, 4) == 0) +
        (memcmp(facts.safe[2], SAFE_APP, 4) == 0);
    if (memcmp(facts.unsafe, UNSAFE_PATCH, 2) == 0) {
        out.integrity_patch = IntegrityPatch::Unsafe;
    } else if (safe_count == 3) {
        out.integrity_patch = IntegrityPatch::Safe;
    } else {
        out.integrity_patch = safe_count ? IntegrityPatch::Partial : IntegrityPatch::NotDetected;
    }

    const uint8_t stock577[] = {0xFD, 0xF7, 0x56, 0xFA};
    const uint8_t stock585[] = {0xFD, 0xF7, 0x26, 0xFA};
    const uint8_t dump577[] = {0x00, 0xF0, 0xE7, 0xFE};
    const uint8_t dump585[] = {0x00, 0xF0, 0x03, 0xFF};
    const bool lumis = platform == Platform::SX585_0200;
    if (memcmp(facts.hook, lumis ? stock585 : stock577, 4) == 0) {
        out.dump_extension = DumpExtension::NotDetected;
    } else if (memcmp(facts.hook, lumis ? dump585 : dump577, 4) == 0 &&
               facts.dump_matches) {
        out.dump_extension = DumpExtension::Recognized;
    }
}

bool inspect_blocks(const Reader &reader, Result &out, const Profile &selected,
                    const Range &input, uint8_t *scratch, size_t scratch_size) {
    BlxFacts facts;
    for (Block block : CONSTITUENTS) {
        const Range &part = *range(selected, block);
        if (part.image_offset < input.image_offset ||
            part.image_offset + part.size > input.image_offset + input.size) continue;

        BlockCheck &check = out.blocks[out.block_count++];
        check.range = part;
        check.source_offset = part.image_offset - input.image_offset;
        for (uint32_t pos = 0; pos < part.size;) {
            const size_t remaining = part.size - pos;
            const size_t count = remaining < scratch_size ? remaining : scratch_size;
            if (!reader.read_at(reader.context, check.source_offset + pos, scratch, count)) {
                out.status = Status::ReadError;
                return false;
            }
            check.crc = crc16_ccitt(scratch, count, check.crc);
            if (block == Block::BLX && out.bid_supported) {
                collect_blx(facts, out.bid_platform, pos, scratch, count);
            }
            pos += count;
        }
        check.complete = true;
        check.crc_ok = check.crc == 0;
        if (block == Block::BLX && out.bid_supported) {
            finish_blx(facts, out.bid_platform, out);
        }
    }
    out.status = Status::Ok;
    return true;
}

}  // namespace

const Profile *profile(Platform platform) {
    for (const Profile &entry : PROFILES) {
        if (entry.platform == platform) return &entry;
    }
    return nullptr;
}

const Profile *profile_for_bid(const char *bid) {
    if (!bid) return nullptr;
    for (const Profile &entry : PROFILES) {
        if (strcmp(entry.bid, bid) == 0) return &entry;
    }
    return nullptr;
}

const Range *range(const Profile &selected, Block block) {
    for (const Range &entry : selected.ranges) {
        if (entry.block == block) return &entry;
    }
    return nullptr;
}

const char *block_name(Block block) {
    switch (block) {
        case Block::FULL: return "FULL";
        case Block::BLX: return "BLX";
        case Block::CCX: return "CCX";
        case Block::CDX: return "CDX";
        case Block::CMX: return "CMX";
        default: return "UNKNOWN";
    }
}

Block parse_block(const char *name) {
    if (name) {
        for (const Range &entry : PROFILES[0].ranges) {
            if (strcmp(name, block_name(entry.block)) == 0) return entry.block;
        }
    }
    return Block::Unknown;
}

bool Result::crc_ok() const {
    if (status != Status::Ok || block_count == 0) return false;
    for (size_t i = 0; i < block_count; ++i) {
        if (!blocks[i].complete || !blocks[i].crc_ok) return false;
    }
    return true;
}

bool inspect(const Reader &reader, Result &out, const Options &options,
             uint8_t *scratch, size_t scratch_size) {
    out = {};
    out.size = reader.size;
    out.target = options.target;
    out.platform = options.platform;
    const Profile *selected = profile(options.platform);
    if (!reader.read_at || ((scratch == nullptr) != (scratch_size == 0)) ||
        (options.platform != Platform::Unknown && !selected) ||
        (options.target != Block::Unknown && !range(PROFILES[0], options.target))) return false;

    if (reader.size == 0 || reader.size > FULL_SIZE) {
        out.status = Status::InvalidLength;
        return false;
    }

    Block target = options.target;
    if (target == Block::Unknown) {
        if (reader.size == FULL_SIZE) target = Block::FULL;
        else if (reader.size == 0x4000) target = Block::BLX;
        else {
            out.status = Status::UnknownTarget;
            return false;
        }
    }
    const bool has_blx = target == Block::FULL || target == Block::BLX;
    // Common FULL/BLX lengths can be rejected before reading even an unknown BID.
    if ((has_blx || selected) && reader.size != range(selected ? *selected : PROFILES[0], target)->size) {
        out.status = Status::InvalidLength;
        return false;
    }
    if (has_blx) {
        uint8_t bid[sizeof(out.bid)];
        if (!reader.read_at(reader.context, BID_OFFSET, bid, sizeof(bid))) {
            out.status = Status::ReadError;
            return false;
        }
        bool terminated = false;
        for (size_t i = 0; i < sizeof(bid); ++i) {
            if (bid[i] == 0) { terminated = true; break; }
            if (bid[i] < 0x20 || bid[i] > 0x7E || i == sizeof(bid) - 1) break;
            out.bid[i] = static_cast<char>(bid[i]);
        }
        out.bid_present = out.bid[0] != 0;
        const Profile *embedded = terminated ? profile_for_bid(out.bid) : nullptr;
        out.bid_supported = embedded != nullptr;
        if (!embedded && (!selected || options.target == Block::Unknown)) {
            out.status = Status::UnsupportedPlatform;
            return false;
        }
        if (!selected) selected = embedded;
        if (embedded) out.bid_platform = embedded->platform;
        out.bid_matches = selected == embedded;
    }
    if (!selected) {
        out.status = Status::UnsupportedPlatform;
        return false;
    }

    out.target = target;
    out.platform = selected->platform;
    const Range &input = *range(*selected, target);
    uint8_t fallback[256];
    if (!scratch) {
        scratch = fallback;
        scratch_size = sizeof(fallback);
    }
    if (scratch_size > RECOMMENDED_SCRATCH) scratch_size = RECOMMENDED_SCRATCH;
    return inspect_blocks(reader, out, *selected, input, scratch, scratch_size);
}

}  // namespace ResmedFirmwareImage
