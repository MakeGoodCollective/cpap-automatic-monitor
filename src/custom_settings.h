#pragma once

#include <stddef.h>
#include <stdint.h>

namespace CustomSettings {

enum kind_t : uint8_t {
    KIND_NUMERIC,
    KIND_ENUM,
};

struct option_view_t {
    uint8_t value;
    const char *label;
};

struct entry_view_t {
    kind_t kind;
    uint8_t category;
    uint32_t mop_mask;
    const char *name;
    const char *label;
    const char *units;
    const char *groups;  // group_count consecutive NUL-terminated names
    uint8_t group_count;
    uint8_t flags;
    uint8_t width;
    int16_t scale;
    int16_t step;
    uint8_t decimals;
    uint32_t minimum;
    uint32_t maximum;
    uint8_t option_count;
};

struct Metadata;

class MetadataLease {
public:
    MetadataLease() = default;
    ~MetadataLease();
    MetadataLease(const MetadataLease &) = delete;
    MetadataLease &operator=(const MetadataLease &) = delete;
    bool acquire();  // Worker only; waits for and promotes shared discovery.
    void reset();
    uint32_t generation() const;
    uint16_t count() const;
    int find(const char *name) const;
    bool entry(uint16_t index, entry_view_t &view) const;
    bool option(uint16_t index, uint8_t option, option_view_t &view) const;
    bool read_raw(uint16_t index, uint32_t &value) const;  // Worker only.

private:
    const Metadata *metadata_ = nullptr;
};

void init();
void poll();  // Main-loop discovery; background work requires confirmed standby.
bool ensure_loaded();  // Worker only; joins discovery within the job deadline.
bool contains(const char *name);
void reclaim();  // Nonblocking cleanup of unreferenced retired metadata.
bool write_raw(const char *name, uint32_t value);

// Invalidate metadata after a device reboot, firmware replacement, or LAN change.
void invalidate(const char *reason);
uint32_t generation();

}  // namespace CustomSettings
