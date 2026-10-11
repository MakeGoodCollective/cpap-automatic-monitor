#pragma once

#include <stddef.h>
#include <stdint.h>

namespace aircannect {

struct MemoryStatus {
    bool psram_available = false;
    size_t heap_total = 0;
    size_t heap_free = 0;
    size_t heap_max_alloc = 0;
    size_t psram_total = 0;
    size_t psram_free = 0;
    size_t psram_max_alloc = 0;
};

namespace Memory {

MemoryStatus status();
bool psram_available();

// One optional cache owner, called after PSRAM allocation failure and before
// internal fallback. Release one item and return true to retry. Must not block
// or allocate: this runs in the allocating task, possibly under another lock.
using Reclaimer = bool (*)();
void set_reclaimer(Reclaimer reclaimer);

void *alloc_large(size_t size, bool allow_internal_fallback = true);
void *realloc_large(void *ptr,
                    size_t size,
                    bool allow_internal_fallback = true);
void *calloc_large(size_t count,
                   size_t size,
                   bool allow_internal_fallback = true);
void free(void *ptr);

}  // namespace Memory
}  // namespace aircannect
