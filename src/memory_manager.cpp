#include "memory_manager.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <esp_heap_caps.h>
#endif

#include <stdlib.h>
#include <string.h>

namespace aircannect {
namespace Memory {
namespace {
Reclaimer cache_reclaimer = nullptr;

#ifdef ARDUINO
bool detect_psram() {
    return psramFound() && ESP.getPsramSize() > 0;
}

bool reclaim() {
    const auto callback = __atomic_load_n(&cache_reclaimer, __ATOMIC_ACQUIRE);
    return callback && callback();
}
#endif

}  // namespace

void set_reclaimer(Reclaimer reclaimer) {
    __atomic_store_n(&cache_reclaimer, reclaimer, __ATOMIC_RELEASE);
}

MemoryStatus status() {
#ifdef ARDUINO
    MemoryStatus out;
    out.heap_total = ESP.getHeapSize();
    out.heap_free = ESP.getFreeHeap();
    out.heap_max_alloc =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                         MALLOC_CAP_8BIT);
    out.psram_available = detect_psram();
    if (out.psram_available) {
        out.psram_total = ESP.getPsramSize();
        out.psram_free = ESP.getFreePsram();
        out.psram_max_alloc =
            heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM |
                                             MALLOC_CAP_8BIT);
    }
    return out;
#else
    return {};
#endif
}

bool psram_available() {
#ifdef ARDUINO
    return detect_psram();
#else
    return false;
#endif
}

void *alloc_large(size_t size, bool allow_internal_fallback) {
#ifdef ARDUINO
    if (size == 0) return nullptr;

    if (detect_psram()) {
        do {
            void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM |
                                               MALLOC_CAP_8BIT);
            if (ptr) return ptr;
        } while (reclaim());
    }

    if (!allow_internal_fallback) return nullptr;
    return heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
    (void)allow_internal_fallback;
    return size == 0 ? nullptr : malloc(size);
#endif
}

void *realloc_large(void *ptr,
                    size_t size,
                    bool allow_internal_fallback) {
#ifdef ARDUINO
    if (!ptr) return alloc_large(size, allow_internal_fallback);
    if (size == 0) {
        ::free(ptr);
        return nullptr;
    }

    if (detect_psram()) {
        do {
            void *next = heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM |
                                                       MALLOC_CAP_8BIT);
            if (next) return next;
        } while (reclaim());
    }

    if (!allow_internal_fallback) return nullptr;
    return heap_caps_realloc(ptr,
                             size,
                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
    (void)allow_internal_fallback;
    return realloc(ptr, size);
#endif
}

void *calloc_large(size_t count,
                   size_t size,
                   bool allow_internal_fallback) {
    if (count == 0 || size == 0) return nullptr;
    if (count > SIZE_MAX / size) return nullptr;

#ifdef ARDUINO
    const size_t bytes = count * size;
    void *ptr = alloc_large(bytes, allow_internal_fallback);
    if (ptr) memset(ptr, 0, bytes);
    return ptr;
#else
    (void)allow_internal_fallback;
    return calloc(count, size);
#endif
}

void free(void *ptr) {
    if (ptr) ::free(ptr);
}

}  // namespace Memory
}  // namespace aircannect
