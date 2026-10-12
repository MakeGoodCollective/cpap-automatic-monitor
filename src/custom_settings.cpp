#include "custom_settings.h"

#include "custom_settings_protocol.h"
#include "debug_log.h"
#include "qframe.h"
#include "uart_arbiter.h"
#include "clinical_jobs.h"
#include "memory_manager.h"
#include "hex_util.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

namespace CustomSettings {
namespace {

constexpr uint8_t FLAG_EDITABLE = 0x04;
constexpr uint8_t MAX_ENUM_OPTIONS = 32;
constexpr uint32_t RETRY_DELAY_MS = 5000;
constexpr uint16_t INVALID_OFFSET = 0xffff;

enum cache_state_t : uint8_t {
    CACHE_UNKNOWN,
    CACHE_READY,
    CACHE_UNSUPPORTED,
    CACHE_INCOMPATIBLE,
    CACHE_FAILED,
};

enum query_result_t : uint8_t {
    QUERY_OK,
    QUERY_ERROR,
    QUERY_TIMEOUT,
};

struct cached_option_t {
    uint16_t label_offset;
    uint8_t value;
};

struct cached_entry_t {
    uint32_t mop_mask;
    uint32_t minimum;
    uint32_t maximum;
    uint16_t label_offset;
    uint16_t units_offset;
    uint16_t groups_offset;
    uint16_t option_start;
    int16_t scale;
    int16_t step;
    char name[4];
    uint8_t category;
    uint8_t flags;
    uint8_t width;
    uint8_t decimals;
    uint8_t option_count;
    uint8_t group_count;
    kind_t kind;
};

}  // namespace

struct Metadata {
    cached_entry_t *entries = nullptr;
    cached_option_t *options = nullptr;
    char *text_pool = nullptr;
    uint8_t entry_count = 0;
    uint16_t option_count = 0, option_capacity = 0;
    uint16_t text_size = 0, text_capacity = 0;
    uint32_t epoch = 0;
    mutable uint32_t readers = 0;
};

namespace {
SemaphoreHandle_t cache_mutex = nullptr;
cache_state_t cache_state = CACHE_UNKNOWN;
Metadata metadata_slots[2];
Metadata *current = &metadata_slots[0];
uint32_t retry_at_ms = 0;
uint32_t cached_language = 0;
bool language_known = false;
uint32_t invalidation_generation = 1;
uint32_t loaded_generation = 0;

void clear_allocations(Metadata &data) {
    aircannect::Memory::free(data.entries);
    aircannect::Memory::free(data.options);
    aircannect::Memory::free(data.text_pool);
    data.entries = nullptr;
    data.options = nullptr;
    data.text_pool = nullptr;
    data.entry_count = 0;
    data.option_count = 0;
    data.option_capacity = 0;
    data.text_size = 0;
    data.text_capacity = 0;
}

bool reset_cache_locked(cache_state_t state) {
    if (__atomic_load_n(&current->readers, __ATOMIC_ACQUIRE)) {
        Metadata *other = current == &metadata_slots[0] ? &metadata_slots[1] : &metadata_slots[0];
        if (__atomic_load_n(&other->readers, __ATOMIC_ACQUIRE)) return false;
        current = other;
    }
    clear_allocations(*current);
    cache_state = state;
    retry_at_ms = state == CACHE_FAILED ? millis() + RETRY_DELAY_MS : 0;
    return true;
}

bool reserve_text_locked(uint16_t required) {
    if (required <= current->text_capacity) return true;
    uint32_t next = current->text_capacity ? current->text_capacity : 256;
    while (next < required) next *= 2;
    if (next > UINT16_MAX) next = UINT16_MAX;
    if (next < required) return false;

    char *replacement = static_cast<char *>(aircannect::Memory::realloc_large(current->text_pool, next));
    if (!replacement) return false;
    current->text_pool = replacement;
    current->text_capacity = (uint16_t)next;
    return true;
}

uint16_t add_text_locked(const char *text, size_t length, bool keep_empty = false) {
    if (!text || (length == 0 && !keep_empty)) return 0;
    if (length >= UINT16_MAX || current->text_size > UINT16_MAX - length - 1)
        return INVALID_OFFSET;
    uint16_t required = (uint16_t)(current->text_size + length + 1);
    if (!reserve_text_locked(required)) return INVALID_OFFSET;
    uint16_t offset = current->text_size;
    memcpy(current->text_pool + current->text_size, text, length);
    current->text_size += (uint16_t)length;
    current->text_pool[current->text_size++] = '\0';
    return offset;
}

bool reserve_options_locked(uint16_t required) {
    if (required <= current->option_capacity) return true;
    uint32_t next = current->option_capacity ? current->option_capacity * 2u : 16u;
    while (next < required) next *= 2;
    if (next > UINT16_MAX) return false;

    cached_option_t *replacement = static_cast<cached_option_t *>(
        aircannect::Memory::realloc_large(current->options, next * sizeof(cached_option_t)));
    if (!replacement) return false;
    current->options = replacement;
    current->option_capacity = (uint16_t)next;
    return true;
}

query_result_t query_value(const char *command, char *response,
                           uint16_t response_size, const char *&value) {
    if (!command || !response || response_size < 2) return QUERY_TIMEOUT;
    uint16_t remaining = ClinicalJobs::remaining_ms();
    if (!remaining || loaded_generation != generation()) return QUERY_TIMEOUT;
    response[0] = '\0';
    uint16_t response_len = response_size;
    bool ok = Arbiter::send_cmd(command, CMD_SRC_TCP, CMD_PRIO_NORMAL,
                                response, &response_len, 0, remaining);
    value = qframe_response_value(response);
    if (ok && value) return QUERY_OK;
    if (command[0] == 'G')
        Log::logf(CAT_CONFIG, LOG_DEBUG, "AirSense: %s: %s\n", command,
                  value ? value : "no response");
    return value ? QUERY_ERROR : QUERY_TIMEOUT;
}

bool error_is(const char *value, const char *code) {
    return value && code && strcmp(value, code) == 0;
}

bool read_raw_locked(const char *name, uint32_t &value) {
    uint16_t remaining = ClinicalJobs::remaining_ms();
    if (!remaining || loaded_generation != generation()) return false;
    return Arbiter::read_var_hex(name, CMD_SRC_TCP, CMD_PRIO_NORMAL, value,
                                 0, nullptr, remaining) ==
           Arbiter::VarResult::Ok;
}

bool add_enum_option_locked(cached_entry_t &entry, uint8_t value,
                            const char *label, size_t label_len) {
    if (entry.option_count >= MAX_ENUM_OPTIONS ||
        !reserve_options_locked((uint16_t)(current->option_count + 1))) {
        return false;
    }
    uint16_t label_offset = add_text_locked(label, label_len);
    if (label_offset == INVALID_OFFSET) return false;
    current->options[current->option_count++] = {label_offset, value};
    entry.option_count++;
    return true;
}

query_result_t query_enum_option(const char *name, uint8_t index,
                                 char *response, uint16_t response_size,
                                 CustomSettingsProtocol::enum_option_t &option,
                                 const char *&error_value) {
    char command[24];
    snprintf(command, sizeof(command), "G C #%s %02X", name, index);
    const char *value = nullptr;
    query_result_t result = query_value(command, response, response_size, value);
    if (result == QUERY_OK) {
        if (!CustomSettingsProtocol::parse_enum_option(value, option))
            return QUERY_TIMEOUT;
        return QUERY_OK;
    }
    error_value = value;
    return result;
}

bool discover_enum_options_locked(cached_entry_t &entry,
                                   char (&response)[QFRAME_MAX_PAYLOAD + 1]) {
    entry.option_start = current->option_count;
    entry.option_count = 0;

    uint8_t total = 0;
    bool total_known = false;
    uint32_t current = 0;
    if (read_raw_locked(entry.name, current) && current <= 0xff) {
        CustomSettingsProtocol::enum_option_t option = {};
        const char *error_value = nullptr;
        query_result_t result = query_enum_option(
            entry.name, (uint8_t)current, response, sizeof(response), option,
            error_value);
        if (result == QUERY_OK) {
            total = option.count;
            total_known = true;
        } else if (result != QUERY_ERROR || !error_is(error_value, "6033")) {
            return false;
        }
    }

    if (!total_known) {
        for (uint8_t index = 0; index < MAX_ENUM_OPTIONS; index++) {
            CustomSettingsProtocol::enum_option_t option = {};
            const char *error_value = nullptr;
            query_result_t result = query_enum_option(
                entry.name, index, response, sizeof(response), option,
                error_value);
            if (result == QUERY_OK) {
                total = option.count;
                total_known = true;
                break;
            }
            if (result != QUERY_ERROR || !error_is(error_value, "6033"))
                return false;
        }
    }

    if (!total_known) return true;
    if (total > MAX_ENUM_OPTIONS) return false;

    for (uint8_t index = 0; index < total; index++) {
        CustomSettingsProtocol::enum_option_t option = {};
        const char *error_value = nullptr;
        query_result_t result = query_enum_option(
            entry.name, index, response, sizeof(response), option, error_value);
        if (result == QUERY_ERROR && error_is(error_value, "6033")) continue;
        if (result != QUERY_OK || option.count != total ||
            !add_enum_option_locked(entry, index, option.label, option.label_len)) {
            return false;
        }
    }
    return true;
}

bool discover_locked() {
    char command[20] = "G C &CSG";
    auto fail = [&](const char *stage) {
        Log::logf(CAT_CONFIG, LOG_WARN, "AirSense: Discovery failed: %s (%s)\n",
                  command, stage);
        return false;
    };
    char response[QFRAME_MAX_PAYLOAD + 1];
    const char *value = nullptr;
    query_result_t result = query_value("G C &CSG", response, sizeof(response), value);
    if (result == QUERY_ERROR && error_is(value, "6009")) {
        reset_cache_locked(CACHE_UNSUPPORTED);
        Log::logf(CAT_CONFIG, LOG_DEBUG,
                  "AirSense: Airbreak custom registry unavailable\n");
        return true;
    }
    if (result != QUERY_OK) return fail("header query");

    CustomSettingsProtocol::header_t header = {};
    if (!CustomSettingsProtocol::parse_header(value, header)) return fail("header syntax");
    if (!CustomSettingsProtocol::supported_version(header.version)) {
        reset_cache_locked(CACHE_INCOMPATIBLE);
        Log::logf(CAT_CONFIG, LOG_WARN,
                  "AirSense: Unsupported custom registry version %02X\n",
                  header.version);
        return true;
    }

    if (header.count == 0) {
        reset_cache_locked(CACHE_READY);
        return true;
    }

    current->entries = static_cast<cached_entry_t *>(
        aircannect::Memory::calloc_large(header.count, sizeof(cached_entry_t)));
    if (!current->entries) return fail("entry allocation");
    current->entry_count = header.count;
    uint32_t initial_text = 1u + (uint32_t)header.count * 64u;
    if (initial_text > UINT16_MAX) initial_text = UINT16_MAX;
    if (!reserve_text_locked((uint16_t)initial_text)) return fail("text allocation");
    current->text_pool[0] = '\0';
    current->text_size = 1;

    for (uint16_t index = 0; index < current->entry_count; index++) {
        snprintf(command, sizeof(command), "G C &CSG %02X", index);
        if (query_value(command, response, sizeof(response), value) != QUERY_OK)
            return fail("entry query");

        CustomSettingsProtocol::entry_t record = {};
        if (!CustomSettingsProtocol::parse_entry(value, header.version, record))
            return fail("entry syntax");
        for (uint16_t previous = 0; previous < index; previous++) {
            if (strcmp(current->entries[previous].name, record.name) == 0)
                return fail("duplicate variable");
        }

        cached_entry_t &entry = current->entries[index];
        memcpy(entry.name, record.name, sizeof(entry.name));
        entry.kind = record.kind == CustomSettingsProtocol::ENTRY_NUMERIC
            ? KIND_NUMERIC : KIND_ENUM;
        entry.category = record.category;
        entry.mop_mask = record.mop_mask;
        entry.scale = record.scale;
        entry.step = record.step;
        entry.decimals = record.decimals;
        entry.label_offset = add_text_locked(record.label, record.label_len);
        entry.units_offset = add_text_locked(record.units, record.units_len);
        if (entry.label_offset == INVALID_OFFSET ||
            entry.units_offset == INVALID_OFFSET) {
            return fail("label allocation");
        }
        entry.group_count = record.group_count;
        if (record.group_count) {
            entry.groups_offset = current->text_size;
            const char *cursor = record.groups;
            for (uint8_t group = 0; group < record.group_count; group++) {
                const char *text;
                size_t length;
                if (!CustomSettingsProtocol::parse_text(cursor, text, length) ||
                    add_text_locked(text, length, true) == INVALID_OFFSET)
                    return fail("group text");
            }
        }

        snprintf(command, sizeof(command), "G C #%s", entry.name);
        if (query_value(command, response, sizeof(response), value) != QUERY_OK)
            return fail("capability query");

        if (entry.kind == KIND_NUMERIC) {
            CustomSettingsProtocol::numeric_caps_t caps = {};
            if (!CustomSettingsProtocol::parse_numeric_caps(value, caps))
                return fail("numeric capabilities");
            entry.flags = caps.flags;
            entry.width = caps.width;
            entry.minimum = caps.minimum;
            entry.maximum = caps.maximum;
        } else {
            if (!CustomSettingsProtocol::parse_enum_flags(value, entry.flags))
                return fail("enum capabilities");
            entry.width = 4;
            if (!discover_enum_options_locked(entry, response)) return fail("enum options");
        }
    }

    if (current->text_size < current->text_capacity) {
        char *text = static_cast<char *>(aircannect::Memory::realloc_large(current->text_pool, current->text_size));
        if (text) {
            current->text_pool = text;
            current->text_capacity = current->text_size;
        }
    }
    if (current->option_count && current->option_count < current->option_capacity) {
        auto *options = static_cast<cached_option_t *>(aircannect::Memory::realloc_large(
            current->options, current->option_count * sizeof(cached_option_t)));
        if (options) {
            current->options = options;
            current->option_capacity = current->option_count;
        }
    }
    cache_state = CACHE_READY;
    Log::logf(CAT_CONFIG, LOG_DEBUG,
              "AirSense: Cached %u custom settings (%u options, %u text bytes)\n",
              current->entry_count, current->option_count, current->text_size);
    return true;
}

cached_entry_t *find_entry_locked(const char *name) {
    if (!name || cache_state != CACHE_READY) return nullptr;
    for (uint16_t i = 0; i < current->entry_count; i++) {
        if (strcmp(current->entries[i].name, name) == 0) return &current->entries[i];
    }
    return nullptr;
}

bool ensure_loaded_locked() {
    uint32_t current = generation();
    if (loaded_generation != current) {
        if (!reset_cache_locked(CACHE_UNKNOWN)) return false;
        language_known = false;
        loaded_generation = current;
    }
    if (cache_state == CACHE_READY) {
        uint32_t language = 0;
        if (read_raw_locked("LAN", language)) {
            if (language_known && language != cached_language) {
                Log::logf(CAT_CONFIG, LOG_DEBUG,
                          "AirSense: LAN changed, refreshing custom registry\n");
                if (!reset_cache_locked(CACHE_UNKNOWN)) return false;
                invalidate("LAN change");
                loaded_generation = generation();
            }
            cached_language = language;
            language_known = true;
        }
        if (cache_state == CACHE_READY) return true;
    } else if (cache_state == CACHE_INCOMPATIBLE) {
        return false;
    } else if (cache_state == CACHE_UNSUPPORTED) {
        return true;
    } else if (cache_state == CACHE_FAILED &&
               (int32_t)(millis() - retry_at_ms) < 0) {
        return false;
    }

    if (!reset_cache_locked(CACHE_UNKNOWN)) return false;
    uint32_t language = 0;
    language_known = read_raw_locked("LAN", language);
    if (language_known) cached_language = language;

    if (!discover_locked()) {
        reset_cache_locked(CACHE_FAILED);
        return false;
    }
    return cache_state != CACHE_INCOMPATIBLE;
}

}  // namespace

void init() {
    if (!cache_mutex) cache_mutex = xSemaphoreCreateMutex();
}

bool ensure_loaded() {
    if (!cache_mutex) init();
    if (!cache_mutex) return false;
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    bool result = ensure_loaded_locked();
    xSemaphoreGive(cache_mutex);
    return result;
}

bool contains(const char *name) {
    if (!cache_mutex) return false;
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    bool result = find_entry_locked(name) != nullptr;
    xSemaphoreGive(cache_mutex);
    return result;
}


MetadataLease::~MetadataLease() { reset(); }

void MetadataLease::reset() {
    if (metadata_) __atomic_sub_fetch(&metadata_->readers, 1, __ATOMIC_RELEASE);
    metadata_ = nullptr;
}

bool MetadataLease::acquire() {
    reset();
    if (!cache_mutex) init();
    if (!cache_mutex) return false;
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    bool ok = ensure_loaded_locked() && loaded_generation == CustomSettings::generation();
    if (ok) {
        if (!__atomic_load_n(&current->readers, __ATOMIC_ACQUIRE))
            current->epoch = loaded_generation;
        __atomic_add_fetch(&current->readers, 1, __ATOMIC_ACQ_REL);
        metadata_ = current;
    }
    xSemaphoreGive(cache_mutex);
    return ok;
}

uint32_t MetadataLease::generation() const { return metadata_ ? metadata_->epoch : 0; }
uint16_t MetadataLease::count() const { return metadata_ ? metadata_->entry_count : 0; }

int MetadataLease::find(const char *name) const {
    for (uint16_t i = 0; i < count(); i++)
        if (strcmp(metadata_->entries[i].name, name) == 0) return i;
    return -1;
}

bool MetadataLease::entry(uint16_t index, entry_view_t &view) const {
    if (index >= count()) return false;
    const cached_entry_t &entry = metadata_->entries[index];
    view = {};
    view.kind = entry.kind;
    view.category = entry.category;
    view.mop_mask = entry.mop_mask;
    view.name = entry.name;
    view.label = metadata_->text_pool + entry.label_offset;
    view.units = metadata_->text_pool + entry.units_offset;
    view.groups = metadata_->text_pool + entry.groups_offset;
    view.group_count = entry.group_count;
    view.flags = entry.flags;
    view.width = entry.width;
    view.scale = entry.scale;
    view.step = entry.step;
    view.decimals = entry.decimals;
    view.minimum = entry.minimum;
    view.maximum = entry.maximum;
    view.option_count = entry.option_count;
    return true;
}

bool MetadataLease::option(uint16_t index, uint8_t option_index, option_view_t &view) const {
    if (index >= count()) return false;
    const cached_entry_t &entry = metadata_->entries[index];
    if (option_index >= entry.option_count) return false;
    const cached_option_t &option = metadata_->options[entry.option_start + option_index];
    view = {option.value, metadata_->text_pool + option.label_offset};
    return true;
}

bool MetadataLease::read_raw(uint16_t index, uint32_t &value) const {
    if (index >= count() || generation() != CustomSettings::generation()) return false;
    return read_raw_locked(metadata_->entries[index].name, value);
}

void reclaim() {
    if (!cache_mutex || xSemaphoreTake(cache_mutex, 0) != pdTRUE) return;
    for (Metadata &data : metadata_slots)
        if (&data != current && !__atomic_load_n(&data.readers, __ATOMIC_ACQUIRE))
            clear_allocations(data);
    xSemaphoreGive(cache_mutex);
}

bool write_raw(const char *name, uint32_t value) {
    if (!cache_mutex) return false;
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    cached_entry_t *entry = find_entry_locked(name);
    if (!entry || (entry->flags & FLAG_EDITABLE) == 0) {
        xSemaphoreGive(cache_mutex);
        return false;
    }

    bool valid = true;
    if (entry->kind == KIND_NUMERIC) {
        valid = value >= entry->minimum && value <= entry->maximum;
    } else {
        valid = false;
        for (uint8_t i = 0; i < entry->option_count; i++) {
            if (current->options[entry->option_start + i].value == value) {
                valid = true;
                break;
            }
        }
    }

    bool result = false;
    if (valid) {
        char command[32];
        snprintf(command, sizeof(command), "P S #%s %0*lX", entry->name,
                 entry->width, (unsigned long)value);
        char response[64];
        const char *response_value = nullptr;
        result = query_value(command, response, sizeof(response), response_value) ==
                 QUERY_OK;
    }
    xSemaphoreGive(cache_mutex);
    return result;
}

void invalidate(const char *reason) {
    __atomic_add_fetch(&invalidation_generation, 1, __ATOMIC_ACQ_REL);
    Log::logf(CAT_CONFIG, LOG_DEBUG, "AirSense: Cache invalidated (%s)\n",
              reason ? reason : "unknown");
}

uint32_t generation() {
    return __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE);
}

}  // namespace CustomSettings
