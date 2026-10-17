#include "custom_settings.h"

#include "custom_settings_protocol.h"
#include "debug_log.h"
#include "qframe.h"
#include "uart_arbiter.h"
#include "clinical_jobs.h"
#include "airsense_state.h"
#include "memory_manager.h"
#include "hex_util.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <atomic>
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
EventGroupHandle_t cache_events = nullptr;
constexpr EventBits_t CACHE_CHANGED = 1;
cache_state_t cache_state = CACHE_UNKNOWN;
Metadata metadata_slots[2];
Metadata *current = &metadata_slots[0];
uint32_t retry_at_ms = 0;
uint32_t cached_language = 0;
bool language_known = false;
uint32_t invalidation_generation = 1;
std::atomic<uint32_t> loaded_generation{0};

enum class DiscoveryStep : uint8_t {
    None, Language, Header, Entry, Capabilities, EnumValue, EnumCount,
    EnumScan, EnumOption,
};
DiscoveryStep discovery_step = DiscoveryStep::None;
uart_transaction_t *discovery_ticket = nullptr;
uint16_t entry_index = 0;
uint8_t registry_version = 0, option_index = 0, option_count = 0;
bool ticket_background = false;
uint16_t foreground_waiters = 0;
uint32_t foreground_until_ms = 0;

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

void complete_discovery_locked(cache_state_t state) {
    discovery_step = DiscoveryStep::None;
    if (state != CACHE_READY) reset_cache_locked(state);
    cache_state = state;
    current->epoch = loaded_generation;
    xEventGroupSetBits(cache_events, CACHE_CHANGED);
}

void next_entry_locked() {
    if (++entry_index < current->entry_count) {
        discovery_step = DiscoveryStep::Entry;
        return;
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
    complete_discovery_locked(CACHE_READY);
    Log::logf(CAT_CONFIG, LOG_DEBUG,
              "AirSense: Cached %u custom settings (%u options, %u text bytes)\n",
              current->entry_count, current->option_count, current->text_size);
}

bool accept_entry_locked(const char *value) {
    CustomSettingsProtocol::entry_t record = {};
    if (!CustomSettingsProtocol::parse_entry(value, registry_version, record)) return false;
    for (uint16_t previous = 0; previous < entry_index; previous++)
        if (strcmp(current->entries[previous].name, record.name) == 0) return false;

    cached_entry_t &entry = current->entries[entry_index];
    memcpy(entry.name, record.name, sizeof(entry.name));
    entry.kind = record.kind == CustomSettingsProtocol::ENTRY_NUMERIC ? KIND_NUMERIC : KIND_ENUM;
    entry.category = record.category;
    entry.mop_mask = record.mop_mask;
    entry.scale = record.scale;
    entry.step = record.step;
    entry.decimals = record.decimals;
    entry.label_offset = add_text_locked(record.label, record.label_len);
    entry.units_offset = add_text_locked(record.units, record.units_len);
    if (entry.label_offset == INVALID_OFFSET || entry.units_offset == INVALID_OFFSET) return false;
    entry.group_count = record.group_count;
    if (record.group_count) {
        entry.groups_offset = current->text_size;
        const char *cursor = record.groups;
        for (uint8_t group = 0; group < record.group_count; group++) {
            const char *text;
            size_t length;
            if (!CustomSettingsProtocol::parse_text(cursor, text, length) ||
                add_text_locked(text, length, true) == INVALID_OFFSET) return false;
        }
    }
    discovery_step = DiscoveryStep::Capabilities;
    return true;
}

bool accept_discovery_locked(query_result_t result, const char *value) {
    using Step = DiscoveryStep;
    if (discovery_step == Step::Language) {
        language_known = result == QUERY_OK &&
            aircannect::parse_hex(value, strlen(value), cached_language);
        discovery_step = Step::Header;
        return true;
    }
    if (discovery_step == Step::Header) {
        if (result == QUERY_ERROR && error_is(value, "6009")) {
            complete_discovery_locked(CACHE_UNSUPPORTED);
            Log::logf(CAT_CONFIG, LOG_DEBUG,
                      "AirSense: Airbreak custom registry unavailable\n");
            return true;
        }
        CustomSettingsProtocol::header_t header = {};
        if (result != QUERY_OK || !CustomSettingsProtocol::parse_header(value, header)) return false;
        if (!CustomSettingsProtocol::supported_version(header.version)) {
            Log::logf(CAT_CONFIG, LOG_WARN, "AirSense: Unsupported custom registry version %02X\n",
                      header.version);
            complete_discovery_locked(CACHE_INCOMPATIBLE);
            return true;
        }
        registry_version = header.version;
        if (!header.count) {
            complete_discovery_locked(CACHE_READY);
            return true;
        }
        current->entries = static_cast<cached_entry_t *>(
            aircannect::Memory::calloc_large(header.count, sizeof(cached_entry_t)));
        if (!current->entries || !reserve_text_locked(1u + uint16_t(header.count) * 64u)) return false;
        current->entry_count = header.count;
        current->text_pool[0] = '\0';
        current->text_size = 1;
        entry_index = 0;
        discovery_step = Step::Entry;
        return true;
    }
    if (discovery_step == Step::Entry)
        return result == QUERY_OK && accept_entry_locked(value);

    cached_entry_t &entry = current->entries[entry_index];
    switch (discovery_step) {
        case Step::Capabilities: {
            if (result != QUERY_OK) return false;
            if (entry.kind == KIND_NUMERIC) {
                CustomSettingsProtocol::numeric_caps_t caps = {};
                if (!CustomSettingsProtocol::parse_numeric_caps(value, caps)) return false;
                entry.flags = caps.flags;
                entry.width = caps.width;
                entry.minimum = caps.minimum;
                entry.maximum = caps.maximum;
                next_entry_locked();
            } else {
                if (!CustomSettingsProtocol::parse_enum_flags(value, entry.flags)) return false;
                entry.width = 4;
                entry.option_start = current->option_count;
                discovery_step = Step::EnumValue;
            }
            return true;
        }
        case Step::EnumValue: {
            uint32_t raw = 0;
            const bool valid = result == QUERY_OK &&
                aircannect::parse_hex(value, strlen(value), raw) && raw <= 0xff;
            option_index = valid ? raw : 0;
            discovery_step = valid ? Step::EnumCount : Step::EnumScan;
            return true;
        }
        case Step::EnumCount:
        case Step::EnumScan:
        case Step::EnumOption: {
            const bool hole = result == QUERY_ERROR && error_is(value, "6033");
            CustomSettingsProtocol::enum_option_t option = {};
            if (!hole && (result != QUERY_OK ||
                !CustomSettingsProtocol::parse_enum_option(value, option))) return false;
            if (discovery_step == Step::EnumOption) {
                if (!hole && (option.count != option_count || !add_enum_option_locked(
                        entry, option_index, option.label, option.label_len))) return false;
                if (++option_index == option_count) next_entry_locked();
            } else if (!hole) {
                if (option.count > MAX_ENUM_OPTIONS) return false;
                option_count = option.count;
                option_index = 0;
                discovery_step = Step::EnumOption;
                if (!option_count) next_entry_locked();
            } else if (discovery_step == Step::EnumCount) {
                option_index = 0;
                discovery_step = Step::EnumScan;
            } else if (++option_index == MAX_ENUM_OPTIONS) {
                next_entry_locked();
            }
            return true;
        }
        default:
            return false;
    }
}

void cancel_discovery_ticket_locked() {
    if (discovery_ticket) Arbiter::cancel_transaction(discovery_ticket);
    discovery_ticket = nullptr;
}

bool refresh_generation_locked() {
    if (loaded_generation == generation()) return true;
    cancel_discovery_ticket_locked();
    discovery_step = DiscoveryStep::None;
    xEventGroupSetBits(cache_events, CACHE_CHANGED);
    if (!reset_cache_locked(CACHE_UNKNOWN)) return false;
    language_known = false;
    loaded_generation = generation();
    return true;
}

bool discovery_allowed(bool foreground) {
    const auto state = Arbiter::get_state();
    if (state != SYS_IDLE && state != SYS_THERAPY) return false;
    if (foreground) return true;
    AirSenseState::Identity identity;
    return AirSenseState::device_standby() && AirSenseState::identity(identity);
}

void advance_discovery_locked() {
    if (!refresh_generation_locked()) return;
    if (cache_state == CACHE_READY || cache_state == CACHE_UNSUPPORTED ||
        cache_state == CACHE_INCOMPATIBLE) return;
    const int32_t remaining = int32_t(foreground_until_ms - millis());
    const bool foreground = foreground_waiters && remaining > 0;
    if (!discovery_allowed(foreground)) {
        cancel_discovery_ticket_locked();
        if (foreground_waiters) xEventGroupSetBits(cache_events, CACHE_CHANGED);
        return;
    }
    if (cache_state == CACHE_FAILED) {
        if (int32_t(millis() - retry_at_ms) < 0) return;
        if (!reset_cache_locked(CACHE_UNKNOWN)) return;
    }
    if (discovery_step == DiscoveryStep::None) discovery_step = DiscoveryStep::Language;
    // A queued idle-only probe cannot become a foreground read during therapy.
    if (discovery_ticket && ticket_background && !AirSenseState::device_standby())
        cancel_discovery_ticket_locked();

    char command[24];
    switch (discovery_step) {
        case DiscoveryStep::Language: strcpy(command, "G S #LAN"); break;
        case DiscoveryStep::Header: strcpy(command, "G C &CSG"); break;
        case DiscoveryStep::Entry:
            snprintf(command, sizeof(command), "G C &CSG %02X", entry_index); break;
        case DiscoveryStep::Capabilities:
            snprintf(command, sizeof(command), "G C #%s", current->entries[entry_index].name); break;
        case DiscoveryStep::EnumValue:
            snprintf(command, sizeof(command), "G S #%s", current->entries[entry_index].name); break;
        default:
            snprintf(command, sizeof(command), "G C #%s %02X",
                     current->entries[entry_index].name, option_index); break;
    }
    const bool scalar = discovery_step == DiscoveryStep::Language ||
                        discovery_step == DiscoveryStep::EnumValue;
    if (!discovery_ticket) {
        const uart_send_window_t window = {0, 0, !foreground, true};
        const auto priority = foreground ? CMD_PRIO_NORMAL : CMD_PRIO_LOW;
        const uint16_t budget = foreground ? remaining : 0;
        discovery_ticket = scalar
            ? Arbiter::begin_var(command + 5, CMD_SRC_INTERNAL, priority, 16, 0, window, budget)
            : Arbiter::begin_cmd(command, CMD_SRC_INTERNAL, priority, QFRAME_MAX_PAYLOAD + 1,
                                 0, window, budget);
        ticket_background = !foreground;
        return;
    }
    char response[QFRAME_MAX_PAYLOAD + 1] = {};
    uint16_t length = sizeof(response);
    query_result_t result = QUERY_TIMEOUT;
    const char *value = nullptr;
    if (Arbiter::transaction_done(discovery_ticket)) {
        if (scalar) {
            if (Arbiter::finish_var(discovery_ticket, response, sizeof(response)) == Arbiter::VarResult::Ok) {
                value = response;
                result = QUERY_OK;
            }
        } else {
            uart_transaction_result_t transaction = {};
            const bool ok = Arbiter::finish_cmd(discovery_ticket, response, &length, &transaction);
            if (length < sizeof(response)) value = qframe_response_value(response);
            if (value) result = ok ? QUERY_OK :
                transaction.terminal_type == QFRAME_TYPE_E ? QUERY_ERROR : QUERY_TIMEOUT;
        }
        discovery_ticket = nullptr;
    } else {
        if (!Arbiter::transaction_expired(discovery_ticket)) return;
        cancel_discovery_ticket_locked();
    }
    if (loaded_generation != generation()) return;
    if (!accept_discovery_locked(result, value)) {
        Log::logf(CAT_CONFIG, LOG_WARN, "AirSense: Discovery failed: %s (%s)\n",
                  command, value ? value : "no response");
        complete_discovery_locked(CACHE_FAILED);
    }
}

cached_entry_t *find_entry_locked(const char *name) {
    if (!name || cache_state != CACHE_READY) return nullptr;
    for (uint16_t i = 0; i < current->entry_count; i++) {
        if (strcmp(current->entries[i].name, name) == 0) return &current->entries[i];
    }
    return nullptr;
}

bool ensure_loaded_locked() {
    if (!refresh_generation_locked()) return false;
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

    if (!discovery_allowed(true)) return false;
    const uint16_t budget = ClinicalJobs::remaining_ms();
    if (!budget) return false;
    const uint32_t expected_generation = loaded_generation;
    const uint32_t until = millis() + budget;
    if (!foreground_waiters || int32_t(until - foreground_until_ms) > 0)
        foreground_until_ms = until;
    ++foreground_waiters;
    while (loaded_generation == expected_generation && generation() == expected_generation &&
           cache_state != CACHE_READY && cache_state != CACHE_UNSUPPORTED &&
           cache_state != CACHE_INCOMPATIBLE && discovery_allowed(true)) {
        if (cache_state == CACHE_FAILED && int32_t(millis() - retry_at_ms) < 0) break;
        const int32_t remaining = int32_t(until - millis());
        if (remaining <= 0) break;
        xSemaphoreGive(cache_mutex);
        xEventGroupWaitBits(cache_events, CACHE_CHANGED, pdTRUE, pdFALSE, pdMS_TO_TICKS(remaining));
        xSemaphoreTake(cache_mutex, portMAX_DELAY);
    }
    --foreground_waiters;
    return loaded_generation == expected_generation && generation() == expected_generation &&
        int32_t(until - millis()) > 0 &&
        (cache_state == CACHE_READY || cache_state == CACHE_UNSUPPORTED);
}

}  // namespace

void init() {
    if (!cache_mutex) cache_mutex = xSemaphoreCreateMutex();
    if (!cache_events) cache_events = xEventGroupCreate();
}

void poll() {
    if (!cache_mutex || !cache_events || xSemaphoreTake(cache_mutex, 0) != pdTRUE) return;
    advance_discovery_locked();
    xSemaphoreGive(cache_mutex);
}

bool ensure_loaded() {
    if (!cache_mutex || !cache_events) init();
    if (!cache_mutex || !cache_events) return false;
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
    if (!cache_mutex || !cache_events) init();
    if (!cache_mutex || !cache_events) return false;
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
