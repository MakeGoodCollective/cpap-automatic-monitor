#include "resmed_firmware_upload.h"
#include "resmed_firmware_repository.h"
#include "airbridge_ota.h"
#include "resmed_ota.h"
#include "memory_manager.h"
#include "debug_log.h"
#include "board.h"

#include <Arduino.h>
#include <algorithm>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <utility>
#if AB_STORAGE_HAS_SDCARD
#include <FS.h>
#include "sd_storage.h"
#endif

namespace ResmedFirmwareUpload {
namespace {
SemaphoreHandle_t mutex = nullptr;
TaskHandle_t task = nullptr;
Status status;
uint32_t next_id = 0;
uint8_t *ring = nullptr;
size_t capacity = 0, produced = 0, consumed = 0;
bool active = false, final_received = false, sealed = false, claimed = false;
Drained drain_notify;
uint32_t drain_end = 0;
const esp_partition_t *partition = nullptr;
uint32_t last_input = 0;
constexpr uint32_t INPUT_TIMEOUT_MS = 30000;

void changed() { ++status.revision; }

void reject(const char *error) {
    if (!status.error[0]) snprintf(status.error, sizeof(status.error), "%s", error);
    status.cancel_allowed = false;
}

bool interrupted() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool result = status.error[0] != 0;
    xSemaphoreGive(mutex);
    return result;
}

bool partition_read(void *context, uint64_t offset, uint8_t *out, size_t size) {
    const auto *source = static_cast<const esp_partition_t *>(context);
    return !interrupted() && offset <= source->size && size <= source->size - offset &&
        esp_partition_read(source, size_t(offset), out, size) == ESP_OK;
}

#if AB_STORAGE_HAS_SDCARD
bool file_read(void *context, uint64_t offset, uint8_t *out, size_t size) {
    return !interrupted() && static_cast<SdStorage::Reader *>(context)->read_at(offset, out, size);
}

bool publish_file(SdStorage::Session &session, const char *part, const char *path) {
    return session.mutate(part, path, [&](fs::FS &fs) {
        return !fs.exists(path) && fs.rename(part, path);
    });
}

bool store_bootloader(SdStorage::Session &session, SdStorage::Reader &source,
                      const ResmedFirmwareImage::Result &image, uint8_t *scratch,
                      size_t scratch_size, char *destination, bool explicit_bootloader) {
    using namespace ResmedFirmwareImage;
    if (!explicit_bootloader && image.dump_extension != DumpExtension::Recognized) return true;
    if (!image.bid_supported || image.status != ResmedFirmwareImage::Status::Ok) return !explicit_bootloader;
    const BlockCheck *blx = nullptr;
    for (size_t i = 0; i < image.block_count; ++i)
        if (image.blocks[i].range.block == Block::BLX) blx = &image.blocks[i];
    if (!blx || !blx->crc_ok) return !explicit_bootloader;
    if (!ResmedFirmwareRepository::bootloader_path(image.bid, destination, 256)) return false;
    if (image.target == Block::BLX) return session.ensure_parent(destination);
    bool exists = false;
    if (!session.run([&](fs::FS &fs) { exists = fs.exists(destination); return true; })) return false;
    if (exists) { destination[0] = 0; return true; }
    char temporary[256];
    snprintf(temporary, sizeof(temporary), "%s.part", destination);
    if (!session.ensure_parent(destination) || !session.run([&](fs::FS &fs) {
        return !fs.exists(temporary);
    })) return false;
    SdStorage::Writer output;
    bool ok = output.open(session, temporary);
    for (size_t offset = 0; ok && offset < blx->range.size;) {
        const size_t size = std::min(scratch_size, size_t(blx->range.size) - offset);
        ok = file_read(&source, blx->source_offset + offset, scratch, size) && output.write(scratch, size);
        offset += size;
    }
    ok = ok && output.finish(blx->range.size);
    output.close();
    if (ok) ok = publish_file(session, temporary, destination);
    if (ok) ResmedFirmwareRepository::notify_published(destination);
    else session.mutate(temporary, [&](fs::FS &fs) { return !fs.exists(temporary) || fs.remove(temporary); });
    destination[0] = 0;
    return ok;
}
#endif

void run_upload() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const Destination destination = status.destination;
    const bool explicit_bootloader = status.bootloader;
    const uint32_t id = status.id;
    const uint32_t expected_size = status.size;
    char path[256];
    snprintf(path, sizeof(path), "%s", status.path);
    xSemaphoreGive(mutex);
    const char *destination_name = destination == Destination::Repository ? "repository" : "staging";
    Log::logf(CAT_OTA, LOG_INFO,
        "ResMed upload started id=%lu dest=%s path=%s bytes=%u\n",
        static_cast<unsigned long>(id), destination_name, path, unsigned(expected_size));
    const char *failure = nullptr;
    uint32_t written = 0;
    size_t erased = 0;
    ResmedFirmwareImage::Result image;
#if AB_STORAGE_HAS_SDCARD
    bool committed = false;
    SdStorage::Session session;
    SdStorage::Writer output;
    SdStorage::Reader input;
    char temporary[256] = {};
    bool private_created = false;
    if (destination == Destination::Repository) {
        snprintf(temporary, sizeof(temporary), "%s/.upload-%lu.part",
                 ResmedFirmwareRepository::ROOT, static_cast<unsigned long>(id));
        if (!session.begin()) failure = "storage_busy";
        else if (!session.ensure_parent(path)) failure = "directory_failed";
        else if (!session.run([&](fs::FS &fs) {
            return !fs.exists(path) && !fs.exists(temporary);
        })) failure = "file_exists";
        else if (!(private_created = output.open(session, temporary))) failure = "open_failed";
    }
#else
    if (destination == Destination::Repository) failure = "unsupported";
#endif
    if (destination == Destination::Staging) {
        partition = ResmedOta::get_staging_partition();
        if (!partition) failure = "staging_partition_missing";
    }

    while (!failure) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (status.error[0]) { xSemaphoreGive(mutex); break; }
        const size_t offset = consumed % capacity;
        const size_t size = std::min({produced - consumed, capacity - offset, MAX_CHUNK_BYTES});
        const bool finished = final_received && !size;
        if (!size && !finished && uint32_t(millis() - last_input) >= INPUT_TIMEOUT_MS)
            reject("upload_timeout");
        xSemaphoreGive(mutex);
        if (finished) break;
        if (!size) { ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)); continue; }
        if (!written && ring[offset] == 0xe9) { failure = "esp_image_not_resmed"; break; }
        if (destination == Destination::Staging) {
            if (written > partition->size || size > partition->size - written) {
                failure = "staging_partition_too_small";
                break;
            }
            while (erased < written + size) {
                if (erased > partition->size || partition->size - erased < 4096 ||
                    esp_partition_erase_range(partition, erased, 4096) != ESP_OK) {
                    failure = "partition_erase_failed";
                    break;
                }
                erased += 4096;
            }
            if (!failure && esp_partition_write(partition, written, ring + offset, size) != ESP_OK)
                failure = "partition_write_failed";
        }
#if AB_STORAGE_HAS_SDCARD
        else if (!output.write(ring + offset, size)) failure = "sd_write_failed";
#endif
        if (failure) break;
        written += size;
        xSemaphoreTake(mutex, portMAX_DELAY);
        consumed += size;
        status.written = written;
        changed();
        Drained notify;
        const bool drained_success = !status.error[0] && status.written >= drain_end;
        if (drain_notify && (status.error[0] || drained_success)) notify.swap(drain_notify);
        xSemaphoreGive(mutex);
        if (notify) notify(drained_success);
        vTaskDelay(1);
    }

    if (!failure && !interrupted()) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        status.state = State::Verifying;
        changed();
        xSemaphoreGive(mutex);
        ResmedFirmwareImage::Reader reader;
        reader.size = written;
        if (destination == Destination::Staging) {
            reader.context = const_cast<esp_partition_t *>(partition);
            reader.read_at = partition_read;
        }
#if AB_STORAGE_HAS_SDCARD
        else if (!output.finish(written) || !input.open(session, temporary)) failure = "sd_finish_failed";
        else { reader.context = &input; reader.read_at = file_read; }
#endif
        if (!failure) {
            ResmedFirmwareImage::inspect(reader, image, {}, ring, capacity);
            if (image.status == ResmedFirmwareImage::Status::ReadError ||
                image.status == ResmedFirmwareImage::Status::InvalidArgument)
                failure = "image_read_failed";
            if (explicit_bootloader && (written != 0x4000 ||
                image.status != ResmedFirmwareImage::Status::Ok || !image.bid_supported ||
                !image.bid_matches || !image.crc_ok())) failure = "invalid_bootloader";
        }
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (!failure && !status.error[0]) {
            sealed = true;
            status.cancel_allowed = false;
            changed();
        }
        xSemaphoreGive(mutex);
#if AB_STORAGE_HAS_SDCARD
        if (!failure && !interrupted() && destination == Destination::Repository) {
            char bootloader[256] = {};
            if (!store_bootloader(session, input, image, ring, std::min(capacity, MAX_CHUNK_BYTES), bootloader,
                                  explicit_bootloader))
                failure = "bootloader_store_failed";
            if (*bootloader) snprintf(path, sizeof(path), "%s", bootloader);
            input.close();
            if (!failure && !(committed = publish_file(session, temporary, path))) failure = "publish_failed";
            if (committed) ResmedFirmwareRepository::notify_published(path);
        }
#endif
    }
#if AB_STORAGE_HAS_SDCARD
    input.close();
    output.close();
    if (private_created && !committed)
        session.mutate(temporary, [&](fs::FS &fs) { return !fs.exists(temporary) || fs.remove(temporary); });
    session.end();
#endif
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (failure) reject(failure);
    const bool success = !status.error[0];
    aircannect::Memory::free(ring);
    ring = nullptr;
    capacity = produced = consumed = 0;
    status.image = image;
    status.source_ready = success;
    status.cancel_allowed = success && destination == Destination::Staging;
    if (!success || destination != Destination::Staging) {
        OtaManager::end_manual_upload(success, success ? nullptr : status.error);
        partition = nullptr;
    }
    status.state = success ? State::Ready : !strcmp(status.error, "cancelled") ? State::Cancelled : State::Failed;
    snprintf(status.path, sizeof(status.path), "%s", path);
    active = false;
    task = nullptr;
    status.complete = true;
    changed();
    Drained notify;
    notify.swap(drain_notify);
    const bool drained_success = success && written >= drain_end;
    const uint32_t received = status.received;
    char error[sizeof(status.error)];
    snprintf(error, sizeof(error), "%s", status.error);
    xSemaphoreGive(mutex);
    const bool cancelled = !strcmp(error, "cancelled");
    Log::logf(CAT_OTA, success || cancelled ? LOG_INFO : LOG_ERROR,
        "ResMed upload %s id=%lu dest=%s path=%s bytes=%u received=%u%s%s\n",
        success ? "complete" : cancelled ? "cancelled" : "failed",
        static_cast<unsigned long>(id), destination_name, path, unsigned(written),
        unsigned(received), success ? "" : ": ", error);
    if (notify) notify(drained_success);
}

void worker(void *) {
    run_upload();
    vTaskDeleteWithCaps(nullptr);
}
}

void init() {
    if (mutex) return;
    mutex = xSemaphoreCreateMutex();
}

Result begin_upload(Destination destination, const char *filename, size_t size, uint32_t &id,
                    bool bootloader) {
    id = 0;
    if (!mutex) return Result::Unavailable;
    if (destination != Destination::Repository && destination != Destination::Staging) return Result::Unsupported;
    char path[ResmedFirmwareRepository::MAX_PATH];
    if (destination == Destination::Repository && !bootloader) {
        const int length = snprintf(path, sizeof(path), "%s/%s",
                                    ResmedFirmwareRepository::ROOT, filename ? filename : "");
        if (length < 0 || size_t(length) >= sizeof(path) ||
            !ResmedFirmwareRepository::valid_path(path) ||
            !strncasecmp(filename, "bootloaders/", 12))
            return Result::InvalidName;
    } else {
        if (!ResmedFirmwareRepository::valid_filename(filename)) return Result::InvalidName;
        const int length = destination == Destination::Repository
            ? snprintf(path, sizeof(path), "%s/%s", ResmedFirmwareRepository::ROOT, filename)
            : snprintf(path, sizeof(path), "%s", filename);
        if (length < 0 || size_t(length) >= sizeof(path)) return Result::InvalidName;
    }
    if (size > MAX_IMAGE_BYTES) return Result::InvalidSize;
    if (bootloader && (destination != Destination::Repository || (size && size != 0x4000)))
        return Result::InvalidSize;
    if (destination == Destination::Repository) {
#if AB_STORAGE_HAS_SDCARD
        if (!SdStorage::mounted() || !SdStorage::local_access_allowed()) return Result::Unavailable;
#else
        return Result::Unsupported;
#endif
    }
    if (xSemaphoreTake(mutex, 0) != pdTRUE) return Result::Busy;
    if (active || (status.source_ready && status.destination == Destination::Staging) ||
        claimed || next_id == UINT32_MAX) {
        xSemaphoreGive(mutex);
        return Result::Busy;
    }
    const char *error = nullptr;
    if (!OtaManager::begin_manual_upload(&error)) {
        xSemaphoreGive(mutex);
        return Result::Busy;
    }
    capacity = std::min(size ? size : MAX_IMAGE_BYTES, size_t(16 * 1024));
    ring = static_cast<uint8_t *>(aircannect::Memory::alloc_large(capacity, false));
    if (!ring) {
        capacity = std::min(capacity, MAX_CHUNK_BYTES);
        ring = static_cast<uint8_t *>(aircannect::Memory::alloc_large(capacity, true));
    }
    if (!ring) {
        OtaManager::end_manual_upload(false, "upload_no_memory");
        capacity = 0;
        xSemaphoreGive(mutex);
        return Result::NoMemory;
    }
    const uint32_t revision = status.revision;
    status = {};
    status.revision = revision + 1;
    status.id = id = ++next_id;
    status.destination = destination;
    status.state = State::Receiving;
    status.cancel_allowed = true;
    status.size = size;
    status.bootloader = bootloader;
    snprintf(status.path, sizeof(status.path), "%s", path);
    active = true;
    final_received = sealed = claimed = false;
    partition = nullptr;
    produced = consumed = 0;
    last_input = millis();
    // Partition erase/write disables the flash/PSRAM cache: staging must keep
    // its executing stack internal even when the copied input ring is PSRAM.
    const unsigned stack_caps = destination == Destination::Staging
        ? MALLOC_CAP_INTERNAL : MALLOC_CAP_SPIRAM;
    int created = xTaskCreatePinnedToCoreWithCaps(worker, "fw_upload", 6144,
        nullptr, 1, &task, 0, stack_caps | MALLOC_CAP_8BIT);
    if (created != pdPASS && destination != Destination::Staging)
        created = xTaskCreatePinnedToCoreWithCaps(worker, "fw_upload", 6144,
            nullptr, 1, &task, 0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (created != pdPASS) {
        aircannect::Memory::free(ring);
        ring = nullptr;
        capacity = 0;
        active = false;
        status.complete = true;
        status.state = State::Failed;
        reject("worker_no_memory");
        OtaManager::end_manual_upload(false, status.error);
        id = 0;
        xSemaphoreGive(mutex);
        return Result::NoMemory;
    }
    xSemaphoreGive(mutex);
    return Result::Accepted;
}

Result submit_chunk(uint32_t id, size_t index, const uint8_t *data, size_t length, bool final) {
    if (!mutex) return Result::Unavailable;
    xSemaphoreTake(mutex, portMAX_DELAY);
    Result result = Result::Accepted;
    const size_t limit = status.bootloader ? 0x4000 : MAX_IMAGE_BYTES;
    if (!id || status.id != id) result = Result::UnknownId;
    else if (!active || sealed || status.error[0]) result = Result::Closed;
    else if (final_received || index != status.received || length > MAX_CHUNK_BYTES ||
             index > limit || length > limit - index ||
             (length && !data) || (status.size && (index > status.size || length > status.size - index)) ||
             (final && (!index && !length)) || (final && status.size && index + length != status.size)) {
        result = Result::InvalidChunk;
        reject("invalid_chunk");
    } else if (length > capacity - (produced - consumed)) {
        result = Result::QueueFull;
        reject("queue_full");
    } else {
        const size_t offset = produced % capacity;
        const size_t first = std::min(length, capacity - offset);
        if (first) memcpy(ring + offset, data, first);
        if (length > first) memcpy(ring, data + first, length - first);
        produced += length;
        status.received += length;
        final_received = final;
        if (final) status.size = status.received;
        last_input = millis();
    }
    if (status.id == id && active) changed();
    if (task) xTaskNotifyGive(task);
    xSemaphoreGive(mutex);
    return result;
}

namespace {
bool cancel_upload(uint32_t id, bool discard_ready) {
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool accepted = id && status.id == id && status.cancel_allowed && !claimed &&
        ((active && !sealed) || (discard_ready && status.source_ready && status.destination == Destination::Staging));
    if (accepted) {
        if (active) reject("cancelled");
        else {
            status.source_ready = false;
            status.state = State::Cancelled;
            reject("cancelled");
            partition = nullptr;
            OtaManager::end_manual_upload(false, "cancelled");
        }
        changed();
    }
    if (accepted && task) xTaskNotifyGive(task);
    xSemaphoreGive(mutex);
    return accepted;
}
}

bool cancel(uint32_t id) { return cancel_upload(id, true); }
bool cancel_input(uint32_t id) { return cancel_upload(id, false); }

void get_status(Status &out) {
    out = {};
    if (!mutex) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    out = status;
    xSemaphoreGive(mutex);
}

bool copy_status(uint32_t id, Status &out) {
    get_status(out);
    return id && out.id == id;
}

Result when_drained(uint32_t id, size_t end, Drained notify) {
    if (!mutex) return Result::Unavailable;
    xSemaphoreTake(mutex, portMAX_DELAY);
    Result result = Result::Accepted;
    bool immediate = false, success = false;
    if (!id || status.id != id) result = Result::UnknownId;
    else if (!end || end > status.received || !notify) result = Result::InvalidChunk;
    else if (drain_notify) result = Result::Busy;
    else if (status.error[0] || status.complete || status.written >= end) {
        immediate = true;
        success = !status.error[0] && status.written >= end;
    } else {
        drain_end = end;
        drain_notify = std::move(notify);
    }
    xSemaphoreGive(mutex);
    if (immediate) notify(success);
    return result;
}

bool copy_source(uint32_t id, Source &out) {
    out = {};
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (id && status.id == id && status.source_ready && !claimed && status.destination == Destination::Staging)
        out = {partition, status.size, status.id, status.image};
    xSemaphoreGive(mutex);
    return out.partition != nullptr;
}

bool claim_source(uint32_t id, Source &out, const char **error) {
    out = {};
    if (error) *error = "stale_upload";
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (id && status.id == id && status.source_ready && !claimed && status.destination == Destination::Staging &&
        OtaManager::begin_resmed_flash(error, true)) {
        claimed = true;
        status.cancel_allowed = false;
        out = {partition, status.size, status.id, status.image};
        changed();
    }
    xSemaphoreGive(mutex);
    return out.partition != nullptr;
}

bool release_source(uint32_t token) {
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool accepted = token && status.id == token && status.source_ready && status.destination == Destination::Staging;
    if (accepted) {
        if (!claimed) OtaManager::end_manual_upload(true);
        claimed = false;
        partition = nullptr;
        status.source_ready = false;
        status.cancel_allowed = false;
        changed();
    }
    xSemaphoreGive(mutex);
    return accepted;
}

const char *result_name(Result result) {
    switch (result) {
    case Result::Accepted: return "accepted";
    case Result::Busy: return "busy";
    case Result::Unsupported: return "unsupported";
    case Result::Unavailable: return "unavailable";
    case Result::InvalidName: return "invalid_name";
    case Result::InvalidSize: return "invalid_size";
    case Result::UnknownId: return "unknown_id";
    case Result::InvalidChunk: return "invalid_chunk";
    case Result::QueueFull: return "queue_full";
    case Result::NoMemory: return "no_memory";
    case Result::Closed: return "closed";
    }
    return "unavailable";
}

}  // namespace ResmedFirmwareUpload
