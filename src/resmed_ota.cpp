#include "resmed_ota.h"

#include "airbridge_ota.h"
#include "airsense_state.h"
#include "custom_settings.h"
#include "debug_log.h"
#include "live_stream.h"
#include "memory_manager.h"
#include "qframe.h"
#include "resmed_firmware_repository.h"
#include "resmed_firmware_upload.h"
#include "sd_storage.h"
#include "uart_arbiter.h"

#include <atomic>
#include <stdarg.h>
#include <esp_ota_ops.h>
#include <freertos/task.h>

namespace ResmedOta {
namespace {

namespace Image = ResmedFirmwareImage;
constexpr size_t BLX_BYTES = 16 * 1024;
constexpr size_t SOURCE_CHUNK = 4096;
constexpr size_t UART_CHUNK = 250;
constexpr size_t INTERNAL_RESERVE = 32 * 1024;
constexpr uint32_t FLASH_TASK_STACK = 8192;

portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
Status published;
std::atomic<bool> active{false};
std::atomic<bool> cancel_requested{false};
bool programming = false;  // Protected by status_mux, like published.

struct Work {
    InstallRequest request;
    Source source = Source::None;
    ResmedFirmwareUpload::Source staged;
};
Work work;

void phase(const char *name) {
    portENTER_CRITICAL(&status_mux);
    snprintf(published.phase, sizeof(published.phase), "%s", name);
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
}

bool fail(const char *format, ...) {
    char message[128];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    portENTER_CRITICAL(&status_mux);
    if (!published.error[0]) {
        memcpy(published.error, message, sizeof(message));
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
    return false;
}

bool cancelled() {
    portENTER_CRITICAL(&status_mux);
    const bool result = cancel_requested.load() && !programming;
    portEXIT_CRITICAL(&status_mux);
    return result;
}

bool begin_programming() {
    portENTER_CRITICAL(&status_mux);
    const bool allowed = programming || !cancel_requested.load();
    if (allowed && !programming) {
        programming = true;
        published.cancel_allowed = false;
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
    return allowed;
}

uint8_t *allocate_buffer(size_t size) {
    void *buffer = aircannect::Memory::alloc_large(size, false);
    if (!buffer) {
        // Fixed workload bounds, with headroom for normal UART/BLE operation.
        const auto memory = aircannect::Memory::status();
        if (size <= BLX_BYTES && memory.heap_free >= size + INTERNAL_RESERVE &&
            memory.heap_max_alloc >= size)
            buffer = aircannect::Memory::alloc_large(size, true);
    }
    return static_cast<uint8_t *>(buffer);
}

struct Input {
    SdStorage::Session session;
    SdStorage::Reader file;
    const esp_partition_t *partition = nullptr;
    uint64_t size = 0;
    uint8_t *blx = nullptr;
    uint8_t *scratch = nullptr;

    ~Input() {
        file.close();
        session.end();
        aircannect::Memory::free(blx);
        aircannect::Memory::free(scratch);
    }

    bool read_source(uint64_t offset, uint8_t *out, size_t length) {
        if (offset > size || length > size - offset || length > SOURCE_CHUNK) return false;
        if (partition)
            return esp_partition_read(partition, offset, out, length) == ESP_OK;
        return session.valid() && file.read_at(offset, out, length);
    }

    static bool read(void *context, uint64_t offset, uint8_t *out, size_t length) {
        auto &input = *static_cast<Input *>(context);
        if (offset > input.size || length > input.size - offset) return false;
        if (input.blx && offset < BLX_BYTES) {
            const size_t count = min(length, BLX_BYTES - static_cast<size_t>(offset));
            memcpy(out, input.blx + offset, count);
            offset += count;
            out += count;
            length -= count;
        }
        return !length || input.read_source(offset, out, length);
    }

    Image::Reader reader() { return {this, size, read}; }
};

bool open_input(Input &input) {
    if (work.source == Source::Staging) {
        input.partition = work.staged.partition;
        input.size = work.staged.size;
        if (!input.partition || !input.size || input.size > input.partition->size)
            return fail("invalid_staged_source");
    } else {
        if (!ResmedFirmwareRepository::valid_path(work.request.path))
            return fail("invalid_repository_path");
        phase("Open SD image");
        const uint32_t started = millis();
        while (!input.session.begin(SdStorage::Access::Firmware, work.request.path)) {
            if (cancelled()) return false;
            if (!SdStorage::local_access_allowed() || !SdStorage::mounted())
                return fail("sd_unavailable");
            if (millis() - started >= 5000) return fail("sd_busy");
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (!input.file.open(input.session, work.request.path))
            return fail("source_open_failed");
        input.size = input.file.size();
    }
    if (!input.size || input.size > Image::FULL_SIZE) return fail("invalid_image_size");
    input.scratch = allocate_buffer(SOURCE_CHUNK);
    if (!input.scratch) return fail("source_buffer_unavailable");
    if (input.size == Image::FULL_SIZE ||
        Image::parse_block(work.request.block) == Image::Block::BLX ||
        (!work.request.block[0] && input.size == BLX_BYTES)) {
        if (input.size < BLX_BYTES) return fail("incomplete_blx");
        input.blx = allocate_buffer(BLX_BYTES);
        if (!input.blx) return fail("blx_buffer_unavailable");
        // Finish every BLX source read before any device erase, including FULL.
        for (size_t pos = 0; pos < BLX_BYTES; pos += SOURCE_CHUNK) {
            if (cancelled()) return false;
            if (!input.read_source(pos, input.blx + pos, SOURCE_CHUNK))
                return fail("blx_preflight_read_failed");
        }
    }
    return true;
}

bool device_text(const char *tag, char *out, size_t capacity) {
    return Arbiter::read_var(tag, CMD_SRC_OTA, CMD_PRIO_CRITICAL,
                             out, capacity, 1000) == Arbiter::VarResult::Ok;
}

struct Plan {
    const Image::Profile *profile = nullptr;
    Image::Result image;
    Image::Range blocks[2] = {};
    uint32_t offsets[2] = {};
    size_t count = 0;
    char bid[32] = {};
    char serial[24] = {};
};

bool prepare(Input &input, Plan &plan) {
    phase("Prepare image");
    if (!AirSenseState::system_idle()) return fail("device_not_idle");
    if (!device_text("BID", plan.bid, sizeof(plan.bid))) return fail("device_bid_unavailable");
    plan.profile = Image::profile_for_bid(plan.bid);
    if (!plan.profile) return fail("unsupported_device_bid: %s", plan.bid);
    if (!device_text("SRN", plan.serial, sizeof(plan.serial)) || !plan.serial[0])
        return fail("device_identity_unavailable");

    Image::Block selected = Image::parse_block(work.request.block);
    const bool full = input.size == Image::FULL_SIZE;
    Image::Options options;
    options.platform = plan.profile->platform;
    options.target = full ? Image::Block::FULL : selected;
    const auto shape = full ? Image::Block::FULL : selected != Image::Block::Unknown
        ? selected : input.size == BLX_BYTES ? Image::Block::BLX : Image::Block::Unknown;
    const auto &staged_image = work.staged.image;
    const bool inspected_staging = work.source == Source::Staging &&
        staged_image.status == Image::Status::Ok && staged_image.size == input.size &&
        staged_image.platform == plan.profile->platform && staged_image.target == shape;
    // The promoted upload lease preserves these exact bytes and their inspector
    // facts. Do not repeat its CRC pass. Unclassified blocks need target/layout.
    if (inspected_staging) plan.image = staged_image;
    else if (!Image::inspect(input.reader(), plan.image, options, input.scratch, SOURCE_CHUNK)) {
        portENTER_CRITICAL(&status_mux);
        published.image = plan.image;
        ++published.revision;
        portEXIT_CRITICAL(&status_mux);
        return fail("image_inspection_failed: %u", unsigned(plan.image.status));
    }
    if (selected == Image::Block::Unknown) selected = plan.image.target;
    const bool has_blx = shape == Image::Block::FULL || shape == Image::Block::BLX;
    const bool warnings = !plan.image.crc_ok() ||
        (has_blx && !plan.image.bid_matches) ||
        plan.image.integrity_patch == Image::IntegrityPatch::Unsafe ||
        plan.image.integrity_patch == Image::IntegrityPatch::Partial;

    // Publish factual inspection even when explicit warning confirmation is needed.
    portENTER_CRITICAL(&status_mux);
    published.image = plan.image;
    snprintf(published.device_bid, sizeof(published.device_bid), "%s", plan.bid);
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    if (warnings && !work.request.force_blx) return fail("image_warnings_require_force");

    if (selected == Image::Block::FULL) {
        if (work.request.flash_blx) plan.blocks[plan.count++] = *Image::range(*plan.profile, Image::Block::BLX);
        plan.blocks[plan.count++] = *Image::range(*plan.profile, Image::Block::CMX);
    } else {
        const auto *block = Image::range(*plan.profile, selected);
        if (!block || (!full && input.size != block->size)) return fail("invalid_selected_region");
        plan.blocks[plan.count++] = *block;
    }
    size_t total = 0;
    for (size_t i = 0; i < plan.count; ++i) {
        plan.offsets[i] = full ? plan.blocks[i].image_offset : 0;
        if (plan.offsets[i] > input.size || plan.blocks[i].size > input.size - plan.offsets[i])
            return fail("selected_region_out_of_bounds");
        if (plan.blocks[i].block == Image::Block::BLX && !input.blx)
            return fail("blx_not_preloaded");
        total += plan.blocks[i].size;
    }
    if (cancelled()) return false;
    portENTER_CRITICAL(&status_mux);
    published.prepared = true;
    published.total = total;
    snprintf(published.block, sizeof(published.block), "%s", Image::block_name(selected));
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

// Existing raw UART framing is used only after exclusive stream suspension.
bool send_raw_cmd(const char *cmd, char *resp, uint16_t capacity, uint16_t timeout_ms = 2000) {
    uint8_t frame[QFRAME_MAX_RAW];
    const int length = qframe_build_cmd(cmd, frame, sizeof(frame));
    if (length <= 0) return false;
    Arbiter::clear_rx_frames();
    Arbiter::write_raw(frame, length);
    if (!timeout_ms) return true;
    qframe_t reply;
    const uint32_t started = millis();
    while (millis() - started < timeout_ms) {
        const uint16_t remaining = timeout_ms - (millis() - started);
        if (!Arbiter::wait_frame(&reply, remaining)) break;
        if ((reply.type != QFRAME_TYPE_R && reply.type != QFRAME_TYPE_E) ||
            !qframe_response_matches(frame, length, reply)) continue;
        if (resp && capacity) {
            const size_t count = min(size_t(reply.payload_len), size_t(capacity - 1));
            memcpy(resp, reply.payload, count);
            resp[count] = 0;
        }
        return reply.type == QFRAME_TYPE_R;
    }
    return false;
}

bool raw_text(const char *command, char *out, size_t capacity, uint16_t timeout = 1000) {
    char response[128] = {};
    if (!send_raw_cmd(command, response, sizeof(response), timeout)) return false;
    const char *value = qframe_response_value(response);
    if (!value || !*value || strlen(value) >= capacity) return false;
    snprintf(out, capacity, "%s", value);
    return true;
}

bool raw_hex(const char *command, uint32_t &out, uint16_t timeout = 1000) {
    char value[16];
    if (!raw_text(command, value, sizeof(value), timeout)) return false;
    char *end = nullptr;
    const unsigned long parsed = strtoul(value, &end, 16);
    if (!end || end == value || *end || parsed > UINT32_MAX) return false;
    out = parsed;
    return true;
}

bool enter_bootloader() {
    phase("Enter bootloader");
    uint32_t state = 0;
    if (cancelled()) return false;
    if (raw_hex("G S #BLS", state, 300) && (state == 1 || state == 2)) return true;
    if (!send_raw_cmd("P S #BLL 0001", nullptr, 0, 0)) return fail("bootloader_entry_failed");
    Arbiter::set_baud(57600);
    for (unsigned attempt = 0; attempt < 100 && !cancelled(); ++attempt) {
        if (raw_hex("G S #BLS", state, 100) && (state == 1 || state == 2)) return true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return cancelled() ? false : fail("bootloader_entry_timeout");
}

bool negotiate_baud() {
    phase("Baud negotiate");
    if (!send_raw_cmd("P S #BDD 0002", nullptr, 0, 1000)) return fail("baud_negotiate_failed");
    Arbiter::set_baud(460800);
    return true;
}

int build_record_03(uint8_t *out, size_t capacity, uint32_t address,
                    const uint8_t *data, size_t count) {
    if (count > UART_CHUNK || capacity < count + 7) return -1;
    out[0] = 0x03;
    out[1] = count + 5;
    out[2] = address >> 24;
    out[3] = address >> 16;
    out[4] = address >> 8;
    out[5] = address;
    memcpy(out + 6, data, count);
    out[6 + count] = 0;
    return count + 7;
}

int build_f_payload(uint8_t *out, size_t capacity, const char *block,
                    uint8_t sequence, const uint8_t *record, size_t length, bool complete) {
    if (strlen(block) != 3 || capacity < 5 + (complete ? 0 : length)) return -1;
    memcpy(out, block, 3);
    out[3] = complete ? 'F' : 0;
    out[4] = sequence;
    if (!complete && length) memcpy(out + 5, record, length);
    return 5 + (complete ? 0 : length);
}

bool wait_for_erase(const uint8_t *request, size_t length) {
    const uint32_t started = millis();
    qframe_t reply;
    while (millis() - started < 30000) {
        if (!Arbiter::wait_frame(&reply, 500)) continue;
        if (!qframe_response_matches(request, length, reply)) continue;
        if (reply.type == QFRAME_TYPE_R) return true;
        if (reply.type == QFRAME_TYPE_E) return fail("device_erase_error");
    }
    return fail("device_erase_timeout");
}

bool flash_one_block(const Image::Reader &reader, size_t source_offset,
                     const Image::Range &block, uint8_t *scratch, bool complete) {
    const char *name = Image::block_name(block.block);
    char label[32];
    snprintf(label, sizeof(label), "Erase %s", name);
    phase(label);
    if (!begin_programming()) return false;
    char command[24];
    snprintf(command, sizeof(command), "P F *%s 0000", name);
    uint8_t erase[64];
    const int erase_length = qframe_build_cmd(command, erase, sizeof(erase));
    if (erase_length <= 0) return fail("erase_frame_failed");
    Arbiter::clear_rx_frames();
    Arbiter::write_raw(erase, erase_length);
    if (!wait_for_erase(erase, erase_length)) return false;
    vTaskDelay(pdMS_TO_TICKS(300));

    snprintf(label, sizeof(label), "Flash %s", name);
    phase(label);
    uint8_t sequence = 0;
    unsigned frames = 0;
    // A source read is 4 KiB, independent of the UART's 250-byte record limit.
    // Never consult cancellation after erase: especially BLX must finish.
    for (size_t offset = 0; offset < block.size;) {
        const size_t count = min(SOURCE_CHUNK, size_t(block.size - offset));
        if (!reader.read_at(reader.context, source_offset + offset, scratch, count))
            return fail("source_read_failed: %s+0x%X", name, unsigned(offset));
        for (size_t pos = 0; pos < count;) {
            const size_t chunk = min(UART_CHUNK, count - pos);
            uint8_t record[UART_CHUNK + 7], payload[UART_CHUNK + 16], frame[QFRAME_MAX_RAW];
            const int record_length = build_record_03(record, sizeof(record),
                block.flash_start + offset + pos, scratch + pos, chunk);
            if (record_length <= 0) return fail("record_frame_failed");
            const int payload_length = build_f_payload(payload, sizeof(payload), name,
                sequence, record, record_length, false);
            if (payload_length <= 0) return fail("payload_frame_failed");
            const int frame_length = qframe_build('f', payload, payload_length, frame, sizeof(frame));
            if (frame_length <= 0) return fail("data_frame_failed");
            Arbiter::write_raw(frame, frame_length);
            pos += chunk;
            ++sequence;
            portENTER_CRITICAL(&status_mux);
            published.sent += chunk;
            ++published.revision;
            portEXIT_CRITICAL(&status_mux);
            if (++frames % 20 == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                qframe_t reply;
                while (Arbiter::wait_frame(&reply, 10)) {
                    if (reply.type == QFRAME_TYPE_E) return fail("device_program_error: %s", name);
                }
            }
        }
        offset += count;
    }
    uint32_t error = 0;
    if (!raw_hex("G S #BLE", error) || (error != 0 && error != 0x700D))
        return fail("device_program_error: %s %04X", name, unsigned(error));
    if (complete) {
        phase("Complete transfer");
        uint8_t payload[8], frame[64];
        const int payload_length = build_f_payload(payload, sizeof(payload), name,
            sequence, nullptr, 0, true);
        const int length = qframe_build('f', payload, payload_length, frame, sizeof(frame));
        if (length <= 0) return fail("completion_frame_failed");
        Arbiter::write_raw(frame, length);
    }
    return true;
}

bool confirm_install(const Plan &plan) {
    phase("Verify device");
    // Completion clears BLE, but does not verify the image. The transfer status
    // was checked before that marker; require fresh app status/identity here.
    uint32_t error = 0;
    Arbiter::set_baud(57600);
    for (unsigned attempt = 0; attempt < 10; ++attempt) {
        // Successful bootloader commands restart its ~2s activity timeout.
        // Leave a real quiet window, including after a clean-status retry.
        vTaskDelay(pdMS_TO_TICKS(3000));
        uint32_t state = UINT32_MAX;
        if (!raw_hex("G S #BLS", state, 1000)) continue;
        if (state != 0) {
            if (raw_hex("G S #BLE", error) && error && error != 0x700D)
                return fail("device_bootloader_error: %04X", unsigned(error));
            continue;
        }
        char bid[32], serial[24], sid[64];
        if (!raw_hex("G S #BLE", error) || error != 0)
            return fail("device_bootloader_error: %04X", unsigned(error));
        if (!raw_text("G S #BID", bid, sizeof(bid)) ||
            !raw_text("G S #SRN", serial, sizeof(serial)) ||
            !raw_text("G S #SID", sid, sizeof(sid))) continue;
        // FORCE permits an explicitly selected different BLX; it does not
        // permit a different physical device to satisfy this operation.
        const char *expected_bid = plan.bid;
        bool wrote_blx = false;
        for (size_t i = 0; i < plan.count; ++i)
            wrote_blx |= plan.blocks[i].block == Image::Block::BLX;
        if (wrote_blx && plan.image.bid_present) expected_bid = plan.image.bid;
        if (strcmp(bid, expected_bid) || strcmp(serial, plan.serial))
            return fail("device_identity_changed");
        return true;
    }
    return fail("device_did_not_return_to_application");
}

bool run_install() {
    Input input;
    Plan plan;
    if (!open_input(input) || cancelled() || !prepare(input, plan)) return false;
    if (cancelled()) return false;
    // Preparing has no irreversible side effects. Claim raw UART only now.
    if (!AirSenseState::system_idle()) return fail("device_not_idle");
    Arbiter::set_state(SYS_OTA_AIRSENSE);
    bool success = false;
    do {
        // The SD owner must explicitly support this lease in OTA mode. Never
        // discover an incompatible/revoked lease after erasing the target.
        if (!input.partition && !input.session.valid()) { fail("sd_ota_lease_unavailable"); break; }
        phase("Claim UART");
        if (!Arbiter::wait_idle(3000)) { fail("uart_busy"); break; }
        phase("Suspend streams");
        if (!LiveStream::suspend_for_ota()) { fail("stream_suspend_failed"); break; }
        char current_bid[32], current_serial[24];
        if (!raw_text("G S #BID", current_bid, sizeof(current_bid)) ||
            !raw_text("G S #SRN", current_serial, sizeof(current_serial)) ||
            strcmp(current_bid, plan.bid) || strcmp(current_serial, plan.serial)) {
            fail("device_identity_changed"); break;
        }
        if (cancelled()) break;
        CustomSettings::invalidate("ResMed OTA");
        AirSenseState::invalidate_identity();
        if (!enter_bootloader() || !negotiate_baud()) break;
        bool programmed = true;
        for (size_t i = 0; i < plan.count; ++i) {
            if (!flash_one_block(input.reader(), plan.offsets[i], plan.blocks[i],
                                  input.scratch, i + 1 == plan.count)) {
                programmed = false; break;
            }
            if (i + 1 < plan.count) {
                phase("BLX mode timeout");
                vTaskDelay(pdMS_TO_TICKS(2500));
                Arbiter::set_baud(57600);
                uint32_t state = 0;
                if (!raw_hex("G S #BLS", state) || (state != 1 && state != 2)) {
                    fail("lost_bootloader_after_blx"); programmed = false; break;
                }
                if (!negotiate_baud()) { programmed = false; break; }
            }
        }
        if (programmed) success = confirm_install(plan);
    } while (false);
    Arbiter::set_baud(57600);
    Arbiter::set_state(SYS_IDLE);
    AirSenseState::request_refresh();
    return success;
}

void finish(bool success) {
    portENTER_CRITICAL(&status_mux);
    published.active = false;
    published.cancel_allowed = false;
    published.result = success ? Result::Succeeded :
        (!programming && cancel_requested.load() && !published.error[0])
        ? Result::Cancelled : Result::Failed;
    if (published.result == Result::Failed && !published.error[0])
        snprintf(published.error, sizeof(published.error), "install_failed");
    snprintf(published.phase, sizeof(published.phase), "%s", success ? "Complete" :
        published.result == Result::Cancelled ? "Cancelled" : "Error");
    ++published.revision;
    active.store(false);
    portEXIT_CRITICAL(&status_mux);
}

void flash_task(void *) {
    const bool success = run_install();  // All C++ I/O/buffer owners unwind here.
    if (work.staged.token) ResmedFirmwareUpload::release_source(work.staged.token);
    finish(success);
    // No timer infers our lifetime; this is the only normal reservation release.
    OtaManager::end_resmed_flash();
    vTaskDelete(nullptr);
}

}  // namespace

bool start_install(const InstallRequest &request, const char **error) {
    if (error) *error = nullptr;
    const auto reject = [error](const char *reason) {
        if (error) *error = reason;
        return false;
    };
    if (!memchr(request.path, 0, sizeof(request.path)) ||
        !memchr(request.block, 0, sizeof(request.block)) ||
        (bool(request.upload_id) == bool(request.path[0]))) return reject("invalid_install_request");
    if (request.block[0] && Image::parse_block(request.block) == Image::Block::Unknown)
        return reject("unknown_block");
    if (request.path[0] && !ResmedFirmwareRepository::valid_path(request.path))
        return reject("invalid_repository_path");

    Work next;
    next.request = request;
    next.source = Source::Repository;
    bool claimed = false;
    if (request.upload_id) {
        ResmedFirmwareUpload::Status upload;
        if (!ResmedFirmwareUpload::copy_status(request.upload_id, upload) ||
            upload.state != ResmedFirmwareUpload::State::Ready || !upload.source_ready)
            return reject("upload_not_ready");
        if (upload.destination == ResmedFirmwareUpload::Destination::Staging) {
            if (!ResmedFirmwareUpload::claim_source(request.upload_id, next.staged, error)) return false;
            next.source = Source::Staging;
            claimed = true;
        } else {
            if (!ResmedFirmwareRepository::valid_path(upload.path)) return reject("invalid_repository_path");
            snprintf(next.request.path, sizeof(next.request.path), "%s", upload.path);
        }
    }
    if (!claimed && !OtaManager::begin_resmed_flash(error)) return false;

    // Reservation, params and visible activity are initialized before scheduling.
    work = next;
    portENTER_CRITICAL(&status_mux);
    const uint32_t previous_revision = published.revision;
    uint32_t id = published.id + 1;
    if (!id) ++id;
    published = {};
    published.id = id;
    published.revision = previous_revision + 1;
    published.active = true;
    published.result = Result::Running;
    published.source = next.source;
    published.upload_id = request.upload_id;
    snprintf(published.source_path, sizeof(published.source_path), "%s", next.request.path);
    snprintf(published.block, sizeof(published.block), "%s", request.block);
    snprintf(published.phase, sizeof(published.phase), "Preparing");
    published.cancel_allowed = true;
    cancel_requested.store(false);
    programming = false;
    active.store(true);
    portEXIT_CRITICAL(&status_mux);
    if (xTaskCreatePinnedToCore(flash_task, "resmed_ota", FLASH_TASK_STACK,
                                nullptr, 3, nullptr, 1) != pdPASS) {
        fail("worker_create_failed");
        if (next.staged.token) ResmedFirmwareUpload::release_source(next.staged.token);
        finish(false);
        OtaManager::end_resmed_flash();
        return reject("worker_create_failed");
    }
    return true;
}

void cancel() {
    portENTER_CRITICAL(&status_mux);
    if (published.active && !cancel_requested.exchange(true)) {
        published.cancellation_requested = true;
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
}

bool is_active() { return active.load(); }

void poll() {
    // Admission facts are cached-owner reads, never UART or filesystem work.
    ResmedFirmwareUpload::Status upload;
    ResmedFirmwareUpload::get_status(upload);
    const bool staged = upload.state == ResmedFirmwareUpload::State::Ready &&
        upload.destination == ResmedFirmwareUpload::Destination::Staging && upload.source_ready;
    const char *blocked = OtaManager::resmed_flash_blocked(staged);
    OtaManager::Status ota;
    const bool can_upload = OtaManager::get_status(ota) && !ota.upload_blocked;
    portENTER_CRITICAL(&status_mux);
    const bool allowed = !published.active && !blocked;
    const bool upload_allowed = !published.active && can_upload;
    const char *reason = published.active ? "resmed_ota_active" : blocked ? blocked : "";
    if (published.install_allowed != allowed || published.upload_allowed != upload_allowed ||
        strcmp(published.blocked, reason)) {
        published.install_allowed = allowed;
        published.upload_allowed = upload_allowed;
        snprintf(published.blocked, sizeof(published.blocked), "%s", reason);
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = published;
    portEXIT_CRITICAL(&status_mux);
}

uint32_t revision() {
    portENTER_CRITICAL(&status_mux);
    const uint32_t value = published.revision;
    portEXIT_CRITICAL(&status_mux);
    return value;
}

const esp_partition_t *get_staging_partition() {
    const auto *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "resmed");
    return partition ? partition : esp_ota_get_next_update_partition(esp_ota_get_running_partition());
}

fw_verify_result_t verify_image(const esp_partition_t *partition, size_t size) {
    fw_verify_result_t result = {};
    if (!partition || size > partition->size) return result;
    Input input;
    input.partition = partition;
    input.size = size;
    Image::Result image;
    Image::inspect(input.reader(), image);
    snprintf(result.bid, sizeof(result.bid), "%s", image.bid);
    result.bid_ok = image.bid_supported;
    for (size_t i = 0; i < image.block_count; ++i) {
        const auto &check = image.blocks[i];
        if (check.range.block == Image::Block::BLX) { result.has_blx = true; result.blx_crc_ok = check.crc_ok; }
        if (check.range.block == Image::Block::CCX) { result.has_ccx = true; result.ccx_crc_ok = check.crc_ok; }
        if (check.range.block == Image::Block::CDX) { result.has_cdx = true; result.cdx_crc_ok = check.crc_ok; }
    }
    result.blx_patch = image.integrity_patch == Image::IntegrityPatch::Unsafe ? BLX_PATCH_A_DANGEROUS :
        image.integrity_patch == Image::IntegrityPatch::Safe ? BLX_PATCH_B_SAFE : BLX_PATCH_NONE;
    return result;
}

const char *detect_block(size_t size) {
    if (size == Image::FULL_SIZE) return "FULL";
    if (size == BLX_BYTES) return "BLX";
    return nullptr;
}

bool start_flash(const char *block, size_t size, bool flash_blx, bool force_blx, const char **error) {
    ResmedFirmwareUpload::Status upload;
    ResmedFirmwareUpload::get_status(upload);
    if (!upload.id || size != upload.size) {
        if (error) *error = "upload_not_ready";
        return false;
    }
    InstallRequest request;
    request.upload_id = upload.id;
    snprintf(request.block, sizeof(request.block), "%s", block ? block : "");
    request.flash_blx = flash_blx;
    request.force_blx = force_blx;
    return start_install(request, error);
}

// Compatibility accessors for callers transitioning to one copied Status.
const char *get_phase() {
    static thread_local char value[32];
    portENTER_CRITICAL(&status_mux);
    memcpy(value, published.phase, sizeof(value));
    portEXIT_CRITICAL(&status_mux);
    return value;
}
const char *last_error() {
    static thread_local char value[128];
    portENTER_CRITICAL(&status_mux);
    memcpy(value, published.error, sizeof(value));
    portEXIT_CRITICAL(&status_mux);
    return value;
}
size_t get_sent() {
    portENTER_CRITICAL(&status_mux);
    const size_t value = published.sent;
    portEXIT_CRITICAL(&status_mux);
    return value;
}
size_t get_total() {
    portENTER_CRITICAL(&status_mux);
    const size_t value = published.total;
    portEXIT_CRITICAL(&status_mux);
    return value;
}

}  // namespace ResmedOta
