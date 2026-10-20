#pragma once
#include <Arduino.h>
#include <esp_partition.h>
#include "resmed_firmware_image.h"


enum blx_patch_t {
    BLX_PATCH_NONE,         // stock bootloader, integrity checks enabled
    BLX_PATCH_A_DANGEROUS,  // 0xF0 patch, bricks serial flash
    BLX_PATCH_B_SAFE,       // safe method
};

struct fw_verify_result_t {
    bool has_blx;
    bool bid_ok;
    char bid[32];

    bool has_ccx, has_cdx;
    bool blx_crc_ok;
    bool ccx_crc_ok;
    bool cdx_crc_ok;

    blx_patch_t blx_patch;
};

namespace ResmedOta {
    enum class Source : uint8_t { None, Repository, Staging };
    enum class Result : uint8_t { None, Running, Succeeded, Failed, Cancelled };

    struct InstallRequest {
        // Exactly one source. Empty block means inspector auto-detection.
        uint32_t upload_id = 0;
        char path[256] = {};
        char block[8] = {};
        bool flash_blx = false;
        // Explicit manual override of CRC/BID/patch warnings, not I/O/bounds.
        bool force_blx = false;
    };

    struct Status {
        uint32_t revision = 0;
        uint32_t id = 0;
        bool active = false;
        Result result = Result::None;
        char phase[32] = "Idle";
        size_t sent = 0;
        size_t total = 0;
        char error[128] = {};
        Source source = Source::None;
        uint32_t upload_id = 0;
        char source_path[256] = {};
        char block[8] = {};
        char device_bid[32] = {};
        bool prepared = false;
        bool cancellation_requested = false;
        bool cancel_allowed = false;
        bool upload_allowed = false;
        bool install_allowed = false;
        bool dump_allowed = false;
        char blocked[48] = {};
        bool recovery_available = false;
        char recovery_path[256] = {};
        bool confirmation_required = false;
        ResmedFirmwareImage::Result image;
    };

    // Callback-safe admission/copy only. The owner reserves synchronously,
    // then validates/prepares/programs on its worker. No hardware I/O here.
    bool start_install(const InstallRequest &request, const char **error = nullptr);
    // Main-loop publication of cached admission facts. No UART/storage I/O.
    void poll();
    // Copy only; does not refresh admission or join other owners.
    void get_status(Status &out);
    uint32_t revision();

    const esp_partition_t* get_staging_partition();

    fw_verify_result_t verify_image(const esp_partition_t *part, size_t fw_size);

    // block: "FULL", "CMX", "CDX", "CCX", "BLX" (nullptr = auto-detect)
    bool start_flash(const char *block, size_t fw_size,
                     bool flash_blx, bool force_blx, const char **error = nullptr);

    void cancel();

    bool is_active();
    const char* get_phase();
    size_t get_sent();
    size_t get_total();
    const char* last_error();

    const char* detect_block(size_t fw_size);
}
