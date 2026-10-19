#pragma once

#include "board.h"

#if AB_STORAGE_SPI_SD_ENABLED
#include <SD.h>
#include <esp_err.h>

namespace SdSpi {

// SdStorage serializes access and owns mode changes, formatting and publication.
class Card : public fs::SDFS {
public:
    Card();
    ~Card();
    Card(const Card &) = delete;
    Card &operator=(const Card &) = delete;

    // Same-mode starts are idempotent; end the current mode before switching.
    bool begin();
    void end();
    // Initializes diskio without mounting or formatting any filesystem.
    bool begin_raw();
    // Releases the drive and bus even on failure; returns the uninit error.
    esp_err_t end_raw();

    uint8_t drive() const { return _pdrv; }
    // Current hardware divider, not the configured limit or a status cache.
    uint32_t frequency_khz();
    // Raw mode only. Sector counts, not byte counts; no implicit initialization.
    esp_err_t read_sectors(uint8_t *buffer, uint32_t sector, size_t count);
    esp_err_t write_sectors(const uint8_t *buffer, uint32_t sector, size_t count);

private:
    bool begin_bus();
    esp_err_t check_transfer(const void *buffer, uint32_t sector, size_t count);

    SPIClass bus_;
};

}  // namespace SdSpi
#endif
