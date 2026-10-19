#include "sd_spi_card.h"

#if AB_STORAGE_SPI_SD_ENABLED
#include <limits.h>
#include <diskio.h>
#include <sd_diskio.h>
#include <vfs_api.h>

namespace SdSpi {
namespace {
constexpr uint8_t NO_DRIVE = 0xff;
constexpr int64_t CONFIGURED_FREQUENCY = AB_SPI_SD_FREQ_HZ;
constexpr uint32_t RAW_FREQUENCY = CONFIGURED_FREQUENCY < 20000000
    ? CONFIGURED_FREQUENCY : 20000000;

esp_err_t disk_error(DRESULT result) {
    switch (result) {
    case RES_OK: return ESP_OK;
    case RES_NOTRDY: return ESP_ERR_INVALID_STATE;
    case RES_PARERR: return ESP_ERR_INVALID_ARG;
    case RES_WRPRT: return ESP_ERR_NOT_SUPPORTED;
    default: return ESP_FAIL;
    }
}
}  // namespace

Card::Card() : fs::SDFS(fs::FSImplPtr(new VFSImpl())), bus_(HSPI) {}

Card::~Card() {
    end();
}

bool Card::begin() {
    if (_pdrv != NO_DRIVE) return _impl->mountpoint() != nullptr;
    if (!begin_bus()) return false;

    if (!fs::SDFS::begin(AB_SPI_SD_CS_GPIO, bus_, AB_SPI_SD_FREQ_HZ,
                         "/sdcard", 8, false)) {
        end();
        return false;
    }
    return true;
}

void Card::end() {
    (void)end_raw();
}

bool Card::begin_raw() {
    if (_pdrv != NO_DRIVE) return _impl->mountpoint() == nullptr;
    if (!begin_bus()) return false;

    _pdrv = sdcard_init(AB_SPI_SD_CS_GPIO, &bus_, RAW_FREQUENCY);
    esp_err_t result = ESP_FAIL;
    if (_pdrv != NO_DRIVE && !(disk_initialize(_pdrv) & (STA_NOINIT | STA_NODISK))) {
        const sdcard_type_t type = cardType();
        result = (type == CARD_SD || type == CARD_SDHC || type == CARD_MMC) &&
            sectorSize() == 512 && numSectors() != 0 ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    }
    if (result != ESP_OK) {
        end();
        return false;
    }

    // disk_initialize leaves the bus at 400 kHz. Select the same bounded rate
    // as diskio, without issuing a command or overriding mounted fallback rates.
    bus_.beginTransaction(SPISettings(RAW_FREQUENCY, MSBFIRST, SPI_MODE0));
    bus_.endTransaction();
    return true;
}

esp_err_t Card::end_raw() {
    esp_err_t result = ESP_OK;
    _impl->mountpoint(nullptr);
    if (_pdrv != NO_DRIVE) {
        const esp_err_t unmount = sdcard_unmount(_pdrv);
        result = sdcard_uninit(_pdrv);
        // Arduino frees the disk registration even when VFS unregister fails.
        _pdrv = NO_DRIVE;
        if (result == ESP_OK) result = unmount;
    }
    bus_.end();
    return result;
}

uint32_t Card::frequency_khz() {
    if (_pdrv == NO_DRIVE || !bus_.bus()) return 0;
    return spiClockDivToFrequency(bus_.bus(), bus_.getClockDivider()) / 1000;
}

esp_err_t Card::read_sectors(uint8_t *buffer, uint32_t sector, size_t count) {
    const esp_err_t result = check_transfer(buffer, sector, count);
    if (result != ESP_OK) return result;
    return disk_error(disk_read(_pdrv, buffer, sector, count));
}

esp_err_t Card::write_sectors(const uint8_t *buffer, uint32_t sector, size_t count) {
    const esp_err_t result = check_transfer(buffer, sector, count);
    if (result != ESP_OK) return result;
    return disk_error(disk_write(_pdrv, buffer, sector, count));
}

bool Card::begin_bus() {
    // -1 asks Arduino to select default pins; narrowing could do the same.
    constexpr int pins[] = {AB_SPI_SD_CS_GPIO, AB_SPI_SD_SCK_GPIO,
                            AB_SPI_SD_MISO_GPIO, AB_SPI_SD_MOSI_GPIO};
    for (size_t i = 0; i < 4; ++i) {
        if (pins[i] < 0 || pins[i] > INT8_MAX) return false;
        for (size_t j = 0; j < i; ++j)
            if (pins[i] == pins[j]) return false;
    }
    // sdcard_init takes a signed int frequency, not SPISettings' uint32_t.
    if (CONFIGURED_FREQUENCY <= 0 || CONFIGURED_FREQUENCY > INT_MAX) return false;

    // SPIClass retains its divider across end(); never restart raw mode at the
    // previous mounted clock. This also configures the divider before bus start.
    bus_.setFrequency(400000);
    if (bus_.begin(AB_SPI_SD_SCK_GPIO, AB_SPI_SD_MISO_GPIO,
                   AB_SPI_SD_MOSI_GPIO, AB_SPI_SD_CS_GPIO)) return true;
    bus_.end();
    return false;
}

esp_err_t Card::check_transfer(const void *buffer, uint32_t sector, size_t count) {
    if (_pdrv == NO_DRIVE || _impl->mountpoint()) return ESP_ERR_INVALID_STATE;
    const size_t sectors = numSectors();
    // Arduino's multi-sector routines narrow count to int and advance by 512.
    if (!buffer || !count || count > INT_MAX || count > SIZE_MAX / 512 ||
        sector >= sectors || count > sectors - sector) return ESP_ERR_INVALID_ARG;
    return ESP_OK;
}

}  // namespace SdSpi
#endif
