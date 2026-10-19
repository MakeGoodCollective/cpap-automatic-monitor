#pragma once

#include <stdint.h>

#include "board.h"

// Adapted from AirCANnect storage_io_diagnostics.h (2b8a8442).
namespace SdStorage {

#if AB_STORAGE_SPI_SD_ENABLED
// Mount-only events, emitted with the SPI transaction released.
void spi_init_event(uint32_t requested_hz, uint32_t attempt_hz,
                    uint32_t actual_hz, bool selected, const char *reason,
                    bool power_cycle_required = false);

// Transfer instrumentation. No logging or allocation on the SPI bus.
void spi_io_begin(uint8_t drive, uint32_t sector, uint32_t count, bool write);
void spi_io_command(uint8_t command, uint32_t argument);
void spi_io_fault(const char *phase, int response);
void spi_io_end(int disk_result);
#else
inline void spi_init_event(uint32_t, uint32_t, uint32_t, bool, const char *, bool = false) {}
inline void spi_io_begin(uint8_t, uint32_t, uint32_t, bool) {}
inline void spi_io_command(uint8_t, uint32_t) {}
inline void spi_io_fault(const char *, int) {}
inline void spi_io_end(int) {}
#endif

}  // namespace SdStorage
