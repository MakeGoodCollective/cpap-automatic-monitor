#pragma once
#include <stdint.h>

namespace AirSenseState {

struct Identity {
    uint32_t generation = 0;
    bool valid = false;
    uint16_t mid = 0, vid = 0;
    char srn[24] = {};
    char pna[64] = {};
    // Preserve the device's wire spelling for Identification.tgt.
    char mid_text[9] = {}, vid_text[9] = {};
};

// Run from the main loop; UART work remains serialized by Arbiter.
void poll();
void request_refresh();

// Copies the current snapshot without UART. PNA is optional for core identity.
bool identity(Identity &out);
uint32_t identity_generation();
uint32_t identity_revision();
// Restart, confirmed connection loss or bootloader entry; also stops capture.
void invalidate_identity();

int rop();
int mhr();
int mop();
bool present_recently();

bool system_idle();
// Local firmware updates do not require a responsive or attached AirSense.
bool esp_update_allowed();
bool device_standby();
bool local_background_allowed();

}  // namespace AirSenseState
