# CPAP Automatic Monitor — Build Checkpoint (2026-10-24)

This checkpoint records the current prototype state so later changes can be compared against a known working configuration.

## Hardware

- Seeed Studio XIAO ESP32-S3 Plus
- External antenna attached
- Custom AirSense/AirCurve 10 service-port tab/contact interface
- Approximately 100 ohm series resistor in each UART signal line

## Verified wiring

- CPAP TX -> 100 ohm -> ESP32 RX
- CPAP RX <- 100 ohm <- ESP32 TX
- CPAP GND -> ESP32 GND

For this board/build:
- ESP32 TX = GPIO1 / D0
- ESP32 RX = GPIO2 / D1

The matching upstream PlatformIO target is `xiao-esp32s3-plus`, which defines:

```text
AB_AS10_TX_GPIO=1
AB_AS10_RX_GPIO=2
```

## Electrical checks completed

- Ground continuity confirmed.
- Signal paths confirmed through the series resistors.
- No unintended TX-to-TX or RX-to-RX continuity.
- No signal-to-ground shorts found.
- With the CPAP powered, UART idle levels were approximately 3.2–3.3 V.

## UART settings

- 57600 baud
- 8 data bits
- no parity
- 1 stop bit
- 3.3 V logic

## Successful identity-query test

A framed service-protocol identity query was transmitted from the ESP32 to the CPAP and a valid response was received.

Two tested machines returned software IDs in the SX567-030x family, confirming that:

- CPAP TX -> ESP32 RX works,
- ESP32 TX -> CPAP RX works,
- ground is correct,
- baud and framing are correct,
- CRC generation is correct,
- and the homemade connector is functional.

## Successful live-stream test

The PMD stream was enabled and continuous live frames were received, including frames such as:

```text
PMDE501200000
PMDE601200000
PMDE701200000
```

The changing sequence byte confirms continuous live communication.

The upstream AirBridge code defines live stream schemas that include fields such as:

- `MKP` — mask pressure
- `RFL` — respiratory flow
- `LYK` — leak

Additional stream schemas in the upstream code include event and breathing-summary channels such as APN, CSN, PBT, and BRH.

## AirBridge firmware baseline verified

The AirBridge-derived firmware has now been built and flashed successfully to the XIAO ESP32-S3 Plus using the repository's `xiao-esp32s3-plus` PlatformIO target.

Verified end-to-end path:

```text
AirSense 10
  -> custom service-UART connector
  -> XIAO ESP32-S3 Plus
  -> AirBridge firmware
  -> Wi-Fi
  -> browser dashboard
```

The existing AirBridge dashboard has been verified to show:

- AirSense state transitions between `STANDBY` and `THERAPY`,
- live mask pressure,
- live respiratory flow,
- live mask leak.

Starting and stopping therapy at the CPAP is reflected correctly in the browser dashboard. This establishes a known-good baseline for the UART decoder, live data path, web UI, and prototype hardware.

## Home-network baseline verified

The ESP32 has also been verified to connect to the normal 2.4 GHz home Wi-Fi network in AirBridge `Auto` mode.

Confirmed:

- the selected home access point is visible at strong signal level,
- STA association succeeds,
- DHCP assigns a LAN address,
- the dashboard is reachable over the normal home LAN,
- `airbridge.local` resolves and loads, confirming mDNS operation,
- the fallback AirBridge AP can remain available only as a recovery path while Auto mode handles normal STA operation.

The actual private SSID, password, and DHCP address are intentionally not recorded in this public repository.

## Wi-Fi troubleshooting note: duplicate SSID profiles

A configuration trap was found while correcting capitalization of a saved SSID.

The setup/configuration flow added a second Wi-Fi profile rather than replacing the original profile. This left two saved entries whose SSIDs differed only by capitalization. AirBridge scan matching is case-insensitive, so the stale first profile could still match the visible access point and continue to be selected.

Symptoms included repeated transitions through scanning, connecting, and AP fallback, while the serial debug log showed the stale SSID spelling.

Useful diagnostics were:

```text
$WIFI STATUS
$WIFI LIST
$LOG WIFI DEBUG
```

USB serial debug output can be monitored at 115200 baud with:

```bash
pio device monitor -b 115200
```

The stale profile was identified with `$WIFI LIST` and removed with:

```text
$WIFI REMOVE <slot>
```

After removal, the remaining corrected profile connected successfully.

This behavior is worth keeping in mind for a future firmware/UI fix: editing an existing SSID should not silently leave a stale case-variant profile that can continue to win matching/selection.

## Upstream code status

The current repository is a fork of AirBridge and already contains:

- a `xiao-esp32s3-plus` PlatformIO target,
- UART support using GPIO1/GPIO2 for this target,
- live stream schema discovery and fallback handling,
- decoding logic for live frames,
- web UI generation code,
- network and mDNS support,
- and supporting logging infrastructure.

The working baseline means the project can now build on the existing decoded live values rather than re-create the protocol parser or network stack.

## Immediate next software goal

Add the smallest possible **temporary diagnostic capture** layer on top of the existing live data path before implementing alarms.

The AirSense itself already records the long-term therapy data. The ESP32 should therefore **not become a redundant physiological data logger**. For development, it only needs enough short-lived history to characterize deliberate test cases such as:

- normal therapy,
- intentional leak,
- complete mask removal,
- mask replacement,
- therapy stop/start.

The diagnostic capture should hold a limited trace in RAM/PSRAM and make it downloadable over the local web interface, for example as CSV. It is acceptable for this diagnostic trace to disappear on reboot; its purpose is algorithm development, not clinical archival.

The first detector should remain observational: record pressure/flow/leak behavior and candidate mask-off intervals without yet triggering the final alarm/snooze/escalation behavior.

Those measurements can then be compared with the AirSense's own recorded data and used to choose thresholds and timing rules based on actual behavior rather than assumptions.

An SD-card attachment is therefore **not required for this phase**. Persistent AirBridge storage can be reconsidered later only if a real need emerges that is not already met by the CPAP's own SD-card record.

## Planned later behavior

After reliable live-state detection is established, likely additions include:

- local mask-off alarm,
- short and long snooze options,
- compact event logging for alarm-system-specific events such as detections, snoozes, and escalations,
- longest mask-off interval,
- large-leak episode tracking if useful for alarm logic,
- remote escalation after a sustained problem,
- optional phone-motion context if real data shows it helps.

The ESP32 should avoid duplicating long-term pressure/flow/leak history already recorded by the AirSense.
