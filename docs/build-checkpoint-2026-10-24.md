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

## Upstream code status

The current repository is a fork of AirBridge and already contains:

- a `xiao-esp32s3-plus` PlatformIO target,
- UART support using GPIO1/GPIO2 for this target,
- live stream schema discovery and fallback handling,
- decoding logic for live frames,
- web UI generation code,
- and supporting logging/network infrastructure.

This means the next step should be to build on the existing AirBridge implementation rather than re-create the protocol parser from scratch.

## Immediate next software goal

Get the existing AirBridge-derived firmware running on the prototype hardware and expose useful live values through the existing local interface.

Priority live values:

1. mask pressure,
2. respiratory flow,
3. leak.

Then deliberately test:

- therapy running normally,
- breathing waveform response,
- intentional leak,
- mask removal,
- mask replacement.

Those observations will guide the actual mask-off / large-leak detection thresholds and timing logic.

## Planned later behavior

After reliable live-state detection is established, likely additions include:

- local mask-off alarm,
- short and long snooze options,
- event logging,
- longest mask-off interval,
- large-leak episode tracking,
- overnight summary,
- remote escalation after a sustained problem,
- optional phone-motion context if real data shows it helps.

The first implementation should avoid unnecessary behavioral rules until real overnight data is available.
