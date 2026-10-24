# CPAP Automatic Monitor — Engineering History

This document preserves the main engineering paths explored before arriving at the current design. It is a working history, not the final build guide.

## 1. Wall-power monitoring

The first approach used a Shelly metering plug to look for mask removal or large leaks indirectly through AC power behavior.

Features considered included rolling averages, range, percentile spread, standard deviation, and several-minute windows intended to reduce dependence on therapy pressure.

This was attractive because it was non-invasive, but the signal was indirect and the Shelly showed reliability problems such as freezes, HTTP failures, and reboots. Power monitoring may still be useful as an auxiliary signal, but not as the primary detector.

## 2. Wi-Fi SD-card approaches

We investigated automatic access to the CPAP SD card using devices and cards such as FlashAir, ez Share, Eyefi, FluCard, and AirStash.

Advantages:
- access to detailed therapy files,
- no need to decode a live service interface.

Problems:
- obsolete or hard-to-source hardware,
- fragile firmware/Lua behavior,
- read/write timing concerns while the CPAP is using the card,
- and file latency that is poorly suited to rapid alarms.

The SD card remains useful for historical analysis, but not as the preferred real-time path.

## 3. Network architecture experiments

A two-device approach was considered because some phones cannot stay connected to both a device access point and normal home Wi-Fi at the same time.

That led to a possible “miner + mule” ESP32 design, and also consideration of a Chromebook or phone with an extra Wi-Fi adapter.

The simpler architecture is one ESP32 at the CPAP that joins the home network directly.

## 4. ESP32 board selection

Boards considered included ESP32-C3 SuperMini, M5Stack AtomS3 Lite, and Seeed XIAO ESP32-S3 variants.

The prototype uses a **Seeed Studio XIAO ESP32-S3 Plus with external antenna**. The upstream AirBridge project includes a build target for this board.

## 5. AirSense/AirCurve 10 service UART

The major simplification was using the service UART documented and implemented by the AirBridge/Airbreak projects.

Confirmed characteristics:
- 3.3 V UART logic,
- 57600 baud,
- 8N1 framing,
- framed request/response protocol with CRC,
- live data streams enabled by command.

This changed the project from indirect inference to direct live instrumentation.

## 6. Physical connector development

The service connector required a custom removable interface.

Ideas explored included:
- hand-cut PCB tabs,
- keying-slot and contact-pitch measurements,
- isolating unused copper fingers,
- using donor gold-finger material from old cards,
- and a more elaborate hand-routed adapter board.

The prototype was simplified to a tab/contact interface with point-to-point wiring.

Verified signal wiring:
- CPAP TX -> 100 ohm -> ESP32 RX
- CPAP RX <- 100 ohm <- ESP32 TX
- CPAP GND -> ESP32 GND

For the XIAO ESP32-S3 Plus prototype:
- ESP32 TX = GPIO1 / D0
- ESP32 RX = GPIO2 / D1

Future documentation should always use endpoint-qualified labels such as **CPAP TX -> ESP32 RX** to avoid confusion when viewing the board from opposite sides.

## 7. Safe bring-up

The interface was tested incrementally:
- continuity through both series resistors,
- no unintended signal shorts,
- ground continuity,
- ESP32 signal pins initially configured as inputs,
- idle voltage checks near 3.3 V,
- then UART initialization.

A simple identity query was used as the first bidirectional communications test. It succeeded on two machines with software IDs in the SX567-030x family, confirming both UART directions, ground, baud rate, framing, CRC, and the homemade connector.

## 8. First live data

The next test enabled the documented PMD mask-data stream. Live frames such as:

`PMDE501200000`

were received continuously.

The stream provides fields for mask pressure, respiratory flow, and leak. This was the first successful real-time data from the CPAP.

## 9. Current direction

Current architecture:

```text
AirSense/AirCurve 10 service UART
            |
            v
XIAO ESP32-S3 Plus
            |
            +-- home Wi-Fi --> local web status/graphs
            +-- event logging
            +-- overnight summaries
            +-- remote alerts

Android phone
            +-- local alarm / snooze logic
            +-- optional motion-based context later
```

The first implementation should stay simple, log what happens, and add more complex rules only when actual data shows they are useful.

## 10. What to preserve for the final Make Good Collective write-up

Useful story elements:
1. Why wall-power monitoring was attractive but indirect.
2. Why Wi-Fi SD-card approaches were explored and deprioritized.
3. Why a single ESP32 on home Wi-Fi is simpler than a two-device bridge.
4. How the service UART changed the problem.
5. How the connector was developed and safely verified.
6. The first successful identity query and live PMD stream.
7. Which alarm rules ultimately proved useful in real testing.
