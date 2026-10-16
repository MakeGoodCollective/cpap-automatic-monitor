# Quick Start

## What you need
- AirBridge hardware: the PCB, or a hand-wired XIAO ESP32-S3 Plus with or
  without SD. See [hardware.md](hardware.md) for variants and which features
  need SD.
- AirSense 10 with edge connector access
- A computer with Chrome or Edge and a USB data cable

## Wiring

PCB: [hardware_pcb.md](hardware_pcb.md).
Hand-wired: [hardware_wiring.md](hardware_wiring.md).

## Flash firmware

Download the `-initial.zip` matching your board from the
[latest release](https://github.com/m-kozlowski/airbridge/releases/latest)
and extract the `-initial.bin` image.

Open [ESPWebTool](https://esptool.spacehuhn.com/) in Chrome or Edge:

1. Connect the board over USB. M5Stamp Pico needs a 3.3 V USB-to-serial adapter.
2. Select **Connect** and choose its serial port.
3. Add the extracted `-initial.bin` at address `0x0`.
4. Select **Program** and wait for flashing to finish.

### Build from source instead

Install PlatformIO and check out the matching release tag.

XIAO with SD (PCB or hand-wired):

```bash
pio run -e xiao-esp32s3-plus-sdmmc4 -t upload
```

XIAO without SD:

```bash
pio run -e xiao-esp32s3-plus -t upload
```

M5Stamp Pico (legacy):

```bash
pio run -e m5stamp-pico -t upload
```

## Configure WiFi

On a fresh installation, connect to the `airbridge_...` WiFi access point
(password `airbridge`) and open `http://192.168.4.1/`.

Log in with username `admin` and password `airbridge`. The setup wizard guides
you through your WiFi network, AirSense connection, time, optional sync
destinations, and Web UI credentials. Use **Save & next**, then **Finish**.
You can reopen the wizard from **Config > Initial setup**.

When flashing with PlatformIO, you can instead copy `provision.env.example`
to `provision.env` and fill in your WiFi credentials before flashing. These
settings are applied automatically after the USB upload.

## Verify it works

Once connected to your WiFi, open `http://airbridge.local/` (mDNS). If you
changed `hostname`, use `http://<hostname>.local/` instead. `http://airbridge/`
also works if your router resolves DHCP hostnames. If the name does not
resolve, use the device IP from your router's DHCP leases.

The **Dashboard** should show the AirSense name and serial number. With the
AirSense powered on, connected, and not running therapy, its state is
`IDLE`.

See [Configuration](configuration.md) for available settings.
