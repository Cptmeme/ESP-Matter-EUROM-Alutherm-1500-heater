# EUROM Alutherm 1500 WiFi Heater - Native Matter over Thread

This repository contains a custom **Native Matter over Thread** firmware for the **EUROM Alutherm 1500 WiFi** convector heater (3 stages: 600 / 900 / 1500 W).

It replaces the original Tuya Wi-Fi firmware with a custom C++ application running on the **ESP32-C6**, allowing fully local control via Apple Home, Google Home and Home Assistant without any Tuya Cloud dependency or bridges.

> Looking for the **Hombli Smart Convector Heater (2000W)**? That heater speaks a different Tuya dialect and has its own firmware: [ESP-Matter-Hombli-2000W-heater](https://github.com/Cptmeme/ESP-Matter-Hombli-2000W-heater).

---

## 🚀 Features

* **Connectivity:** **Matter over Thread** (Minimal Thread Device). Requires a Thread Border Router such as a HomePod Mini, Apple TV 4K or Nest Hub v2.
* **Thermostat Endpoint:** Heat / Off, target temperature and room temperature. *Heat* switches the heater on in **Program** mode, so the setpoint governs instead of a fixed power level. Eco mode and the manual power levels stay available on the heater itself.
* **Two-way Sync:** Changes made on the heater itself (buttons, remote) are reported back to Matter.
* **Full Tuya Handshake:** This heater only responds after the complete Tuya MCU start-up sequence (heartbeat → product query → work mode → network status "connected" → query all datapoints), followed by a 15 s heartbeat. The handshake, the network status (`0x03`) and the command pacing (one command at a time, each waiting up to 300 ms for the MCU's reply) follow ESPHome's `tuya` component, which is known to work with this heater. Unlike ESPHome, a write only counts as answered once the MCU reports the written value, so a quick "Heat + 21 °C" isn't lost or mistaken for power-button presses. The MCU asks for the time every ~10 s; those requests are not answered, as the heater works without a clock.
* **Factory Reset:** Toggle the heater's power 10 times rapidly (each toggle within 3 s of the previous one) to factory reset the Matter credentials.

---

## 🛠️ Hardware Modifications

The original Tuya Wi-Fi module was replaced with a **WT0132C6-S5** (ESP32-C6) module.

* **SoC:** Espressif ESP32-C6 (RISC-V, Zigbee/Thread/BLE/Wi-Fi 6)
* **Flash:** 4MB
* **Communication:** UART1, 9600 Baud, 8N1

### Wiring

| ESP32-C6 Pin | Connection | Function |
| :--- | :--- | :--- |
| **GPIO 7** | Heater MCU RX | TX (Transmit) |
| **GPIO 6** | Heater MCU TX | RX (Receive) |
| **3.3V** | 3.3V | Power |
| **GND** | GND | Ground |

> **⚠️ Warning:** Do not power the ESP32 via UART/USB while it is connected to the heater's mains-powered UART lines. The voltage potentials may differ. Flash first, then install.

---

## ⚙️ Installation & Build

### Prerequisites
* ESP-IDF v5.5.x (built with v5.5.4)
* ESP-Matter SDK (built against `release/v1.5`)

### Configuration
1.  **Partition Table:** A custom `partitions.csv` is used to allocate space for Matter credentials and Thread storage.
2.  **Endpoint Limit:** The root endpoint counts towards the dynamic endpoint limit. This firmware uses 2 (Root, Thermostat); the limit is set to **3** in `sdkconfig.defaults`. Raise it before adding more endpoints:
    * `Component config` -> `ESP Matter` -> `Maximum dynamic endpoints`
3.  **Thread Device Type:** Minimal Thread Device (`CONFIG_OPENTHREAD_MTD=y`).

### Build Commands

```bash
# 1. Clean the build (Essential when changing endpoint limits)
idf.py fullclean

# 2. Erase Flash (Essential to format the new partition table)
idf.py erase-flash

# 3. Flash and Monitor
idf.py flash monitor
```

The first flash needs download mode: hold **IO9** (pin 18) to GND while powering up. After that, no jumper is needed (see below).

---

## 🔄 Updating the Firmware

### Versions

Bump all three for every release, or the device ignores the update. The build fails if the two numbers differ.

* `PROJECT_VER` (e.g. `"1.1"`) and `PROJECT_VER_NUMBER` (e.g. `2`) in `CMakeLists.txt`
* `CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER` in `sdkconfig` / `sdkconfig.defaults` (same number)

### Over the serial wires (no GPIO9 jumper)

Type `matter esp download` on the serial console (UART0, 115200 baud). The module reboots into the ROM download mode and stays there until it is power-cycled.

```bash
python -m esptool --chip esp32c6 -p <PORT> -b 115200 --before no_reset --after no_reset \
    write_flash 0x20000 build/thermostat.bin
# then power-cycle the module
```

### Over the air (Matter OTA over Thread)

The build also produces `build/thermostat-ota.bin`, a Matter OTA image. Apple Home and Google Home only update certified devices, so you serve it yourself with the Matter SDK's host tools. Build them from the SDK with `./scripts/build/build_examples.py --target darwin-arm64-chip-tool --target darwin-arm64-ota-provider build`.

One-time setup: in the Home app, turn on pairing mode for the heater, then add the provider and the heater to `chip-tool`'s fabric:

```bash
chip-tool pairing code 2 <setup-code-from-home-app>              # heater = node 2
chip-ota-provider-app -f build/thermostat-ota.bin --discriminator 3841 --secured-device-port 5565 &
chip-tool pairing onnetwork-long 1 20202021 3841                 # provider = node 1
chip-tool accesscontrol write acl '[{"fabricIndex": 1, "privilege": 5, "authMode": 2, "subjects": [112233], "targets": null}, {"fabricIndex": 1, "privilege": 3, "authMode": 2, "subjects": null, "targets": [{"cluster": 41, "endpoint": null, "deviceType": null}]}]' 1 0
```

Each update (with the provider running and serving the new `thermostat-ota.bin`):

```bash
chip-tool otasoftwareupdaterequestor announce-otaprovider 1 0 2 0 2 0
chip-tool basicinformation read software-version 2 0              # afterwards: the new number
```

The 1.5 MB image takes about 12 minutes over Thread. The heater then restarts into the new version and keeps it once it has run for a minute while attached to the Thread network. If it doesn't get there within 5 minutes, or crashes before that, the bootloader automatically returns to the previous version.

---

## 📊 Technical: Datapoint Mapping

For reference, the mapping handled by `tuya_driver.cpp`:

| DP ID | Type | Function | Logic |
| :--- | :--- | :--- | :--- |
| **1** | bool | Power | `1`=On, `0`=Off |
| **2** | value | Target Temp | Whole °C |
| **3** | value | Current Temp | Whole °C |
| **4** | enum | Mode | `1`=Program (thermostat), `0`=Manual |
| **12** | bitmap | Fault flags | Read-only (overheat / tip-over) |
| **101** | enum | Manual power level | `0`=600 W, `1`=900 W, `2`=1500 W, `3`=none (Program mode) |
| **102** | bool | Eco mode | `1`=On, `0`=Off |

Setting a manual power level (DP101) makes the heater switch itself to Manual mode (DP4=0).

An ESPHome configuration for the same heater (`heater.yaml`, using the standard `tuya:` component) is in [Hombli-heater-esphome-config](https://github.com/Cptmeme/Hombli-heater-esphome-config).

---

## ⚠️ Disclaimer
This project involves modifying mains-voltage appliances.
* **Always unplug the heater** before opening it.
* The software is provided "as is", without warranty of any kind.
