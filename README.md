# EUROM Alutherm 1500 WiFi Heater - Native Matter over Thread

This repository contains a custom **Native Matter over Thread** firmware for the **EUROM Alutherm 1500 WiFi** convector heater (3 stages: 600 / 900 / 1500 W).

It replaces the original Tuya Wi-Fi firmware with a custom C++ application running on the **ESP32-C6**, allowing fully local control via Apple Home, Google Home and Home Assistant without any Tuya Cloud dependency or bridges.

> Looking for the **Hombli Smart Convector Heater (2000W)**? That heater speaks a different Tuya dialect and has its own firmware: [ESP-Matter-Hombli-2000W-heater](https://github.com/Cptmeme/ESP-Matter-Hombli-2000W-heater).

---

## 🚀 Features

* **Connectivity:** **Matter over Thread** (Minimal Thread Device). Requires a Thread Border Router such as a HomePod Mini, Apple TV 4K or Nest Hub v2.
* **Three Endpoints:**
    1.  **Thermostat:** Heat / Off, target temperature and room temperature. *Heat* switches the heater on in **Program** mode, so the setpoint governs instead of a fixed power level.
    2.  **Eco Switch:** Toggles the heater's Eco mode ("EC" on the display).
    3.  **Powerful Switch:** On = manual 1500 W. Off = back to Program (thermostat) mode.
* **Two-way Sync:** Changes made on the heater itself (buttons, remote) are reported back to Matter.
* **Full Tuya Handshake:** This heater only responds after the complete Tuya MCU start-up sequence (heartbeat → product query → work mode → network status "online" → query all datapoints), followed by a 15 s heartbeat. The MCU's clock requests are answered.
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
* ESP-IDF v5.3.x
* ESP-Matter SDK (built against `release/v1.5`)

### Configuration
1.  **Partition Table:** A custom `partitions.csv` is used to allocate space for Matter credentials and Thread storage.
2.  **Endpoint Limit:** The root endpoint counts towards the dynamic endpoint limit, so this firmware needs **4** (Root, Thermostat, Eco, Powerful):
    * `Component config` -> `ESP Matter` -> `Maximum dynamic endpoints` = **4** (or higher)
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
