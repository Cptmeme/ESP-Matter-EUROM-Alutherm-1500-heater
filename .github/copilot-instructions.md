
# Copilot Instructions for Thermostat (ESP-Matter)

## Project Overview
- Implements a Thermostat device using the ESP-Matter data model for ESP32 and related chips.
- Entry point: `main/app_main.cpp` (defines device initialization, Matter node creation, and event handling).
- Device logic and hardware abstraction: `main/app_driver.cpp` and `main/app_priv.h`.
- Uses Espressif's ESP-Matter and CHIP/ConnectedHomeIP libraries (external, not vendored here).

## Architecture & Patterns
- **Device Model**: Follows the Matter/CHIP data model. Endpoints, clusters, and attributes are managed via ESP-Matter APIs. The device identifies as a Thermostat entity for HomeKit compatibility.
- **Driver Abstraction**: All hardware interaction is abstracted in driver functions (see `app_driver_*` APIs, now stubbed for thermostat).
- **Callbacks**: Attribute updates and identification are handled via callback functions registered at node creation.
- **Commissioning**: Handles Matter commissioning events and window management in `app_event_cb`.
- **Defaults**: Device state defaults are set via `app_driver_thermostat_set_defaults()` after startup.

## Build & Development
- **Build System**: Uses ESP-IDF CMake. Top-level `CMakeLists.txt` configures ESP-Matter and device paths via environment variables (`ESP_MATTER_PATH`, `ESP_MATTER_DEVICE_PATH`).
- **Build Command**: Use ESP-IDF tooling (e.g., `idf.py build`, `idf.py flash`).
- **Environment**: Requires ESP-IDF and ESP-Matter SDKs. Set `ESP_MATTER_PATH` before building.
- **No custom test or CI scripts** are present in this repo.

## Key Conventions
- **All device-specific logic** is in `main/`.
- **No additional environment or post-commissioning setup** is required (see `README.md`).
- **Public Domain/CC0**: All code is explicitly licensed as such.
- **Thread support**: Conditional on `CHIP_DEVICE_CONFIG_ENABLE_THREAD` macro.
- **Button events**: Button press is stubbed for thermostat; implement as needed.

## Integration Points
- **External dependencies**: ESP-Matter, CHIP/ConnectedHomeIP, ESP-IDF, and board-specific drivers (not included here).
- **Commissioning**: Integrates with Matter's commissioning and fabric management APIs.

## Examples
- To add a new attribute or cluster, extend the callbacks in `app_main.cpp` and update driver logic in `app_driver.cpp`.
- To change default device state, update `DEFAULT_TEMPERATURE` in `app_priv.h`.

## References
- [ESP-Matter documentation](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/developing.html)
- Key files: `main/app_main.cpp`, `main/app_driver.cpp`, `main/app_priv.h`, `CMakeLists.txt`

---
For questions about build, flashing, or SDK setup, see the ESP-Matter docs linked above.
