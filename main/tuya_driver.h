#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // Required for memcpy/memmove
#include "esp_err.h"

// --- TUYA CONSTANTS ---
#define TUYA_HEADER_0 0x55
#define TUYA_HEADER_1 0xAA

// Command words (4th byte of a frame)
#define TUYA_CMD_HEARTBEAT   0x00
#define TUYA_CMD_PRODUCT     0x01
#define TUYA_CMD_WORK_MODE   0x02
#define TUYA_CMD_WIFI_STATE  0x03
#define TUYA_CMD_SET_DP      0x06   // module -> MCU (control)
#define TUYA_CMD_REPORT_DP   0x07   // MCU -> module (status report)
#define TUYA_CMD_QUERY_DP    0x08   // module -> MCU (query all)
#define TUYA_CMD_GET_TIME    0x1C   // MCU -> module (get local time)

// Wi-Fi/network status byte for TUYA_CMD_WIFI_STATE
#define TUYA_WIFI_ONLINE     0x04   // connected to router + cloud (we are "online" via Thread)

// DP IDs (reverse-engineered from the EUROM Alutherm 1500 WiFi)
#define DP_POWER      1     // bool   - on / off
#define DP_SET_TEMP   2     // value  - target temperature, whole degC
#define DP_CUR_TEMP   3     // value  - current temperature, whole degC
#define DP_MODE       4     // enum   - 1 = Program (thermostat), 0 = Manual
#define DP_FAULT      12    // bitmap - fault flags (read-only: overheat / tip-over)
#define DP_POWER_LVL  101   // enum   - 0=600W, 1=900W, 2=1500W, 3=none(Program)
#define DP_ECO        102   // bool   - eco mode ("EC" on the display)

// DP_MODE (DP4) values
#define MODE_MANUAL   0
#define MODE_PROGRAM  1

// DP_POWER_LVL (DP101) values
#define PWR_600   0
#define PWR_900   1
#define PWR_1500  2   // <- "Powerful"
#define PWR_NONE  3

// Fixed buffer size to avoid heap allocation (thread safe)
#define RX_BUF_SIZE 512

// Handshake / run state machine
enum TuyaInitState {
    TY_HEARTBEAT = 0,  // ping until the MCU answers
    TY_PRODUCT,        // query product info (0x01)
    TY_CONF,           // query working mode (0x02)
    TY_WIFI_STATUS,    // report "online" (0x03)
    TY_QUERY,          // query all datapoints (0x08)
    TY_RUNNING         // normal operation
};

typedef struct {
    bool power;          // DP1
    int target_temp;     // DP2 (degC)
    int current_temp;    // DP3 (degC)
    uint8_t mode;        // DP4  (MODE_*)
    uint8_t power_level; // DP101 (PWR_*)
    bool eco;            // DP102
} heater_state_t;

typedef void (*tuya_state_change_cb_t)(const heater_state_t *state);
typedef void (*tuya_reset_cb_t)();

class TuyaHeaterDriver {
public:
    TuyaHeaterDriver();

    esp_err_t Init(int tx_pin, int rx_pin);

    // Call periodically (e.g. every 50 ms). Reads the UART, drives the
    // handshake, and runs the heartbeat / keep-alive timers.
    void Service();

    // Low-level frames (also usable directly)
    void Poll();
    void Heartbeat();    // 0x00 keep-alive
    void QueryStatus();  // 0x08 request a full datapoint dump

    // Thermostat (heat/off) control
    void SetPower(bool on);   // DP1
    void SetHeat();           // DP1 on + DP4 Program, so the setpoint governs
    void SetTemp(int temp);   // DP2

    // Extras (exposed as separate Matter endpoints)
    void SetEco(bool on);        // DP102
    void SetPowerful(bool on);   // on: DP101=1500W (auto Manual); off: DP4 back to Program

    void SetMode(uint8_t mode);  // DP4

    void SetStateCallback(tuya_state_change_cb_t cb);
    void SetResetCallback(tuya_reset_cb_t cb);

    heater_state_t GetState() const { return m_state; }
    bool IsReady() const { return m_init_state == TY_RUNNING; }

private:
    heater_state_t m_state;
    tuya_state_change_cb_t m_callback;
    tuya_reset_cb_t m_reset_callback;

    int m_uart_num;

    // --- MEMORY FIX: FIXED ARRAY INSTEAD OF VECTOR ---
    uint8_t rx_buffer[RX_BUF_SIZE];
    int rx_count;

    // Handshake state machine
    TuyaInitState m_init_state;
    int64_t m_last_action_time;   // last command sent in the current handshake step
    int m_attempts;               // retries in the current handshake step
    int64_t m_hb_time;            // RUNNING: last heartbeat
    int64_t m_wifi_time;          // RUNNING: last network-status refresh
    int64_t m_query_time;         // RUNNING: last full resync

    // Reset detection (power toggled 10x in quick succession)
    int64_t last_toggle_time;
    int toggle_count;

    void RunStateMachine();
    void AdvanceState(TuyaInitState next);

    void SendProductQuery();   // 0x01
    void SendConfQuery();      // 0x02
    void SendWifiStatus(uint8_t status); // 0x03

    void SetPowerLevel(uint8_t lvl);  // DP101
    void ProcessPacket(const uint8_t *data, int len);
    void ParseDatapoints(const uint8_t *data, int len);
    void SendCommand(uint8_t dp_id, uint8_t type, const uint8_t *value, int len);
    void SendFrame(uint8_t cmd, const uint8_t *payload, int payload_len);
    void NotifyStateChange();
};
