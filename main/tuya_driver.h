#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // Required for memcpy/memmove
#include <atomic>
#include "esp_err.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

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

// Network status byte for TUYA_CMD_WIFI_STATE: 0x03 = connected to the router.
// This is what ESPHome's tuya component reports to this heater (it only sends
// 0x04 "cloud connected" to protocol v3 MCUs with an API client attached).
#define TUYA_WIFI_CONNECTED  0x03

// Command pacing, taken from ESPHome's tuya component: one command in flight,
// at least 10 ms apart, wait up to 300 ms for the MCU's reply, and retry
// handshake commands up to 5 times.
#define TUYA_COMMAND_DELAY_US    10000
#define TUYA_RECEIVE_TIMEOUT_US  300000
#define TUYA_MAX_RETRIES         5
#define TUYA_HEARTBEAT_US        15000000   // ESPHome: every 15 s
#define TUYA_PING_US             1000000    // until the MCU first answers
#define TUYA_QUEUE_LEN           16
#define TUYA_MAX_PAYLOAD         16

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

// Handshake / run state machine (same steps as ESPHome's TuyaInitState)
enum TuyaInitState {
    TY_HEARTBEAT = 0,  // ping until the MCU answers
    TY_PRODUCT,        // query product info (0x01)
    TY_CONF,           // query working mode (0x02)
    TY_WIFI,           // report network status (0x03), wait for the MCU's ack
    TY_DATAPOINT,      // query all datapoints (0x08), wait for the first report
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

// One queued frame (module -> MCU)
typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[TUYA_MAX_PAYLOAD];
} tuya_command_t;

typedef void (*tuya_state_change_cb_t)(const heater_state_t *state);
typedef void (*tuya_reset_cb_t)();

class TuyaHeaterDriver {
public:
    TuyaHeaterDriver();

    esp_err_t Init(int tx_pin, int rx_pin);

    // Call periodically (e.g. every 10 ms). Reads the UART, drives the
    // handshake, sends queued commands and runs the heartbeat timer.
    void Service();

    // Low-level frames (queued, also usable directly)
    void Poll();
    void Heartbeat();    // 0x00 keep-alive
    void QueryStatus();  // 0x08 request a full datapoint dump

    // Thermostat (heat/off) control
    void SetPower(bool on);   // DP1
    void SetHeat();           // DP1 on + DP4 Program, so the setpoint governs
    void SetTemp(int temp);   // DP2

    // Extras (not exposed over Matter)
    void SetEco(bool on);        // DP102
    void SetPowerful(bool on);   // on: DP101=1500W (auto Manual); off: DP4 back to Program

    void SetMode(uint8_t mode);  // DP4

    void SetStateCallback(tuya_state_change_cb_t cb);
    void SetResetCallback(tuya_reset_cb_t cb);

    heater_state_t GetState() const { return m_state; }
    bool IsReady() const { return m_init_state == TY_RUNNING; }

private:
    heater_state_t m_state;      // what Matter sees (updated optimistically by the setters)
    heater_state_t m_reported;   // last values reported by the MCU (poll task only)
    bool m_have_report;          // m_reported holds real values
    tuya_state_change_cb_t m_callback;
    tuya_reset_cb_t m_reset_callback;

    int m_uart_num;

    // --- MEMORY FIX: FIXED ARRAY INSTEAD OF VECTOR ---
    uint8_t rx_buffer[RX_BUF_SIZE];
    int rx_count;
    int64_t m_last_rx_time;

    // Outgoing commands. The front entry stays queued until the MCU answers it.
    QueueHandle_t m_cmd_queue;
    int m_expected_response;      // reply the front command waits for, -1 = none
    int64_t m_last_command_time;
    int m_retries;                // handshake retries of the front command
    std::atomic<int> m_pending[5];   // queued writes per DP (see PendingSlot)

    // Handshake state machine
    TuyaInitState m_init_state;
    int64_t m_hb_time;            // last heartbeat queued

    // Reset detection (power toggled 10x in quick succession)
    int64_t last_toggle_time;
    int toggle_count;

    void RunTimers();
    void ProcessCommandQueue();
    void Enqueue(uint8_t cmd, const uint8_t *payload, int len);
    void DropFront();
    void SendRaw(const tuya_command_t &c);
    void SendFrame(uint8_t cmd, const uint8_t *payload, int payload_len);
    void SendDatapoint(uint8_t dp_id, uint8_t type, uint32_t value, int len);
    bool IsUnchanged(const tuya_command_t &c) const;
    bool IsReplyToFront(uint8_t cmd, const uint8_t *payload, int plen);
    static int PendingSlot(uint8_t dp_id);
    bool IsPending(uint8_t dp_id) const;
    void SyncFromReported(uint8_t dp_id);

    void SetPowerLevel(uint8_t lvl);  // DP101
    void HandleFrame(uint8_t cmd, const uint8_t *payload, int plen);
    void ParseDatapoints(const uint8_t *payload, int plen);
    void NotifyStateChange();
};
