#include "tuya_driver.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <inttypes.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "TUYA_DRIVER";

#define BAUD_RATE 9600

TuyaHeaterDriver::TuyaHeaterDriver() {
    m_callback = nullptr;
    m_reset_callback = nullptr;
    m_uart_num = UART_NUM_1;

    m_state.power = false;
    m_state.target_temp = 22;
    m_state.current_temp = 20;
    m_state.mode = MODE_PROGRAM;
    m_state.power_level = PWR_NONE;
    m_state.eco = false;
    m_reported = m_state;
    m_have_report = false;

    rx_count = 0;
    memset(rx_buffer, 0, RX_BUF_SIZE);
    m_last_rx_time = 0;

    m_cmd_queue = nullptr;
    m_expected_response = -1;
    m_last_command_time = 0;
    m_retries = 0;
    for (auto &p : m_pending) p = 0;

    m_init_state = TY_HEARTBEAT;
    m_hb_time = 0;

    last_toggle_time = 0;
    toggle_count = 0;
}

esp_err_t TuyaHeaterDriver::Init(int tx_pin, int rx_pin) {
    m_cmd_queue = xQueueCreate(TUYA_QUEUE_LEN, sizeof(tuya_command_t));
    if (!m_cmd_queue) return ESP_ERR_NO_MEM;

    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install((uart_port_t)m_uart_num, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;

    err = uart_param_config((uart_port_t)m_uart_num, &uart_config);
    if (err != ESP_OK) return err;

    // tx_pin = ESP TX -> heater MCU RX ;  rx_pin = ESP RX <- heater MCU TX
    err = uart_set_pin((uart_port_t)m_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return err;
}

// --- Command queue (mirrors ESPHome's process_command_queue_) -------------

void TuyaHeaterDriver::Enqueue(uint8_t cmd, const uint8_t *payload, int len) {
    if (!m_cmd_queue || len > TUYA_MAX_PAYLOAD) return;

    tuya_command_t c = {};
    c.cmd = cmd;
    c.len = len;
    if (len > 0 && payload) memcpy(c.payload, payload, len);

    // Count the write as pending before the poll task can see (and finish) it
    int slot = (cmd == TUYA_CMD_SET_DP) ? PendingSlot(c.payload[0]) : -1;
    if (slot >= 0) m_pending[slot]++;
    if (xQueueSend(m_cmd_queue, &c, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full, dropping command 0x%02X", cmd);
        if (slot >= 0) m_pending[slot]--;
    }
}

void TuyaHeaterDriver::DropFront() {
    tuya_command_t c;
    if (xQueueReceive(m_cmd_queue, &c, 0) != pdTRUE || c.cmd != TUYA_CMD_SET_DP) return;

    // Last write for this DP is done (answered, skipped or timed out): show the MCU's value
    int slot = PendingSlot(c.payload[0]);
    if (slot >= 0 && --m_pending[slot] == 0) SyncFromReported(c.payload[0]);
}

// Only the DPs Matter writes are tracked
int TuyaHeaterDriver::PendingSlot(uint8_t dp_id) {
    switch (dp_id) {
    case DP_POWER:     return 0;
    case DP_SET_TEMP:  return 1;
    case DP_MODE:      return 2;
    case DP_POWER_LVL: return 3;
    case DP_ECO:       return 4;
    default:           return -1;
    }
}

bool TuyaHeaterDriver::IsPending(uint8_t dp_id) const {
    int slot = PendingSlot(dp_id);
    return slot >= 0 && m_pending[slot] > 0;
}

// While writes to a DP are queued, Matter keeps showing the value it asked for and
// reports for that DP only update m_reported. Once they are done, take the MCU's value.
void TuyaHeaterDriver::SyncFromReported(uint8_t dp_id) {
    if (!m_have_report || IsPending(dp_id)) return;

    bool changed = false;
    switch (dp_id) {
    case DP_POWER:     changed = m_state.power != m_reported.power;             m_state.power = m_reported.power; break;
    case DP_SET_TEMP:  changed = m_state.target_temp != m_reported.target_temp; m_state.target_temp = m_reported.target_temp; break;
    case DP_MODE:      changed = m_state.mode != m_reported.mode;               m_state.mode = m_reported.mode; break;
    case DP_POWER_LVL: changed = m_state.power_level != m_reported.power_level; m_state.power_level = m_reported.power_level; break;
    case DP_ECO:       changed = m_state.eco != m_reported.eco;                 m_state.eco = m_reported.eco; break;
    default: break;
    }
    if (changed) NotifyStateChange();
}

// ESPHome takes any status report as the reply to a write. We wait for the report
// that carries the written value: right after a write the MCU also sends a full dump
// built before it handled our next write, and that must not count as its reply.
bool TuyaHeaterDriver::IsReplyToFront(uint8_t cmd, const uint8_t *payload, int plen) {
    tuya_command_t front;
    if (cmd != TUYA_CMD_REPORT_DP || xQueuePeek(m_cmd_queue, &front, 0) != pdTRUE ||
        front.cmd != TUYA_CMD_SET_DP) {
        return true;
    }

    int vlen = front.len - 4;
    int pos = 0;
    while (pos + 4 <= plen) {
        uint16_t dlen = (payload[pos+2] << 8) | payload[pos+3];
        if (pos + 4 + dlen > plen) break;
        if (payload[pos] == front.payload[0] && dlen == vlen &&
            memcmp(&payload[pos+4], &front.payload[4], vlen) == 0) {
            return true;
        }
        pos += 4 + dlen;
    }
    return false;
}

void TuyaHeaterDriver::ProcessCommandQueue() {
    int64_t now = esp_timer_get_time();
    int64_t since = now - m_last_command_time;

    // No reply in time: retry during the handshake, give up once running
    if (m_expected_response >= 0 && since > TUYA_RECEIVE_TIMEOUT_US) {
        uint8_t missing = (uint8_t)m_expected_response;
        m_expected_response = -1;
        if (m_init_state == TY_RUNNING) {
            DropFront();
        } else if (++m_retries >= TUYA_MAX_RETRIES) {
            DropFront();
            m_retries = 0;
            // ESPHome stops the handshake here. We carry on as if the MCU had sent
            // an empty reply, except for the heartbeat: nothing works until it answers.
            if (missing != TUYA_CMD_HEARTBEAT) {
                ESP_LOGW(TAG, "No reply 0x%02X during handshake; proceeding", missing);
                HandleFrame(missing, nullptr, 0);
            }
        }
        // otherwise the command stays at the front and is sent again below
    }

    tuya_command_t c;
    if (since > TUYA_COMMAND_DELAY_US && rx_count == 0 && m_expected_response < 0 &&
        xQueuePeek(m_cmd_queue, &c, 0) == pdTRUE) {
        if (IsUnchanged(c)) {
            ESP_LOGI(TAG, "Not sending unchanged value for datapoint %u", c.payload[0]);
            DropFront();
            return;
        }
        SendRaw(c);
        if (m_expected_response < 0) DropFront();
    }
}

// ESPHome doesn't send a write that matches what the MCU last reported. We check
// when the write is about to go out rather than when it is queued, so the MCU has
// already answered the writes before it (ESPHome checks at queue time, which loses
// e.g. a quick Off -> Heat).
bool TuyaHeaterDriver::IsUnchanged(const tuya_command_t &c) const {
    if (c.cmd != TUYA_CMD_SET_DP || !m_have_report) return false;

    uint32_t value = 0;
    for (int i = 4; i < c.len; i++) value = (value << 8) | c.payload[i];

    switch (c.payload[0]) {
    case DP_POWER:     return value == (uint32_t)m_reported.power;
    case DP_SET_TEMP:  return value == (uint32_t)m_reported.target_temp;
    case DP_MODE:      return value == m_reported.mode;
    case DP_POWER_LVL: return value == m_reported.power_level;
    case DP_ECO:       return value == (uint32_t)m_reported.eco;
    default:           return false;
    }
}

void TuyaHeaterDriver::SendRaw(const tuya_command_t &c) {
    m_last_command_time = esp_timer_get_time();

    switch (c.cmd) {
    case TUYA_CMD_HEARTBEAT:
    case TUYA_CMD_PRODUCT:
    case TUYA_CMD_WORK_MODE:
        m_expected_response = c.cmd;
        break;
    case TUYA_CMD_WIFI_STATE:
        // ESPHome doesn't wait for this ack; we do, so a lost ack gets retried
        m_expected_response = TUYA_CMD_WIFI_STATE;
        break;
    case TUYA_CMD_SET_DP:
    case TUYA_CMD_QUERY_DP:
        m_expected_response = TUYA_CMD_REPORT_DP;
        break;
    default:
        m_expected_response = -1;
        break;
    }

    SendFrame(c.cmd, c.payload, c.len);
}

// --- Frame TX helpers ----------------------------------------------------

void TuyaHeaterDriver::SendFrame(uint8_t cmd, const uint8_t *payload, int payload_len) {
    uint8_t frame[64];
    int idx = 0;

    frame[idx++] = TUYA_HEADER_0;
    frame[idx++] = TUYA_HEADER_1;
    frame[idx++] = 0x00; // version (module-originated)
    frame[idx++] = cmd;
    frame[idx++] = (payload_len >> 8) & 0xFF;
    frame[idx++] = payload_len & 0xFF;
    if (payload_len > 0 && payload) {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }

    uint8_t cs = 0;
    for (int i = 0; i < idx; i++) cs += frame[i];
    frame[idx++] = cs;

    uart_write_bytes((uart_port_t)m_uart_num, (const char*)frame, idx);
}

void TuyaHeaterDriver::SendDatapoint(uint8_t dp_id, uint8_t type, uint32_t value, int len) {
    ESP_LOGI(TAG, "Setting datapoint %u to %" PRIu32, dp_id, value);

    // DP control payload: DP_ID(1) Type(1) Len(2) Value(len, big-endian)
    uint8_t payload[8];
    int idx = 0;
    payload[idx++] = dp_id;
    payload[idx++] = type;
    payload[idx++] = (len >> 8) & 0xFF;
    payload[idx++] = len & 0xFF;
    for (int i = len - 1; i >= 0; i--) payload[idx++] = (value >> (8 * i)) & 0xFF;
    Enqueue(TUYA_CMD_SET_DP, payload, idx);
}

void TuyaHeaterDriver::Heartbeat()   { Enqueue(TUYA_CMD_HEARTBEAT, nullptr, 0); }
void TuyaHeaterDriver::QueryStatus() { Enqueue(TUYA_CMD_QUERY_DP, nullptr, 0); }

// --- Heartbeat timer -----------------------------------------------------

void TuyaHeaterDriver::RunTimers() {
    int64_t now = esp_timer_get_time();

    if (m_init_state == TY_HEARTBEAT) {
        // Ping until the MCU answers (one heartbeat in the queue at a time)
        if (now - m_hb_time > TUYA_PING_US && uxQueueMessagesWaiting(m_cmd_queue) == 0) {
            Heartbeat();
            m_hb_time = now;
        }
    } else if (now - m_hb_time > TUYA_HEARTBEAT_US) {
        Heartbeat();
        m_hb_time = now;
    }
}

void TuyaHeaterDriver::Service() {
    if (!m_cmd_queue) return;   // Init() failed
    Poll();
    RunTimers();
    ProcessCommandQueue();
}

// --- Frame RX ------------------------------------------------------------

void TuyaHeaterDriver::Poll() {
    int64_t now = esp_timer_get_time();

    // Drop a partial frame after 300 ms of silence (ESPHome's RECEIVE_TIMEOUT)
    if (rx_count > 0 && now - m_last_rx_time > TUYA_RECEIVE_TIMEOUT_US) {
        rx_count = 0;
    }

    int remaining_space = RX_BUF_SIZE - rx_count;
    if (remaining_space <= 0) {
        rx_count = 0;
        remaining_space = RX_BUF_SIZE;
    }

    size_t avail = 0;
    uart_get_buffered_data_len((uart_port_t)m_uart_num, &avail);
    if (avail == 0) return;
    if ((int)avail > remaining_space) avail = remaining_space;

    int len = uart_read_bytes((uart_port_t)m_uart_num, &rx_buffer[rx_count], avail, 0);
    if (len <= 0) return;

    rx_count += len;
    m_last_rx_time = now;

    while (rx_count > 0) {

        // 1. Resync on the header
        if (rx_buffer[0] != TUYA_HEADER_0 || (rx_count > 1 && rx_buffer[1] != TUYA_HEADER_1)) {
            memmove(rx_buffer, &rx_buffer[1], rx_count - 1);
            rx_count--;
            continue;
        }
        if (rx_count < 7) {
            break; // wait for the rest of the header
        }

        // 2. Check payload length
        uint16_t payload_len = (rx_buffer[4] << 8) | rx_buffer[5];
        int total_len = 6 + payload_len + 1;

        if (total_len > RX_BUF_SIZE) {       // garbage length, drop the header byte
            memmove(rx_buffer, &rx_buffer[1], rx_count - 1);
            rx_count--;
            continue;
        }
        if (rx_count < total_len) {
            break; // wait for the rest of the packet
        }

        // 3. Verify checksum
        uint8_t calc_cs = 0;
        for (int i = 0; i < total_len - 1; i++) calc_cs += rx_buffer[i];

        if (calc_cs == rx_buffer[total_len - 1]) {
            HandleFrame(rx_buffer[3], &rx_buffer[6], payload_len);
        } else {
            ESP_LOGW(TAG, "Invalid checksum %02X != %02X", rx_buffer[total_len - 1], calc_cs);
        }

        // 4. Remove processed packet
        int remaining = rx_count - total_len;
        if (remaining > 0) {
            memmove(rx_buffer, &rx_buffer[total_len], remaining);
        }
        rx_count = remaining;
    }
}

// Mirrors ESPHome's Tuya::handle_command_ for the commands this heater uses.
void TuyaHeaterDriver::HandleFrame(uint8_t cmd, const uint8_t *payload, int plen) {
    // The reply the front command is waiting for? It is retired after the frame is handled.
    bool reply = m_expected_response >= 0 && cmd == m_expected_response &&
                 IsReplyToFront(cmd, payload, plen);

    switch (cmd) {
    case TUYA_CMD_HEARTBEAT:
        if (plen >= 1 && payload[0] == 0x00 && m_init_state != TY_HEARTBEAT) {
            ESP_LOGW(TAG, "MCU restarted; repeating handshake");
            m_init_state = TY_HEARTBEAT;
        }
        if (m_init_state == TY_HEARTBEAT) {
            ESP_LOGI(TAG, "MCU alive");
            m_init_state = TY_PRODUCT;
            Enqueue(TUYA_CMD_PRODUCT, nullptr, 0);
        }
        break;

    case TUYA_CMD_PRODUCT:
        if (plen > 0) ESP_LOGI(TAG, "Product info: %.*s", plen, (const char *)payload);
        if (m_init_state == TY_PRODUCT) {
            m_init_state = TY_CONF;
            Enqueue(TUYA_CMD_WORK_MODE, nullptr, 0);
        }
        break;

    case TUYA_CMD_WORK_MODE:
        if (m_init_state == TY_CONF) {
            if (plen >= 2) {
                // MCU drives its network LED / reset button through GPIOs: no status report
                ESP_LOGW(TAG, "MCU reports status pin %u, reset pin %u; skipping network status",
                         payload[0], payload[1]);
                m_init_state = TY_DATAPOINT;
                Enqueue(TUYA_CMD_QUERY_DP, nullptr, 0);
            } else {
                ESP_LOGI(TAG, "Work mode: module reports network status over serial");
                m_init_state = TY_WIFI;
                uint8_t s = TUYA_WIFI_CONNECTED;
                Enqueue(TUYA_CMD_WIFI_STATE, &s, 1);
            }
        }
        break;

    case TUYA_CMD_WIFI_STATE:
        // MCU acknowledged our network status
        if (m_init_state == TY_WIFI) {
            m_init_state = TY_DATAPOINT;
            Enqueue(TUYA_CMD_QUERY_DP, nullptr, 0);
        }
        break;

    case TUYA_CMD_REPORT_DP:
        if (m_init_state == TY_DATAPOINT) {
            m_init_state = TY_RUNNING;
            ESP_LOGI(TAG, "Tuya handshake complete -> RUNNING");
        }
        ParseDatapoints(payload, plen);
        break;

    case TUYA_CMD_GET_TIME:
        // Not answered: the heater runs fine without a clock.
        break;

    default:
        break;
    }

    if (reply) {
        m_expected_response = -1;
        DropFront();
        m_retries = 0;
    }
}

void TuyaHeaterDriver::ParseDatapoints(const uint8_t *payload, int plen) {
    int pos = 0;
    bool first = !m_have_report;
    bool changed = false;

    while (pos + 4 <= plen) {
        uint8_t dp_id = payload[pos];
        uint16_t data_len = (payload[pos+2] << 8) | payload[pos+3];
        int val_idx = pos + 4;

        if (val_idx + data_len > plen) break;

        int32_t val = 0;
        if (data_len == 1) val = payload[val_idx];
        else if (data_len == 4) {
            val = (payload[val_idx] << 24) | (payload[val_idx+1] << 16) |
                  (payload[val_idx+2] << 8) | payload[val_idx+3];
        }
        m_have_report = true;
        // A queued Matter write for this DP wins until the MCU has answered it
        bool pending = IsPending(dp_id);

        if (dp_id == DP_POWER) {
            bool new_power = (val == 1);
            m_reported.power = new_power;
            if (!pending && m_state.power != new_power) {
                // --- FACTORY RESET: power toggled on the heater 10x within 3s windows ---
                int64_t now = esp_timer_get_time();
                if (now - last_toggle_time < 3000000) {
                    toggle_count++;
                } else {
                    toggle_count = 1;
                }
                last_toggle_time = now;
                ESP_LOGI(TAG, "Power Toggle Detected! Count: %d/10", toggle_count);
                if (toggle_count >= 10) {
                    ESP_LOGW(TAG, "FACTORY RESET SEQUENCE DETECTED!");
                    if (m_reset_callback) m_reset_callback();
                    toggle_count = 0;
                }
                m_state.power = new_power;
                changed = true;
            }
        }
        else if (dp_id == DP_SET_TEMP) {
            m_reported.target_temp = val;
            if (!pending && m_state.target_temp != val) { m_state.target_temp = val; changed = true; }
        }
        else if (dp_id == DP_CUR_TEMP) {
            m_reported.current_temp = val;
            if (m_state.current_temp != val) { m_state.current_temp = val; changed = true; }
        }
        else if (dp_id == DP_MODE) {
            m_reported.mode = (uint8_t)val;
            if (!pending && m_state.mode != (uint8_t)val) { m_state.mode = (uint8_t)val; changed = true; }
        }
        else if (dp_id == DP_POWER_LVL) {
            m_reported.power_level = (uint8_t)val;
            if (!pending && m_state.power_level != (uint8_t)val) { m_state.power_level = (uint8_t)val; changed = true; }
        }
        else if (dp_id == DP_ECO) {
            bool new_eco = (val == 1);
            m_reported.eco = new_eco;
            if (!pending && m_state.eco != new_eco) { m_state.eco = new_eco; changed = true; }
        }
        // DP_FAULT (12) is read-only status; ignored here.

        pos += 4 + data_len;
    }

    // The first report always syncs Matter, even if it matches the defaults
    if (changed || (first && m_have_report)) NotifyStateChange();
}

void TuyaHeaterDriver::NotifyStateChange() {
    if (m_callback) m_callback(&m_state);
}

// --- Setters -------------------------------------------------------------
// Each write is queued; the poll task sends it once the MCU has answered the
// previous command, the same way ESPHome paces its writes.

void TuyaHeaterDriver::SetPower(bool on) {
    SendDatapoint(DP_POWER, 0x01, on ? 1 : 0, 1);
    m_state.power = on;
    NotifyStateChange();
}

void TuyaHeaterDriver::SetHeat() {
    // "Heat" = powered on AND in Program/thermostat mode, so the setpoint (DP2)
    // governs instead of a fixed manual power level. Same as ESPHome's tuya
    // climate with active_state {datapoint: 4, heating_value: 1}: DP1, then DP4.
    SetPower(true);
    SetMode(MODE_PROGRAM);
}

void TuyaHeaterDriver::SetTemp(int temp) {
    SendDatapoint(DP_SET_TEMP, 0x02, (uint32_t)temp, 4);
    m_state.target_temp = temp;
    NotifyStateChange();
}

void TuyaHeaterDriver::SetMode(uint8_t mode) {
    SendDatapoint(DP_MODE, 0x04, mode, 1);
    m_state.mode = mode;
    NotifyStateChange();
}

void TuyaHeaterDriver::SetPowerLevel(uint8_t lvl) {
    SendDatapoint(DP_POWER_LVL, 0x04, lvl, 1);
    m_state.power_level = lvl;
    NotifyStateChange();
}

void TuyaHeaterDriver::SetPowerful(bool on) {
    if (on) {
        SetPowerLevel(PWR_1500);   // 1500 W manual; heater auto-switches DP4 to Manual
    } else {
        SetMode(MODE_PROGRAM);     // back to thermostat behaviour
    }
}

void TuyaHeaterDriver::SetEco(bool on) {
    SendDatapoint(DP_ECO, 0x01, on ? 1 : 0, 1);
    m_state.eco = on;
    NotifyStateChange();
}

void TuyaHeaterDriver::SetStateCallback(tuya_state_change_cb_t cb) {
    m_callback = cb;
}
void TuyaHeaterDriver::SetResetCallback(tuya_reset_cb_t cb) {
    m_reset_callback = cb;
}
