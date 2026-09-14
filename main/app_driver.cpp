#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include <esp_matter.h>
#include <app_priv.h>
#include <app/reporting/reporting.h>
#include <platform/CHIPDeviceLayer.h>
#include "iot_button.h"
#include "button_gpio.h"
#include <app/server/Server.h>
#include <app/server/CommissioningWindowManager.h>

#include "tuya_driver.h"

using namespace chip::app::Clusters;
using namespace chip::app::Clusters::Thermostat;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t thermostat_endpoint_id;
extern uint16_t eco_endpoint_id;
extern uint16_t powerful_endpoint_id;
static TuyaHeaterDriver heater;

// Global Temp for AAI (LocalTemperature is served via AttributeAccessInterface)
int16_t g_current_temp_int = 2000;

#define BUTTON_GPIO_PIN 23

// --- POLL TASK ---
static void tuya_poll_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Tuya Poll Task Started");
    vTaskDelay(pdMS_TO_TICKS(300));   // let the UART/MCU settle after boot
    while (1) {
        // Service() reads the UART, drives the product/work-mode/wifi-status
        // handshake, then runs heartbeat + status keep-alive in RUNNING.
        heater.Service();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- THREAD BRIDGE: push heater state into the Matter data model ---
struct AppEventData {
    heater_state_t state;
};

static void AppDriverUpdateTask(intptr_t context)
{
    AppEventData *data = (AppEventData *)context;
    if (!data) return;

    // LocalTemperature (served through the AAI; just refresh + notify subscribers)
    g_current_temp_int = data->state.current_temp * 100;
    MatterReportingAttributeChangeCallback(thermostat_endpoint_id, Thermostat::Id,
                                           Thermostat::Attributes::LocalTemperature::Id);

    // OccupiedHeatingSetpoint (report() = reflect without re-triggering the write callback)
    esp_matter_attr_val_t target_val = esp_matter_int16(data->state.target_temp * 100);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id,
                                  Thermostat::Attributes::OccupiedHeatingSetpoint::Id, &target_val);

    // SystemMode: powered -> Heat, otherwise Off
    uint8_t matter_mode = data->state.power ? (uint8_t)Thermostat::SystemModeEnum::kHeat
                                            : (uint8_t)Thermostat::SystemModeEnum::kOff;
    esp_matter_attr_val_t mode_val = esp_matter_enum8(matter_mode);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id,
                                  Thermostat::Attributes::SystemMode::Id, &mode_val);

    // ThermostatRunningState (no dedicated DP, so derive it)
    uint16_t running_state = 0; // Idle
    if (data->state.power && data->state.current_temp < (data->state.target_temp + 1)) {
        running_state = 1; // Heat stage on
    }
    esp_matter_attr_val_t run_val = esp_matter_bitmap16(running_state);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id,
                                  Thermostat::Attributes::ThermostatRunningState::Id, &run_val);

    // Eco mode (DP102)
    if (eco_endpoint_id != 0) {
        esp_matter_attr_val_t eco_val = esp_matter_bool(data->state.eco);
        esp_matter::attribute::report(eco_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &eco_val);
    }

    // Powerful = manual 1500 W (DP101 == 2)
    if (powerful_endpoint_id != 0) {
        bool powerful = (data->state.power_level == PWR_1500);
        esp_matter_attr_val_t pf_val = esp_matter_bool(powerful);
        esp_matter::attribute::report(powerful_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &pf_val);
    }

    free(data);
}

static void tuya_state_change_callback(const heater_state_t *state)
{
    if (thermostat_endpoint_id == 0) return;
    AppEventData *data = (AppEventData *)malloc(sizeof(AppEventData));
    if (data) {
        data->state = *state;
        chip::DeviceLayer::PlatformMgr().ScheduleWork(AppDriverUpdateTask, (intptr_t)data);
    }
}

// --- FACTORY RESET HANDLER (power toggled 10x) ---
static void tuya_reset_callback()
{
    ESP_LOGW(TAG, "Initiating Factory Reset due to Power Button sequence...");
    esp_matter::factory_reset();
}

// --- Matter -> heater (write path) ---
static esp_err_t app_driver_thermostat_set_value(void *handle, esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
        uint8_t mode = val->val.u8;
        if (mode == (uint8_t)Thermostat::SystemModeEnum::kOff) {
            heater.SetPower(false);
        } else {
            heater.SetHeat();   // power on + Program mode so the setpoint governs
        }
    }
    else if (attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {
        heater.SetTemp(val->val.i16 / 100);  // Matter centi-degC -> whole degC
    }
    return ESP_OK;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    if (endpoint_id == thermostat_endpoint_id && cluster_id == Thermostat::Id) {
        return app_driver_thermostat_set_value(driver_handle, val, attribute_id);
    }
    else if (endpoint_id == eco_endpoint_id && cluster_id == OnOff::Id &&
             attribute_id == OnOff::Attributes::OnOff::Id) {
        ESP_LOGI(TAG, "Matter: Eco %s", val->val.b ? "ON" : "OFF");
        heater.SetEco(val->val.b);
    }
    else if (endpoint_id == powerful_endpoint_id && cluster_id == OnOff::Id &&
             attribute_id == OnOff::Attributes::OnOff::Id) {
        ESP_LOGI(TAG, "Matter: Powerful %s", val->val.b ? "ON" : "OFF");
        heater.SetPowerful(val->val.b);
    }
    return ESP_OK;
}

esp_err_t app_driver_thermostat_set_defaults(uint16_t endpoint_id) { return ESP_OK; }

app_driver_handle_t app_driver_thermostat_init()
{
    heater.Init(TUYA_TX_PIN, TUYA_RX_PIN);
    heater.SetStateCallback(tuya_state_change_callback);
    heater.SetResetCallback(tuya_reset_callback);

    xTaskCreate(tuya_poll_task, "tuya_poll", 4096, NULL, 5, NULL);
    return (app_driver_handle_t)1;
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button: Commissioning Window");
    chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (!commissionMgr.IsCommissioningWindowOpen()) {
        commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                                   chip::CommissioningWindowAdvertisement::kDnssdOnly);
    }
}

app_driver_handle_t app_driver_button_init()
{
    button_config_t btn_cfg = {0};
    button_gpio_config_t btn_gpio_cfg = { .gpio_num = BUTTON_GPIO_PIN, .active_level = 0 };
    button_handle_t btn_handle = NULL;
    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn_handle);
    if (err == ESP_OK && btn_handle) {
        iot_button_register_cb(btn_handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
        return (app_driver_handle_t)btn_handle;
    }
    return NULL;
}
