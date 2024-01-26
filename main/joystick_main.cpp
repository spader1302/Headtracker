// Copyright 2019 Mark Wolfe.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     https://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "driver/adc.h"

#include "joystick_buttons.h"

#define HID_JOYSTICK_TAG "HID_JOYSTICK"

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

static bool connected = false;

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "Dingo Gamepad"

static constexpr uint8_t BNO055_I2C_ADDR        = 0x29;

#include "bno055.hpp"
#include "tracker.hpp"

BNOSensor bno(BNO055_I2C_ADDR);
Tracker tracker(&bno);

static constexpr gpio_num_t GPIO_POWER_PIN = GPIO_NUM_18;
static constexpr gpio_num_t GPIO_BUTTON_PIN = GPIO_NUM_10;
static constexpr gpio_num_t GPIO_LED_PIN = GPIO_NUM_9;
static constexpr gpio_num_t GPIO_BNO_INTERRUPT_PIN = GPIO_NUM_3;

static constexpr uint64_t GPIO_OUTPUT_PIN_SEL = ((1ULL<<GPIO_POWER_PIN) | (1ULL<<GPIO_LED_PIN));
static constexpr uint64_t GPIO_INPUT_PIN_SEL = ((1ULL<<GPIO_BUTTON_PIN) | (1ULL<<GPIO_BNO_INTERRUPT_PIN));

static QueueHandle_t gpio_evt_queue = NULL;

#define ESP_INTR_FLAG_DEFAULT 0

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    }
    case ESP_BAT_EVENT_REG: {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
        ESP_LOGI(HID_JOYSTICK_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        connected = true;
        hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
        sec_conn = false;
        connected = false;
        ESP_LOGI(HID_JOYSTICK_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
        ESP_LOGI(HID_JOYSTICK_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(HID_JOYSTICK_TAG, param->vendor_write.data, param->vendor_write.length);
    }
    default:
        break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            ESP_LOGD(HID_JOYSTICK_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_JOYSTICK_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_JOYSTICK_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_JOYSTICK_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_JOYSTICK_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

static void write_joystick_task(void *pvParameter)
{
    uint16_t js1x,js1y,js1z;
    uint64_t current_sum;
    uint64_t last_sum = 0;

    for (;;)
    {
        js1x = tracker.getX();
        js1y = tracker.getY();
        js1z = tracker.getZ();

        current_sum = (static_cast<uint64_t>(js1z)<<32) + (static_cast<uint64_t>(js1y)<<16) + js1x;

        if (current_sum != last_sum)
        {
            ESP_LOGE(HID_JOYSTICK_TAG, "X=%d Y=%d Z=%d", js1x, js1y, js1z);
            esp_hidd_send_joystick_value(hid_conn_id, js1x, js1y, js1z);

            last_sum = current_sum;
        }
    }
}

static esp_err_t write_joystick_init(void)
{

    xTaskCreate(write_joystick_task, "write_joystick_task", 2048, NULL, 4, NULL);

    return ESP_OK;
}

static void gpio_task_input(void* arg)
{
    gpio_num_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            switch (io_num) {
                case GPIO_BUTTON_PIN:
                    // turn off power
                    gpio_set_level(GPIO_POWER_PIN, 0);
                    break;
                case GPIO_BNO_INTERRUPT_PIN:
                    //write_joystick_task();
                    break;
                default:
                    break;
            }
        }
    }
}

static void confGPIO()
{
    gpio_config_t gpio_conf = {};

    // config output pins
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    // config input pins
    gpio_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_input, "gpio_task_input", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_BUTTON_PIN, gpio_isr_handler, (void*) GPIO_BUTTON_PIN);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_BNO_INTERRUPT_PIN, gpio_isr_handler, (void*) GPIO_BNO_INTERRUPT_PIN);
}

extern "C" void app_main()
{
    // config GPIO Pins
    confGPIO();
    // switch on power
    gpio_set_level(GPIO_POWER_PIN, 1);
    // turn off LED
    gpio_set_level(GPIO_LED_PIN, 0);

    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // start up i2c interface to bno, takes ~3s
    if ((ret = bno.begin()) != ESP_OK)
    {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s i2c init failed\n", __func__);
        ESP_LOGE(HID_JOYSTICK_TAG, "%s", esp_err_to_name(ret));
        return;
    }
    
    // set bno euler angle unit to degree    
    while ((ret = bno.setUnitMode(EUL_DEG)) != ESP_OK)
    {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s bno set unit failed, retrying..\n", __func__);
        ESP_LOGE(HID_JOYSTICK_TAG, "%s", esp_err_to_name(ret));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    /*
    //configure bno interrupts
    while ((ret = bno.configureInterrupt()) != ESP_OK)
    {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s bno configure interrupt failed, retrying..\n", __func__);
        ESP_LOGE(HID_JOYSTICK_TAG, "%s", esp_err_to_name(ret));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    */

    // set operating mode to NDOF, takes ~20ms
    if ((ret = bno.setOpMode(NDOF)) != ESP_OK)
    {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s operating mode set failed\n", __func__);
        ESP_LOGE(HID_JOYSTICK_TAG, "%s", esp_err_to_name(ret));
    }

    // start sending joystick values
    if((ret = write_joystick_init()) != ESP_OK) 
    {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init write joystick failed\n", __func__);
    }

    // turn on LED
    gpio_set_level(GPIO_LED_PIN, 1);

}
