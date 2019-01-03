/*
	Copyright (c) 2019 Evan Harvey <evanwork1234@gmail.com>

	Permission is hereby granted, free of charge, to any person
	obtaining a copy of this software and associated documentation
	files (the "Software"), to deal in the Software without
	restriction, including without limitation the rights to use,
	copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the
	Software is furnished to do so, subject to the following
	conditions:

	The above copyright notice and this permission notice shall be
	included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	OTHER DEALINGS IN THE SOFTWARE.
*/

#include "sdkconfig.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif

#include "BTSerialClient.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include <esp_log.h>

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif

#define QUEUE_SIZE 256
#define PRINT_MAX_BUF 128
#define SPP_DATA_LEN 20

static uint32_t _spp_server = 0;
static xQueueHandle _spp_queue = NULL;

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;

static esp_bd_addr_t peer_bd_addr;
static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;

static uint8_t spp_data[SPP_DATA_LEN];
static uint8_t print_buf[PRINT_MAX_BUF];

static String bt_name;
static String bt_target_name;


static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
        case ESP_SPP_INIT_EVT:              // SPP Inital
            esp_bt_dev_set_device_name(bt_name.c_str());
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
            Serial.println("ESP_SPP_INIT_EVT");
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:    // Discovery Complete
            Serial.println("ESP_SPP_DISCOVERY_COMP_EVT");
            if (param->disc_comp.status == ESP_SPP_SUCCESS) {
                esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
            }
            break; 
        case ESP_SPP_OPEN_EVT:              // Client Connection Open
            Serial.println("ESP_SPP_OPEN_EVT");
            _spp_server = param->srv_open.handle;
            break;
        case ESP_SPP_CLOSE_EVT:             // Client Connection Closed
            break;
        case ESP_SPP_START_EVT:             // Server Started
            break;
        case ESP_SPP_CL_INIT_EVT:           // Client Initiated a Connection
            break;
        case ESP_SPP_DATA_IND_EVT:          // Connection Received Data
            if (_spp_queue != NULL){
                for (int i = 0; i < param->data_ind.len; i++)
                    xQueueSend(_spp_queue, param->data_ind.data + i, (TickType_t)0);
            } else {
                log_e("SerialQueueBT ERROR");
            }
            break;
        case ESP_SPP_CONG_EVT:              // Connection Congestion Status Changed
            break;
        case ESP_SPP_WRITE_EVT:             // Write Operation Completed
            break;
        case ESP_SPP_SRV_OPEN_EVT:          // Server Connection Open
            break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            for (int i = 0; i < param->disc_res.num_prop; i++){
                if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                    && get_name_from_eir((uint8_t *)param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                    
                    Serial.println(peer_bdname);

                    if (!strcmp(peer_bdname, bt_target_name.c_str())) {
                        memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                        Serial.print((char)peer_bd_addr[0], HEX); Serial.print(":");
                        Serial.print((char)peer_bd_addr[1], HEX); Serial.print(":");
                        Serial.print((char)peer_bd_addr[2], HEX); Serial.print(":");
                        Serial.print((char)peer_bd_addr[3], HEX); Serial.print(":");
                        Serial.print((char)peer_bd_addr[4], HEX); Serial.print(":");
                        Serial.println((char)peer_bd_addr[5], HEX);

                        esp_spp_start_discovery(peer_bd_addr);
                        esp_bt_gap_cancel_discovery();
                    }
                }
            }
            Serial.println("ESP_BT_GAP_DISC_RES_EVT");
            break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            break; 
        case ESP_BT_GAP_RMT_SRVCS_EVT:
            break;
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            break;
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            break;
    }
}

static bool _init_bt()
{
    esp_err_t ret;

    // Bluetooth Controller Inital
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    // Bluedroid Inital
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    // Register Callback
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    // SPP Init
    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    // Create Queue
    _spp_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
    if (_spp_queue == NULL){
        log_e("%s Queue creation error\n", __func__);
        return false;
    }
	
    return true;
}

static bool _stop_bt()
{
    if (btStarted()){
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        btStop();
    }
    return true;
}

/*
 * Serial Bluetooth Arduino
 *
 * */

BTSerialClient::BTSerialClient()
{
    Serial.begin(115200);
}

BTSerialClient::~BTSerialClient(void)
{
   _stop_bt();
}

bool BTSerialClient::begin(String localName, String connectName)
{
    if (localName.length()){
        bt_name = localName;
    }

    if (connectName.length()) {
        bt_target_name = connectName;
    }

    return _init_bt();
}

int BTSerialClient::available(void)
{
    if (!_spp_server || _spp_queue == NULL){
        return 0;
    }
    return uxQueueMessagesWaiting(_spp_queue);
}

int BTSerialClient::peek(void)
{
    if (available()){
        if (!_spp_server || _spp_queue == NULL){
            return 0;
        }

        uint8_t c;
        if (xQueuePeek(_spp_queue, &c, 0)){
            return c;
        }
    }
    return -1;
}

bool BTSerialClient::hasClient(void)
{
    if (_spp_server)
        return true;
	
    return false;
}

int BTSerialClient::read(void)
{
    if (available()){
        if (!_spp_server || _spp_queue == NULL){
            return 0;
        }

        uint8_t c;
        if (xQueueReceive(_spp_queue, &c, 0)){
            return c;
        }
    }
    return 0;
}

size_t BTSerialClient::write(uint8_t c)
{
    if (!_spp_server){
        return 0;
    }

    uint8_t buffer[1];
    buffer[0] = c;
    esp_err_t err = esp_spp_write(_spp_server, 1, buffer);
    return (err == ESP_OK) ? 1 : 0;
}

size_t BTSerialClient::write(const uint8_t *buffer, size_t size)
{
    if (!_spp_server){
        return 0;
    }

    esp_err_t err = esp_spp_write(_spp_server, size, (uint8_t *)buffer);
    return (err == ESP_OK) ? size : 0;
}

size_t BTSerialClient::print(const char *str) {
    if (!_spp_server){
        return 0;
    }

    size_t len = strlen(str);

    for (int i = 0; i < len; i++) {
        print_buf[i] = str[i];
    }

    esp_err_t err = esp_spp_write(_spp_server, len, (uint8_t *)print_buf);
    return (err == ESP_OK) ? len : 0;
}

size_t BTSerialClient::println(const char *str) {
    if (!_spp_server){
        return 0;
    }

    size_t len = strlen(str);

    for (int i = 0; i < len; i++) {
        print_buf[i] = str[i];
    }

    print_buf[len] = '\r';
    print_buf[len + 1] = '\n';

    esp_err_t err = esp_spp_write(_spp_server, len + 2, (uint8_t *)print_buf);
    return (err == ESP_OK) ? len : 0;
}

void BTSerialClient::flush()
{
    if (_spp_server){
        int qsize = available();
        uint8_t buffer[qsize];
        esp_spp_write(_spp_server, qsize, buffer);
    }
}

void BTSerialClient::end()
{
    _stop_bt();
}

#endif
