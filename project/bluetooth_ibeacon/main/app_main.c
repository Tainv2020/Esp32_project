/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt_hci_common.h"

static const char *tag = "BLE_ADV";

static uint8_t hci_cmd_buf[128];

/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
    printf("controller rcv pkt ready\n");
}

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};
static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
    uint16_t adv_intv_min = 256; // 160ms
    uint16_t adv_intv_max = 256; // 160ms
    uint8_t adv_type = 0; // connectable undirected advertising (ADV_IND)
    uint8_t own_addr_type = 0; // Public Device Address
    uint8_t peer_addr_type = 0; // Public Device Address
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; // Process All Conn and Scan

    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                  adv_intv_min,
                  adv_intv_max,
                  adv_type,
                  own_addr_type,
                  peer_addr_type,
                  peer_addr,
                  adv_chn_map,
                  adv_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_data(void)
{
    uint8_t adv_data[31];
    uint8_t adv_data_len;

    adv_data[0] = 0x02;      // Len
    adv_data[1] = 0x01;   // Type Flags
    adv_data[2] = 0x06;   // BR_EDR_NOT_SUPPORTED 0x04
    adv_data[3] = 0x1A;     // Len
    adv_data[4] = 0xFF;   // Type
    adv_data[5] = 0x4C;   // Company 2 -> fake Apple 0x004C LSB
    adv_data[6] = 0x00;   // Company 1 MSB
    adv_data[7] = 0x02;   // Type Beacon
    adv_data[8] = 0x15;     // Length of Beacon Data
    adv_data[9] = 0x11;   // UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)
    adv_data[10] = 0x22;  // UUID 2
    adv_data[11] = 0x33;  // UUID 3
    adv_data[12] = 0x53;  // UUID 4
    adv_data[13] = 0x32;  // UUID 5
    adv_data[14] = 0x6C;  // UUID 6
    adv_data[15] = 0x44;  // UUID 7
    adv_data[16] = 0x23;  // UUID 8
    adv_data[17] = 0xBB;  // UUID 9
    adv_data[18] = 0x89;  // UUID 10
    adv_data[19] = 0x65;  // UUID 11
    adv_data[20] = 0x87;  // UUID 12
    adv_data[21] = 0xAA;  // UUID 13
    adv_data[22] = 0xEE;  // UUID 14
    adv_data[23] = 0xEE;  // UUID 15
    adv_data[24] = 0x07;  // UUID 16
    adv_data[25] = 0x00;  // Major 1 Value
    adv_data[26] = 0x20;  // Major 2 Value
    adv_data[27] = 0x21;  // Minor 1 Value
    adv_data[28] = 0x22;  // Minor 2 Value
    adv_data[29] = 0xA0;  // Beacons TX power

    adv_data_len = 30;

    printf("iBeacon adv_data [%d]=",adv_data_len);
    for (int i=0; i<adv_data_len; i++) {
        printf("%02x",adv_data[i]);
    }
    printf("\n");

    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

/*
 * @brief: send HCI commands to perform BLE advertising;
 */
void bleAdvtTask(void *pvParameters)
{
    int cmd_cnt = 0;
    bool send_avail = false;
    esp_vhci_host_register_callback(&vhci_host_cb);
    printf("BLE advt task start\n");
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        send_avail = esp_vhci_host_check_send_available();
        if (send_avail) {
            switch (cmd_cnt) {
            case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
            case 1: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
            case 2: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;
            case 3: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;
            }
        }
        printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d\n", send_avail, cmd_cnt);
    }
}

void app_main(void)
{
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGI(tag, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    /*
     * If call mem release here, also work. Input ESP_BT_MODE_CLASSIC_BT, the function will
     * release the memory of classic bt mode.
     * esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
     *
     */

    /*
     * If call mem release here, also work. Input ESP_BT_MODE_BTDM, the function will calculate
     * that the BLE mode is already used, so it will release of only classic bt mode.
     * esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
     */

    xTaskCreatePinnedToCore(&bleAdvtTask, "bleAdvtTask", 2048, NULL, 5, NULL, 0);
}
