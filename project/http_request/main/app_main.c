/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

/* My library */
#include "lib_wifi.h"
#include "lib_http_request.h"

static const uint8_t app_wifi_name[] = "Hoa Thieu";
static const uint8_t app_wifi_pass[] = "0398710415";

void app_http_request_task( void * pvParameters )
{
    lib_http_request_task();
}

void app_main(void)
{
    /* Init wifi */
    lib_wifi_init_sta(app_wifi_name, app_wifi_pass);

    /* Run http request */
    xTaskCreate(&app_http_request_task, "app_http_request_task", 4096, NULL, 5, NULL);
}
