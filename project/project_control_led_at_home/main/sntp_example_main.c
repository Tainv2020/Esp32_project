/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "lib_gpio.h"

static const char *TAG = "example";

/* Time to turn on and turn off Led */
#define JAN_MONTH       0
#define OCT_MONTH       9
#define NOV_MONTH       10
#define DEC_MONTH       11

#define HOUR_WINTER_ON  17 /* 6 pm */
#define MIN_WINTER_ON   0
#define SEC_WINTER_ON   0
#define HOUR_WINTER_OFF 6 /* 6 am */
#define MIN_WINTER_OFF  0
#define SEC_WINTER_OFF  0

#define HOUR_SUMMER_ON  19 /* 7 pm */
#define MIN_SUMMER_ON   0
#define SEC_SUMMER_ON   0
#define HOUR_SUMMER_OFF 5 /* 5 am */
#define MIN_SUMMER_OFF  0
#define SEC_SUMMER_OFF  0

/* Led and Button */
#define WIFI_PIN   2
#define RELAY_PIN  22
#define BUTTON_PIN 23

/* Output State */
#define ON  0
#define OFF 1

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;
static bool TimeStt = false;
static bool BtnStt = false;


/*  FUNCTION */
static void obtain_time(void);
static void initialize_sntp(void);
static void gpio_setup(void);
static void sntp_example_task(void *arg);
static void button_check_task(void *arg);
static void controll_led_task(void *arg);

void app_main(void)
{
    esp_err_t WifiStt;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Setup GPIO pins*/
    gpio_setup();
    /* Wifi init */
    WifiStt = example_connect();

    /* Connected to Wifi */
    if(WifiStt == ESP_OK)
    {
        lib_gpio_write(WIFI_PIN, ON);
    }

    // SNTP service uses LwIP, please allocate large stack space.
    xTaskCreate(sntp_example_task, "sntp_example_task", 2048, NULL, 10, NULL);
    xTaskCreate(button_check_task, "button_check_task", 1024, NULL, 9, NULL);
    xTaskCreate(controll_led_task, "controll_led_task", 1024, NULL, 8, NULL);

}

static void gpio_setup(void)
{
    /* Setup mode */
    lib_gpio_output_init(RELAY_PIN);
    lib_gpio_output_init(WIFI_PIN);
    lib_gpio_input_init(BUTTON_PIN, GPIO_PULLUP_ONLY, GPIO_INTR_DISABLE);

    lib_gpio_write(WIFI_PIN, OFF);
    lib_gpio_write(RELAY_PIN, OFF);
}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

/* Check time to control Relay */
static void sntp_example_task(void *arg)
{
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    // Set timezone to China Standard Time
    setenv("TZ", "CST-7", 1);
    tzset();

    while (1) {
        // update 'now' variable with current time
        time(&now);
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_year < (2016 - 1900)) {
            ESP_LOGE(TAG, "The current date/time error");
        } else {
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "The current date/time in VietNam is: %s", strftime_buf);

            /* Time turn on/off for WINTER */
            if((timeinfo.tm_mon == JAN_MONTH) || (timeinfo.tm_mon == OCT_MONTH) || (timeinfo.tm_mon == NOV_MONTH) || (timeinfo.tm_mon == DEC_MONTH))
            {
                ESP_LOGI(TAG, "Is winter now");
                /* Time ON */
                if((timeinfo.tm_hour >= HOUR_WINTER_ON) || (timeinfo.tm_hour <= HOUR_WINTER_OFF))
                {
                    TimeStt = true;
                }
                else
                {
                    TimeStt = false;
                }
            }
            else
            {
                ESP_LOGI(TAG, "Is summer now");
                /* Time ON */
                if((timeinfo.tm_hour >= HOUR_SUMMER_ON) || (timeinfo.tm_hour <= HOUR_SUMMER_OFF))
                {
                    TimeStt = true;
                }
                else
                {
                    TimeStt = false;
                }
            }
        }

        /*ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());*/
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

/* Button check function */
static void button_check_task(void *arg)
{
    if(!lib_gpio_read(BUTTON_PIN))
    {
        vTaskDelay(10 / portTICK_RATE_MS);

        while(!lib_gpio_read(BUTTON_PIN));

        BtnStt = !BtnStt;

    }
}

/* Button check function */
static void controll_led_task(void *arg)
{
    if((BtnStt && TimeStt) || (BtnStt && !TimeStt) || (!BtnStt && TimeStt))
    {
        ESP_LOGI(TAG, "RELAY ON");
        lib_gpio_write(RELAY_PIN, ON);
    }
    else
    {
        ESP_LOGI(TAG, "RELAY OFF");
        lib_gpio_write(RELAY_PIN, OFF);
    }
}