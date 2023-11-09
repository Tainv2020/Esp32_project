/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "lib_gpio.h"

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2

static uint8_t s_led_state = 0;

void app_gpio_callback_func(int pin)
{
    if(pin == GPIO_NUM_0)
    {
        //lib_gpio_toggled(BLINK_GPIO);
         lib_gpio_write(BLINK_GPIO, s_led_state);
        s_led_state = !s_led_state; 
    }
}

static void app_toggled(void *pvParameters)
{
    lib_gpio_toggled(BLINK_GPIO);
    vTaskDelay(100);
}

void app_main(void)
{
    lib_gpio_input_init(GPIO_NUM_0, GPIO_PULLUP_ONLY, GPIO_INTR_NEGEDGE);
    lib_gpio_set_callback(app_gpio_callback_func);
    lib_gpio_output_init(BLINK_GPIO);

    xTaskCreate(app_toggled, "Toggled", 2048, NULL, 12, NULL);
}
