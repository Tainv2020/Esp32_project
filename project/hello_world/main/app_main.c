/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#define MAX_TIMER 3
#define TIMER0_ID 0
#define TIMER1_ID 1
#define TIMER2_ID 2

TimerHandle_t xTimers[MAX_TIMER];

/******************************************** Function ******************************/
/* Task */
static void app_task1(void *pvParameters)
{
    while(1)
    {
        printf("Task1\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void app_task2(void *pvParameters)
{
    while(1)
    {
        printf("Task2\n");
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}
static void app_task3(void *pvParameters)
{
    while(1)
    {
        printf("Task3\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

/* Soft timer callback */
void vTimerCallback(TimerHandle_t xTimer)
{
    uint8_t Counter = 0;

    configASSERT( xTimer );
    Counter = ( uint32_t ) pvTimerGetTimerID( xTimer );

    if(Counter == TIMER0_ID)
    {
        printf("Timer 1\n");
    }
    else if(Counter == TIMER1_ID)
    {
        printf("Timer 2\n");
    }
    else if(Counter == TIMER2_ID)
    {
        printf("Timer 3\n");
    }
}

void app_main(void)
{
    /* Create timer */
    xTimers[TIMER0_ID] = xTimerCreate("timer task 0", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, vTimerCallback);
    xTimers[TIMER1_ID] = xTimerCreate("timer task 1", pdMS_TO_TICKS(1000), pdTRUE, (void *)1, vTimerCallback);
    xTimers[TIMER2_ID] = xTimerCreate("timer task 2", pdMS_TO_TICKS(1000), pdTRUE, (void *)2, vTimerCallback);

    /* Create task */
    xTaskCreate(app_task1, "app task1", 2048, NULL, 12, NULL);
    xTaskCreate(app_task2, "app task2", 2048, NULL, 11, NULL);
    xTaskCreate(app_task3, "app task3", 2048, NULL, 10, NULL);

    xTimerStart(xTimers[0], 0);
    xTimerStart(xTimers[1], 0);
    xTimerStart(xTimers[2], 0);
}
