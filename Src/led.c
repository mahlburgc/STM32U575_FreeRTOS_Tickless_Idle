/*
 * ledTasks.c
 *
 *  Created on: Mar 19, 2023
 *      Author: Christian Mahlburg
 */

#include "main.h"
#include "cli.h"
#include "led.h"

#include "FreeRTOS.h"
#include "task.h"

static void ledGreenTask(void* argument);

/**
 * @brief Task to control the green LED.
 */
static void ledGreenTask(void* argument)
{
    CLI_print("Start LED Green Task.\r\n");

    while(1)
    {
        CLI_print("LED Green toggled.\r\n");
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Initialize LED task.
 *        Should be called once on startup.
 */
void LED_taskInit(void)
{
    xTaskCreate(ledGreenTask, "ledGreenTask", 256, NULL, tskIDLE_PRIORITY + 5, NULL);
}
