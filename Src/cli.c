/*
 * cli.c
 *
 *  Created on: Mar 19, 2023
 *      Author: Christian Mahlburg
 */

#include "usart1.h"
#include "cli.h"

#include "stdint.h"
#include "stm32_assert.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdbool.h"

#define CLI_QUEUE_LENGTH    10
#define CLI_ITEM_SIZE       80

static QueueHandle_t cliQueueHandle;

typedef struct
{
    char msg[CLI_ITEM_SIZE];
} CLI_QueueItem_t;

/* The variable used to hold the queue's data structure. */
static StaticQueue_t xStaticQueue;
/* The array to use as the queue's storage area.  This must be at least
uxQueueLength * uxItemSize bytes. */
uint8_t ucQueueStorageArea[CLI_QUEUE_LENGTH * CLI_ITEM_SIZE];

static void CLI_task(void* argument);

/**
 * @brief Task to send outgoing CLI messages to uart.
 */
static void CLI_task(void* argument)
{
    CLI_QueueItem_t receivedQueueItem;

    cliQueueHandle = xQueueCreateStatic(CLI_QUEUE_LENGTH, CLI_ITEM_SIZE, ucQueueStorageArea, &xStaticQueue);

    /* pxQueueBuffer was not NULL so xQueue should not be NULL. */
    configASSERT(cliQueueHandle);

    while (1)
    {
        if (xQueueReceive(cliQueueHandle, &receivedQueueItem, portMAX_DELAY) == pdPASS)
        {
            usart1_transmit(receivedQueueItem.msg);
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief Public API to send messages to the CLI.
 *        A queue mechanism is used. CLI messages are printed to terminal via UART in CLI task.
 */
bool CLI_print(const char* msg)
{
    assert_param(msg != NULL);

    bool status = false;
    CLI_QueueItem_t queueItem;
    uint8_t msgLen = strnlen(msg, CLI_ITEM_SIZE);

    if (msgLen < CLI_ITEM_SIZE)
    {
        if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        {
            /* directly print messages if scheduler was not started yet */
            usart1_transmit(msg);
            status = true;
        }
        else
        {
            strncpy(queueItem.msg, msg, CLI_ITEM_SIZE);
            if(xQueueSendToBack(cliQueueHandle, &queueItem, 100) == pdPASS)
            {
                status = true;
            }
        }
    }

    return status;
}

/**
 * @brief Initialize the CLI task.
 *        Should be called once on startup.
 */
void CLI_taskInit(void)
{
    xTaskCreate(CLI_task, "CLI_task", 256, NULL, tskIDLE_PRIORITY + 5, NULL);
}
