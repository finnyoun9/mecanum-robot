/**
 * @file FreeRTOS.h — Minimal SIL mock for FreeRTOS kernel types.
 *
 * Provides just enough definitions so the firmware sources compile as a
 * Linux host binary.  All task/queue/semaphore operations are no-ops; the
 * SIL scheduler drives each "task" function directly in a round-robin loop.
 */
#ifndef SIL_FREERTOS_H
#define SIL_FREERTOS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Base types --- */
typedef uint32_t TickType_t;
typedef int32_t  BaseType_t;
typedef void *   TaskHandle_t;
typedef void *   QueueHandle_t;
typedef void *   SemaphoreHandle_t;
typedef void *   TimerHandle_t;
typedef void *   EventGroupHandle_t;
typedef void *   StreamBufferHandle_t;
typedef void *   MessageBufferHandle_t;

#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define pdFAIL  0

#define portMAX_DELAY      UINT32_MAX
#define portTICK_PERIOD_MS 1U

/* --- Critical section (no-op in SIL) --- */
#define taskENTER_CRITICAL()
#define taskEXIT_CRITERIAL()
#define portENTER_CRITICAL()
#define portEXIT_CRITICAL()

/* --- Scheduler control --- */
#define taskDISABLE_INTERRUPTS()  0
#define taskENABLE_INTERRUPTS()

/* --- Task notification --- */
#define eNoAction       0
#define eSetBits        1
#define eIncrement      2
#define eSetValueWithOverwrite 3
#define eSetValueWithoutOverwrite 4

/* --- Memory (no MPU in SIL) --- */
#define configMINIMAL_STACK_SIZE  128

BaseType_t xTaskCreate(void (*pxTaskCode)(void *), const char *pcName,
                       uint16_t usStackDepth, void *pvParameters,
                       unsigned portBASE_PRIORITY, TaskHandle_t *pxCreatedTask);
void vTaskStartScheduler(void);
void vTaskDelay(TickType_t xTicksToDelay);
void vTaskDelayUntil(TickType_t *pxPreviousWakeTime, TickType_t xTimeIncrement);
TickType_t xTaskGetTickCount(void);
uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait);
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask);
void vTaskSuspend(TaskHandle_t xTask);
void vTaskResume(TaskHandle_t xTask);
void vTaskDelete(TaskHandle_t xTask);

/* --- Queue --- */
QueueHandle_t xQueueCreate(uint32_t uxQueueLength, uint32_t uxItemSize);
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue,
                      TickType_t xTicksToWait);
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer,
                         TickType_t xTicksToWait);

/* --- Semaphore --- */
SemaphoreHandle_t xSemaphoreCreateBinary(void);
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore,
                                 BaseType_t *pxHigherPriorityTaskWoken);

/* --- Software timer --- */
TimerHandle_t xTimerCreate(const char *pcTimerName, TickType_t xTimerPeriod,
                           unsigned portBASE_AUTO_RELOAD, void *pvTimerID,
                           void (*pxCallbackFunction)(TimerHandle_t));
BaseType_t xTimerStart(TimerHandle_t xTimer, TickType_t xTicksToWait);

/* --- Helpers --- */
#define pdMS_TO_TICKS(ms)  ((TickType_t)(ms))

/* --- Notify from ISR --- */
BaseType_t xTaskNotifyFromISR(TaskHandle_t xTask, uint32_t ulValue,
                              uint32_t eAction, BaseType_t *pxHigherPriorityTaskWoken);
#define portYIELD_FROM_ISR(x)  ((void)(x))

/* --- SIL scheduler API (called from sil_main.c) --- */
void sil_sched_tick(void);
int  sil_sched_task_count(void);

#ifdef __cplusplus
}
#endif

#endif /* SIL_FREERTOS_H */
