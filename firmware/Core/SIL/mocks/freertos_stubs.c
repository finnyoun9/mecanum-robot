/**
 * @file freertos_stubs.c — SIL mock FreeRTOS implementation.
 *
 * When SIL_BUILD is defined, each task function runs exactly ONE loop
 * iteration and returns (the `for(;;)` and blocking calls are compiled
 * out).  This file provides a simple round-robin scheduler that calls
 * every registered task once per tick.
 */
#include "FreeRTOS.h"
#include <string.h>
#include <stdio.h>

#define SIL_MAX_TASKS 8

typedef struct {
    void (*code)(void *);
    const char *name;
    void       *param;
    unsigned    prio;
} sil_task_t;

static sil_task_t  sil_tasks[SIL_MAX_TASKS];
static int         sil_task_count   = 0;

/* ========================================================================
 *  Task
 * ======================================================================== */

BaseType_t xTaskCreate(void (*pxTaskCode)(void *), const char *pcName,
                       uint16_t usStackDepth, void *pvParameters,
                       unsigned portBASE_PRIORITY, TaskHandle_t *pxCreatedTask) {
    (void)usStackDepth;
    if (sil_task_count >= SIL_MAX_TASKS) return pdFAIL;

    sil_tasks[sil_task_count].code      = pxTaskCode;
    sil_tasks[sil_task_count].name      = pcName;
    sil_tasks[sil_task_count].param     = pvParameters;
    sil_tasks[sil_task_count].prio      = portBASE_PRIORITY;

    if (pxCreatedTask) {
        *pxCreatedTask = (TaskHandle_t)(uintptr_t)(sil_task_count + 1);
    }
    sil_task_count++;
    return pdPASS;
}

void vTaskStartScheduler(void) {
    /* Sort by priority descending */
    for (int i = 0; i < sil_task_count - 1; i++) {
        for (int j = i + 1; j < sil_task_count; j++) {
            if (sil_tasks[j].prio > sil_tasks[i].prio) {
                sil_task_t t = sil_tasks[i];
                sil_tasks[i] = sil_tasks[j];
                sil_tasks[j] = t;
            }
        }
    }

    extern volatile int sil_exit_flag;
    for (;;) {
        sil_sched_tick();
        if (sil_exit_flag) break;
    }
}

/* ========================================================================
 *  SIL Scheduler — called once per tick from sil_main
 * ======================================================================== */

/**
 * Run one tick: call every registered task once.  Each task runs one
 * loop iteration and returns (the `for(;;)` + yield are ifdef'd out).
 */
void sil_sched_tick(void) {
    extern volatile uint32_t mock_tick_ms;
    mock_tick_ms++;

    for (int i = 0; i < sil_task_count; i++) {
        if (sil_tasks[i].code) {
            sil_tasks[i].code(sil_tasks[i].param);
        }
    }
}

int sil_sched_task_count(void) {
    return sil_task_count;
}

/* ========================================================================
 *  Blocking calls — no-ops in SIL
 * ======================================================================== */

void vTaskDelay(TickType_t xTicksToDelay) {
    (void)xTicksToDelay;
}

void vTaskDelayUntil(TickType_t *pxPreviousWakeTime, TickType_t xTimeIncrement) {
    if (pxPreviousWakeTime) {
        *pxPreviousWakeTime += xTimeIncrement;
    }
}

TickType_t xTaskGetTickCount(void) {
    extern volatile uint32_t mock_tick_ms;
    return mock_tick_ms;
}

uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait) {
    (void)xClearCountOnExit;
    (void)xTicksToWait;
    return 0;
}

uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask) {
    (void)xTask;
    return 1024;
}

void vTaskSuspend(TaskHandle_t xTask)   { (void)xTask; }
void vTaskResume(TaskHandle_t xTask)    { (void)xTask; }
void vTaskDelete(TaskHandle_t xTask)    { (void)xTask; }

/* ========================================================================
 *  Queue
 * ======================================================================== */

QueueHandle_t xQueueCreate(uint32_t uxQueueLength, uint32_t uxItemSize) {
    (void)uxQueueLength;
    (void)uxItemSize;
    return (QueueHandle_t)1;
}

BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue,
                      TickType_t xTicksToWait) {
    (void)xQueue;
    (void)pvItemToQueue;
    (void)xTicksToWait;
    return pdPASS;
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer,
                         TickType_t xTicksToWait) {
    (void)xQueue;
    (void)pvBuffer;
    (void)xTicksToWait;
    return pdPASS;
}

/* ========================================================================
 *  Semaphore
 * ======================================================================== */

SemaphoreHandle_t xSemaphoreCreateBinary(void) {
    return (SemaphoreHandle_t)1;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    (void)xSemaphore;
    return pdPASS;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait) {
    (void)xSemaphore;
    (void)xTicksToWait;
    return pdPASS;
}

BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore,
                                 BaseType_t *pxHigherPriorityTaskWoken) {
    (void)xSemaphore;
    if (pxHigherPriorityTaskWoken) *pxHigherPriorityTaskWoken = pdFALSE;
    return pdPASS;
}

/* ========================================================================
 *  Timer
 * ======================================================================== */

TimerHandle_t xTimerCreate(const char *pcTimerName, TickType_t xTimerPeriod,
                           unsigned portBASE_AUTO_RELOAD, void *pvTimerID,
                           void (*pxCallbackFunction)(TimerHandle_t)) {
    (void)pcTimerName;
    (void)xTimerPeriod;
    (void)portBASE_AUTO_RELOAD;
    (void)pvTimerID;
    (void)pxCallbackFunction;
    return NULL;
}

BaseType_t xTimerStart(TimerHandle_t xTimer, TickType_t xTicksToWait) {
    (void)xTimer;
    (void)xTicksToWait;
    return pdPASS;
}

/* ========================================================================
 *  Notify from ISR
 * ======================================================================== */

BaseType_t xTaskNotifyFromISR(TaskHandle_t xTask, uint32_t ulValue,
                              uint32_t eAction, BaseType_t *pxHigherPriorityTaskWoken) {
    (void)xTask;
    (void)ulValue;
    (void)eAction;
    if (pxHigherPriorityTaskWoken) *pxHigherPriorityTaskWoken = pdFALSE;
    return pdPASS;
}
