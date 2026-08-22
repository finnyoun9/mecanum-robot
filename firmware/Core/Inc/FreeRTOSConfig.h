/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS kernel configuration for the mecanum-robot chassis MCU
 *        (STM32F103C8T6, Cortex-M3, GCC ARM_CM3 port).
 *
 * configCPU_CLOCK_HZ must match SystemClock_Config()'s actual output.
 * This board has no HSE crystal, so the PLL runs off HSI (see main.c) —
 * target is 64 MHz, not the usual HSE-sourced 72 MHz.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      (64000000UL)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    (8)
#define configMINIMAL_STACK_SIZE                 ((unsigned short)128)
#define configTOTAL_HEAP_SIZE                   ((size_t)(10 * 1024))
#define configMAX_TASK_NAME_LEN                 (10)
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_TASK_NOTIFICATIONS            1
#define configQUEUE_REGISTRY_SIZE               4

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (2)
#define configTIMER_QUEUE_LENGTH                 4
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE)

/* --- Hook function related definitions --- */
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_MALLOC_FAILED_HOOK             1

/* --- Run time and task stats gathering --- */
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

/* --- Co-routine definitions (unused) --- */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          (2)

/* --- API functions this firmware actually calls --- */
#define INCLUDE_vTaskPrioritySet                 0
#define INCLUDE_uxTaskPriorityGet                0
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark       1
#define INCLUDE_xTaskGetCurrentTaskHandle         1

/* --- Cortex-M3 interrupt priority config ---
 * F1 uses 4 priority bits (NVIC_PRIORITYGROUP_4, all preemption, no
 * subpriority) — matches HAL_Init()'s default priority grouping. */
#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* --- Route the CMSIS vector table straight to the FreeRTOS port ---
 * The startup file's vector table names these SVC_Handler/PendSV_Handler/
 * SysTick_Handler; the ARM_CM3 port defines vPortSVCHandler/
 * xPortPendSVHandler/xPortSysTickHandler. Renaming here makes port.c's
 * definitions compile directly under the vector table's expected names —
 * no separate stub needed in stm32f1xx_it.c for these three. */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for (;;); }

#endif /* FREERTOS_CONFIG_H */
