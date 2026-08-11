.syntax unified
.cpu cortex-m3
.thumb

.equ Stack_Size, 0x400
.equ Heap_Size,  0x200

.section .stack, "aw", %nobits
.align 3
.global __initial_sp
Stack_Mem:
    .space Stack_Size
__initial_sp:

.section .heap, "aw", %nobits
.align 3
.global __heap_base
.global __heap_limit
__heap_base:
Heap_Mem:
    .space Heap_Size
__heap_limit:

.section .isr_vector, "a"
.global __Vectors
.global __Vectors_End
.global __Vectors_Size

__Vectors:
    .word __initial_sp
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

    /* External Interrupts */
    .word WWDG_IRQHandler
    .word PVD_IRQHandler
    .word TAMPER_IRQHandler
    .word RTC_IRQHandler
    .word FLASH_IRQHandler
    .word RCC_IRQHandler
    .word EXTI0_IRQHandler
    .word EXTI1_IRQHandler
    .word EXTI2_IRQHandler
    .word EXTI3_IRQHandler
    .word EXTI4_IRQHandler
    .word DMA1_Channel1_IRQHandler
    .word DMA1_Channel2_IRQHandler
    .word DMA1_Channel3_IRQHandler
    .word DMA1_Channel4_IRQHandler
    .word DMA1_Channel5_IRQHandler
    .word DMA1_Channel6_IRQHandler
    .word DMA1_Channel7_IRQHandler
    .word ADC1_2_IRQHandler
    .word USB_HP_CAN1_TX_IRQHandler
    .word USB_LP_CAN1_RX0_IRQHandler
    .word CAN1_RX1_IRQHandler
    .word CAN1_SCE_IRQHandler
    .word EXTI9_5_IRQHandler
    .word TIM1_BRK_IRQHandler
    .word TIM1_UP_IRQHandler
    .word TIM1_TRG_COM_IRQHandler
    .word TIM1_CC_IRQHandler
    .word TIM2_IRQHandler
    .word TIM3_IRQHandler
    .word TIM4_IRQHandler
    .word I2C1_EV_IRQHandler
    .word I2C1_ER_IRQHandler
    .word I2C2_EV_IRQHandler
    .word I2C2_ER_IRQHandler
    .word SPI1_IRQHandler
    .word SPI2_IRQHandler
    .word USART1_IRQHandler
    .word USART2_IRQHandler
    .word USART3_IRQHandler
    .word EXTI15_10_IRQHandler
    .word RTCAlarm_IRQHandler
    .word USBWakeUp_IRQHandler
__Vectors_End:

.equ __Vectors_Size, __Vectors_End - __Vectors

.section .text

.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =SystemInit
    blx r0

    /* Copy .data from FLASH to RAM */
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
    cmp r1, r2
    beq 2f
1:  ldr r3, [r0], #4
    str r3, [r1], #4
    cmp r1, r2
    bne 1b
2:

    /* Zero .bss */
    ldr r0, =_sbss
    ldr r1, =_ebss
    mov r2, #0
    cmp r0, r1
    beq 2f
1:  str r2, [r0], #4
    cmp r0, r1
    bne 1b
2:

    bl main
    b .
.size Reset_Handler, . - Reset_Handler

.macro DEFAULT_HANDLER name
.weak \name
.type \name, %function
\name:
    b .
.size \name, . - \name
.endm

DEFAULT_HANDLER NMI_Handler
DEFAULT_HANDLER HardFault_Handler
DEFAULT_HANDLER MemManage_Handler
DEFAULT_HANDLER BusFault_Handler
DEFAULT_HANDLER UsageFault_Handler
DEFAULT_HANDLER SVC_Handler
DEFAULT_HANDLER DebugMon_Handler
DEFAULT_HANDLER PendSV_Handler
DEFAULT_HANDLER SysTick_Handler
DEFAULT_HANDLER WWDG_IRQHandler
DEFAULT_HANDLER PVD_IRQHandler
DEFAULT_HANDLER TAMPER_IRQHandler
DEFAULT_HANDLER RTC_IRQHandler
DEFAULT_HANDLER FLASH_IRQHandler
DEFAULT_HANDLER RCC_IRQHandler
DEFAULT_HANDLER EXTI0_IRQHandler
DEFAULT_HANDLER EXTI1_IRQHandler
DEFAULT_HANDLER EXTI2_IRQHandler
DEFAULT_HANDLER EXTI3_IRQHandler
DEFAULT_HANDLER EXTI4_IRQHandler
DEFAULT_HANDLER DMA1_Channel1_IRQHandler
DEFAULT_HANDLER DMA1_Channel2_IRQHandler
DEFAULT_HANDLER DMA1_Channel3_IRQHandler
DEFAULT_HANDLER DMA1_Channel4_IRQHandler
DEFAULT_HANDLER DMA1_Channel5_IRQHandler
DEFAULT_HANDLER DMA1_Channel6_IRQHandler
DEFAULT_HANDLER DMA1_Channel7_IRQHandler
DEFAULT_HANDLER ADC1_2_IRQHandler
DEFAULT_HANDLER USB_HP_CAN1_TX_IRQHandler
DEFAULT_HANDLER USB_LP_CAN1_RX0_IRQHandler
DEFAULT_HANDLER CAN1_RX1_IRQHandler
DEFAULT_HANDLER CAN1_SCE_IRQHandler
DEFAULT_HANDLER EXTI9_5_IRQHandler
DEFAULT_HANDLER TIM1_BRK_IRQHandler
DEFAULT_HANDLER TIM1_UP_IRQHandler
DEFAULT_HANDLER TIM1_TRG_COM_IRQHandler
DEFAULT_HANDLER TIM1_CC_IRQHandler
DEFAULT_HANDLER TIM2_IRQHandler
DEFAULT_HANDLER TIM3_IRQHandler
DEFAULT_HANDLER TIM4_IRQHandler
DEFAULT_HANDLER I2C1_EV_IRQHandler
DEFAULT_HANDLER I2C1_ER_IRQHandler
DEFAULT_HANDLER I2C2_EV_IRQHandler
DEFAULT_HANDLER I2C2_ER_IRQHandler
DEFAULT_HANDLER SPI1_IRQHandler
DEFAULT_HANDLER SPI2_IRQHandler
DEFAULT_HANDLER USART1_IRQHandler
DEFAULT_HANDLER USART2_IRQHandler
DEFAULT_HANDLER USART3_IRQHandler
DEFAULT_HANDLER EXTI15_10_IRQHandler
DEFAULT_HANDLER RTCAlarm_IRQHandler
DEFAULT_HANDLER USBWakeUp_IRQHandler

.end
