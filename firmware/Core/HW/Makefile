# Real-hardware build for the mecanum-robot chassis MCU (STM32F103C8T6).
#
# This is the bring-up smoke test target only (blink PC13) — proves the
# toolchain/linker/startup/clock chain on actual silicon. The full
# FreeRTOS + motor/encoder/UART/I2C firmware is layered in afterward as a
# separate target once this passes; see docs/hardware-closed-loop-roadmap.md.
#
# Vendored dependencies (see git log for provenance):
#   Drivers/CMSIS/{Include,Device/ST/STM32F1xx}  — ARM CMSIS-Core + ST device pack
#   Drivers/STM32F1xx_HAL_Driver                  — STM32Cube HAL (official ST repo)
#   Middleware/FreeRTOS                           — FreeRTOS kernel + ARM_CM3 GCC port

TOOLCHAIN = /Users/finn/.local/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

TARGET ?= bringup
EXTRA_SRCS ?=
OPENOCD ?= openocd
OPENOCD_CONFIG = stlink_stm32f1.cfg
STFLASH ?= st-flash

ROOT       = ..
DRIVERS    = $(ROOT)/Drivers
CMSIS_INC  = $(DRIVERS)/CMSIS/Include
CMSIS_DEV  = $(DRIVERS)/CMSIS/Device/ST/STM32F1xx/Include
HAL_INC    = $(DRIVERS)/STM32F1xx_HAL_Driver/Inc
HAL_SRC    = $(DRIVERS)/STM32F1xx_HAL_Driver/Src

CFLAGS  = -mcpu=cortex-m3 -mthumb
CFLAGS += -Os -g -Wall -Werror -fdata-sections -ffunction-sections
CFLAGS += -DSTM32F103xB -DUSE_HAL_DRIVER
CFLAGS += -I. -I$(ROOT)/Inc -I$(CMSIS_INC) -I$(CMSIS_DEV) -I$(HAL_INC)
CFLAGS += -I$(ROOT)/../../shared

LDFLAGS  = -mcpu=cortex-m3 -mthumb -T STM32F103XB_FLASH.ld
LDFLAGS += -Wl,--gc-sections -nostartfiles -specs=nosys.specs

# Libraries must follow the objects on the link line, not precede them: ld
# resolves left to right, so -lc -lm placed in LDFLAGS (which comes first)
# were scanned before any undefined symbol existed and then never revisited.
# This only surfaced when SensorTask started referencing ahrs.c and pulled
# in sqrtf -> __errno; before that --gc-sections dropped the whole AHRS
# section and nothing needed libm.
LDLIBS = -lc -lm

C_SRCS  = $(TARGET)_main.c system_clock_config.c
C_SRCS += $(ROOT)/Drivers/CMSIS/Device/ST/STM32F1xx/system_stm32f1xx.c
C_SRCS += $(wildcard $(HAL_SRC)/*.c)

ASM_SRCS = startup_stm32f103xb.s

ifeq ($(RTOS),1)
# --- FreeRTOS drive target: full Core/Src application on real silicon ---
# TARGET=rtos_drive RTOS=1
FREERTOS = $(ROOT)/Middleware/FreeRTOS/Source
C_SRCS += stm32f1xx_it_rtos.c
# Application (Core/Src) + shared protocol
C_SRCS += main.c robot_control.c motor.c encoder.c pid.c mecanum_ik.c ahrs.c \
          mpu6050.c tof_sensor.c ssd1306.c nrf24l01.c remote_control.c protocol.c
# FreeRTOS kernel (ARM_CM3 port, heap_4 dynamic allocation)
C_SRCS += tasks.c queue.c list.c timers.c port.c heap_4.c
CFLAGS += -I fwinc -DHW_MINIMAL_TASKS
ifneq ($(TOF),1)
# Keep an unfitted VL53L0X from presenting as a permanent fault. Build with
# TOF=1 once the module is wired to the shared PB10/PB11 bus.
CFLAGS += -DHW_IMU_ONLY
endif
CFLAGS += -I$(FREERTOS)/include -I$(FREERTOS)/portable/GCC/ARM_CM3
else
# --- Bare-metal bring-up targets (no FreeRTOS) ---
C_SRCS += stm32f1xx_it_bringup.c
C_SRCS += $(EXTRA_SRCS)
endif

OBJS  = $(notdir $(C_SRCS:.c=.o))
OBJS += $(ASM_SRCS:.s=.o)

vpath %.c . $(ROOT)/Src $(ROOT)/Drivers/CMSIS/Device/ST/STM32F1xx $(HAL_SRC) \
    $(ROOT)/../../shared $(FREERTOS) $(FREERTOS)/portable/GCC/ARM_CM3 \
    $(FREERTOS)/portable/MemMang

.PHONY: all clean flash flash-stlink debug-server

all: $(TARGET).bin

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

flash: $(TARGET).bin
	$(OPENOCD) -f $(OPENOCD_CONFIG) -c "program $(TARGET).elf verify reset exit"

# Some older ST-Link V2 probes report SWD IDCODE 0x1ba01477, which OpenOCD
# 0.12 rejects while stlink's own flash loader handles and verifies correctly.
flash-stlink: $(TARGET).bin
	$(STFLASH) --reset write $(TARGET).bin 0x08000000

debug-server:
	$(OPENOCD) -f $(OPENOCD_CONFIG)

clean:
	rm -f *.o *.elf *.bin
