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

LDFLAGS  = -mcpu=cortex-m3 -mthumb -T STM32F103XB_FLASH.ld
LDFLAGS += -Wl,--gc-sections -nostartfiles -specs=nosys.specs -lc -lm

C_SRCS  = $(TARGET)_main.c stm32f1xx_it_bringup.c system_clock_config.c
C_SRCS += $(EXTRA_SRCS)
C_SRCS += $(ROOT)/Drivers/CMSIS/Device/ST/STM32F1xx/system_stm32f1xx.c
C_SRCS += $(wildcard $(HAL_SRC)/*.c)

ASM_SRCS = startup_stm32f103xb.s

OBJS  = $(notdir $(C_SRCS:.c=.o))
OBJS += $(ASM_SRCS:.s=.o)

vpath %.c . $(ROOT)/Src $(ROOT)/Drivers/CMSIS/Device/ST/STM32F1xx $(HAL_SRC)

.PHONY: all clean flash debug-server

all: $(TARGET).bin

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

flash: $(TARGET).bin
	$(OPENOCD) -f $(OPENOCD_CONFIG) -c "program $(TARGET).elf verify reset exit"

debug-server:
	$(OPENOCD) -f $(OPENOCD_CONFIG)

clean:
	rm -f *.o *.elf *.bin
