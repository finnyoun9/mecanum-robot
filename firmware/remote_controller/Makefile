TOOLCHAIN = /Users/finn/.local/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
OBJDUMP = $(TOOLCHAIN)/arm-none-eabi-objdump
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

TARGET = remote-control

CFLAGS  = -mcpu=cortex-m3 -mthumb
CFLAGS += -Os -g -Wall -fdata-sections -ffunction-sections
CFLAGS += -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD
CFLAGS += -I./Start -I./Library -I./User -I./System -I./Hardware

LDFLAGS = -mcpu=cortex-m3 -mthumb -T STM32F103C8_FLASH.ld
LDFLAGS += -Wl,--gc-sections -nostartfiles -specs=nosys.specs -lc -lm

C_SRCS  = $(filter-out %core_cm3.c, $(wildcard ./Start/*.c))
C_SRCS += $(wildcard ./Library/*.c)
C_SRCS += $(wildcard ./User/*.c)
C_SRCS += $(wildcard ./System/*.c)
C_SRCS += $(wildcard ./Hardware/*.c)

ASM_SRCS = ./Start/startup_stm32f10x_md_gcc.s

OBJS  = $(C_SRCS:.c=.o)
OBJS += $(ASM_SRCS:.s=.o)

.PHONY: all clean flash

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
	st-flash write $< 0x08000000

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin
