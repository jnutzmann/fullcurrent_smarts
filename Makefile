TARGET:=smarts

# Define the toolchain
TOOLCHAIN_ROOT:=/usr/local/lib/gcc-arm-none-eabi-4_9-2015q3
TOOLCHAIN_PATH:=$(TOOLCHAIN_ROOT)/bin
TOOLCHAIN_PREFIX:=arm-none-eabi

CC=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
LD=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-objcopy
AS=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-as
AR=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-ar
GDB=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gdb

# Optimization level, can be [0, 1, 2, 3, s].
OPTLVL:=0
DBG:=-g

STARTUP:=$(CURDIR)/hardware
LINKER_SCRIPT:=$(CURDIR)/Utilities/stm32_flash.ld

CDEFS=-DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DHSE_VALUE=8000000 -D__FPU_PRESENT=1 -D__FPU_USED=1 -DARM_MATH_CM4

MCUFLAGS=-mcpu=cortex-m4 -mthumb -mfloat-abi=hard
COMMONFLAGS=-O$(OPTLVL) $(DBG) -Wall
CFLAGS=$(COMMONFLAGS) $(MCUFLAGS) $(INCLUDE) $(CDEFS) -std=gnu99
LDLIBS=$(TOOLCHAIN_ROOT)/arm-none-eabi/lib/armv7e-m/fpu/libc.a $(TOOLCHAIN_ROOT)/arm-none-eabi/lib/armv7e-m/fpu/libm.a
LDFLAGS=$(COMMONFLAGS) -fno-exceptions -ffunction-sections -fdata-sections -nostartfiles -Wl,--gc-sections,-T$(LINKER_SCRIPT)

BUILD_DIR = build
BIN_DIR = bin

# Include Files

INCLUDE= \
-Ihardware \
-Id1k/FreeRTOS/include \
-Id1k/FreeRTOS/portable/GCC/ARM_CM4F \
-Id1k/CMSIS/Device/ST/STM32F4xx/Include \
-Id1k/CMSIS/Include \
-Id1k/STM32F4xx_StdPeriph_Driver/inc \
-Iconfig \
-Id1k/src \
-Isrc

# vpath is used so object files are written to the current directory instead
# of the same directory as their source files
vpath %.c d1k/STM32F4xx_StdPeriph_Driver/src d1k/syscall hardware d1k/FreeRTOS \
	  d1k/FreeRTOS/portable/MemMang d1k/FreeRTOS/portable/GCC/ARM_CM4F \
	  d1k/src d1k/src/drivers src

vpath %.s $(STARTUP)
ASRC=startup_stm32f4xx.s

# Project Source Files
SRC= \
stm32f4xx_it.c \
system_stm32f4xx.c \
main.c \
syscalls.c

# FreeRTOS Source Files
SRC+=port.c list.c queue.c tasks.c event_groups.c timers.c heap_4.c

# d1k Source Files
SRC+= \
led.c \
can.c \
i2c.c \
nvmem.c

# Standard Peripheral Source Files
SRC+= \
misc.c \
stm32f4xx_adc.c \
stm32f4xx_can.c \
stm32f4xx_dac.c \
stm32f4xx_dma.c \
stm32f4xx_exti.c \
stm32f4xx_flash.c \
stm32f4xx_gpio.c \
stm32f4xx_i2c.c \
stm32f4xx_rcc.c \
stm32f4xx_rng.c \
stm32f4xx_spi.c \
stm32f4xx_syscfg.c \
stm32f4xx_tim.c \
stm32f4xx_usart.c

OBJ = $(SRC:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)
	@echo [CC] $(notdir $<)
	@$(CC) $(CFLAGS) $< -c -o $@

all: $(OBJ)
	@echo [AS] $(ASRC)
	@$(AS) -o $(ASRC:%.s=$(BUILD_DIR)/%.o) $(STARTUP)/$(ASRC)
	@echo [LD] $(TARGET).elf
	@$(CC) -o $(BIN_DIR)/$(TARGET).elf $(LDFLAGS) $(OBJ) $(ASRC:%.s=$(BUILD_DIR)/%.o) $(LDLIBS)
	@echo [OBJCOPY] $(TARGET).hex
	@$(OBJCOPY) -O ihex $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex
	@echo [OBJCOPY] $(TARGET).bin
	@$(OBJCOPY) -O binary $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).bin

.PHONY: clean

clean:
	@echo [RM] OBJ
	@rm -f $(OBJ)
	@echo [RM] BUILD
	@rm -rf $(BUILD_DIR)
	@echo [RM] BIN
	@rm -rf $(BIN_DIR)
	
jlink:
	JLinkExe flash.jlink

jtag:
	openocd -f $(TARGET).cfg

cb: clean all

cbf: clean all jlink

bf: all jlink
