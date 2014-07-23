CCPREFIX = $$HOME/stm32f4/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-
CC   = $(CCPREFIX)gcc
CP   = $(CCPREFIX)objcopy
AS   = $(CCPREFIX)gcc -x assembler-with-cpp
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary -S
MCU  = cortex-m4

# list all C defines here
DEFS = -DSTM32F4XX -DUSE_STDPERIPH_DRIVER

PROJECT = sample

# select source files to include in build process

SRC  = ./src/init/init.c
SRC += ./src/project/main.c
SRC += ./src/project/stm32f4xx_it.c
SRC += ./src/project/system_stm32f4xx.c

#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cec.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_comp.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c
SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
#SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c
SRC += src/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c

SRC += src/Utilities/STM32F4-Discovery/stm32f4_discovery.c
#SRC += src/Utilities/STM32F4-Discovery/stm32f4_discovery_lis302dl.c
#SRC += src/Utilities/STM32F4-Discovery/stm32f4_discovery_audio_codec.c

#SRC += src/Libraries/STM32_USB_Device_Library/Core/src/usbd_core.c
#SRC += src/Libraries/STM32_USB_Device_Library/Core/src/usbd_req.c
#SRC += src/Libraries/STM32_USB_Device_Library/Core/src/usbd_ioreq.c
#SRC += src/Libraries/STM32_USB_Device_Library/Class/hid/src/usbd_hid_core.c
#SRC += src/Libraries/STM32_USB_OTG_Driver/src/usb_core.c
#SRC += src/Libraries/STM32_USB_OTG_Driver/src/usb_dcd.c
#SRC += src/Libraries/STM32_USB_OTG_Driver/src/usb_dcd_int.c

###################################################################################

STARTUP = ./src/startup/startup_stm32f4xx.s

INCDIRS = \
          ../../gcc-arm-none-eabi-4_8-2014q2/arm-none-eabi/include \
          ./src/project \
          ./src/Libraries/CMSIS/Include \
          ./src/Libraries/CMSIS/ST/STM32F4xx/Include \
          ./src/Libraries/STM32F4xx_StdPeriph_Driver/inc \
          ./src/Libraries/STM32_USB_Device_Library/Class/audio/inc \
          ./src/Libraries/STM32_USB_Device_Library/Class/cdc/inc \
          ./src/Libraries/STM32_USB_Device_Library/Class/dfu/inc \
          ./src/Libraries/STM32_USB_Device_Library/Class/hid/inc \
          ./src/Libraries/STM32_USB_Device_Library/Class/msc/inc \
          ./src/Libraries/STM32_USB_Device_Library/Core/inc \
          ./src/Libraries/STM32_USB_HOST_Library/Class/HID/inc \
          ./src/Libraries/STM32_USB_HOST_Library/Class/MSC/inc \
          ./src/Libraries/STM32_USB_HOST_Library/Core/inc \
          ./src/Libraries/STM32_USB_OTG_Driver/inc \
          ./src/Utilities/STM32F4-Discovery

OPT = -O0

INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))

OBJS  = $(STARTUP:.s=.o) $(SRC:.c=.o)
MCFLAGS = -mcpu=$(MCU)

ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.s=.lst)
CPFLAGS = $(MCFLAGS) $(OPT) -g -gdwarf-2 -mthumb -nostdinc -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
LDFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb -nostartfiles -T$(LINKER_SCRIPT) -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)

all:
	@echo "Use \"make clean flash\" to compile all code for flash."
	@echo "Use \"make clean ram\" to compile isr vector for flash and everything else for ram."

flash:
	make flash0 LINKER_SCRIPT=./linker/stm32f4_linker_flash.ld

ram:
	make ram0 LINKER_SCRIPT=./linker/stm32f4_linker_ram.ld

flash0: $(OBJS) $(PROJECT).elf  $(PROJECT).hex
	$(CCPREFIX)objcopy -O binary -S -R .isr_vector $(PROJECT).elf $(PROJECT).bin
	$(TRGT)size $(PROJECT).elf
	$(CCPREFIX)objdump -d $(PROJECT).elf > $(PROJECT).dis
	@(find src -name "*.o"; echo obj) | xargs mv
	@(find src -name "*.lst"; echo lst) | xargs mv
	@mv $(PROJECT).* bin

ram0: $(OBJS) $(PROJECT).elf  $(PROJECT).hex
	$(CCPREFIX)objcopy -j .isr_vector -O binary -S $(PROJECT).elf $(PROJECT).isrvector
	$(CCPREFIX)objcopy -O binary -S -R .isr_vector $(PROJECT).elf $(PROJECT).bin
	$(TRGT)size $(PROJECT).elf
	$(CCPREFIX)objdump -d $(PROJECT).elf > $(PROJECT).dis
	@(find src -name "*.o"; echo obj) | xargs mv
	@(find src -name "*.lst"; echo lst) | xargs mv
	@mv $(PROJECT).* bin

%.o: %.c
	$(CC) -c $(CPFLAGS) $(INCDIR) $< -o $@

%.o: %.s
	$(AS) -c $(ASFLAGS) $< -o $@

%elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%hex: %elf
	$(HEX) $< $@
	
clean:
	-rm -rf obj/*
	-rm -rf lst/*
	-rm -rf bin/*
	-find src -name "*.o" -exec rm {} \;
