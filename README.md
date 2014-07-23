stm32f4discovery-linux-how-to
=============================

#### Sample STM32F4Discovery environment setup under linux with arm-none-eabi-gcc, st-flash, st-util, arm-none-ebi-gdb and eclipse

##### 1. Working directory
Create a working directory with

       mkdir $HOME/stm32f4

and prepare its structure with the following steps to get:
<pre>
stm32f4
├── gcc-arm-none-eabi-4_8-2014q2
│   ├── arm-none-eabi
│   │   ...
│   ├── bin
│   │   ├── arm-none-eabi-addr2line
│   │   ├── arm-none-eabi-ar
│   │   ├── arm-none-eabi-as
│   │   ├── arm-none-eabi-c++
│   │   ├── arm-none-eabi-c++filt
│   │   ├── arm-none-eabi-cpp
│   │   ├── arm-none-eabi-elfedit
│   │   ├── arm-none-eabi-g++
│   │   ├── arm-none-eabi-gcc
│   │   ├── arm-none-eabi-gcc-4.8.4
│   │   ├── arm-none-eabi-gcc-ar
│   │   ├── arm-none-eabi-gcc-nm
│   │   ├── arm-none-eabi-gcc-ranlib
│   │   ├── arm-none-eabi-gcov
│   │   ├── arm-none-eabi-gdb
│   │   ├── arm-none-eabi-gprof
│   │   ├── arm-none-eabi-ld
│   │   ├── arm-none-eabi-ld.bfd
│   │   ├── arm-none-eabi-nm
│   │   ├── arm-none-eabi-objcopy
│   │   ├── arm-none-eabi-objdump
│   │   ├── arm-none-eabi-ranlib
│   │   ├── arm-none-eabi-readelf
│   │   ├── arm-none-eabi-size
│   │   ├── arm-none-eabi-strings
│   │   └── arm-none-eabi-strip
│   ├── lib
│   │   ...
│   └── share
│       ...
├── HOWTO
├── projects
│   └── 1
│       ...
├── stlink
│   ...
├── stlink_without_write (created in step 11.)
│   ...
└── STM32F4-Discovery_FW_V1.1.0
</pre>

##### 2. Example codes from ST
From STM32F4Discovery main page ( http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419 ) go to STSW-STM32068: STM32F4DISCOVERY board firmware package, including 22 examples (covering USB Host, audio, MEMS accelerometer and microphone) (AN3983) page ( http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419 ) and download STSW-STM32068 zip archive ( http://www.st.com/web/en/catalog/tools/PF257904# ).
Unpack it to get "$HOME/stm32f4/STM32F4-Discovery_FW_V1.1.0".
Add write permission to source files with

        cd $HOME/stm32f4/STM32F4-Discovery_FW_V1.1.0; find . -name "*.[chs]" -exec chmod +w {} \;

If you prefer, convert source files to have Unix line endings with 

        cd $HOME/stm32f4/STM32F4-Discovery_FW_V1.1.0; find . -name "*.[chs]" -exec dos2unix {} \;

##### 3. C cross compiler
From GNU Tools for ARM Embedded Processors page ( https://launchpad.net/gcc-arm-embedded ) download latest C compiler and debugger archive package ( https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q2-update/+download/gcc-arm-none-eabi-4_8-2014q2-20140609-linux.tar.bz2 as of 2014-07-22 ).
Unpack it to get "$HOME/stm32f4/gcc-arm-none-eabi-4_8-2014q2/". It is already precompiled and its binary tools are in its bin directory.

##### 4. Flash programmer
Install git. For Fedora it should be "yum install git" as root.
"cd $HOME/stm32f4" and download Texane's stlink software with "git clone git://github.com/texane/stlink.git".
Install pkg-config and libusb for development. For Fedora it should be "yum install pkgconfig libusb-devel" as root.
Compile st-util and st-flash in "$HOME/stm32f4/stlink" with

        ./autogen.sh
        ./configure
        make

Create "/etc/udev/rules.d/49-stlinkv2.rules" udev rule for STLINK usb device as root with following content:

        # stm32 discovery boards, with onboard st/linkv2
        # ie, STM32L, STM32F4.
        # STM32VL has st/linkv1, which is quite different
    
        SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", \
            MODE:="0666", \
            SYMLINK+="stlinkv2_%n"

        # If you share your linux system with other users, or just don't like the
        # idea of write permission for everybody, you can replace MODE:="0666" with
        # OWNER:="yourusername" to create the device owned by you, or with
        # GROUP:="somegroupname" and mange access using standard unix groups.

Use "udevadm control --reload-rules" as root to reload udev rules.

##### 5. Graphical software development environment
Install eclipse with CDT plugin. For Fedora it should be "yum install eclipse-cdt" as root.

##### 6. Prepare one example ST project
Create project directory with

        mkdir -p $HOME/stm32f4/projects/1

For easy overview and first examination I prefer copying all necessary source codes to the sample project directory.
Prepare the $HOME/stm32f4/projects/1 directory with the following steps to get:
<pre>
.
├── bin (populated at build)
│   ├── sample.bin
│   ├── sample.dis
│   ├── sample.elf
│   ├── sample.hex
│   ├── sample.isrvector
│   └── sample.map
├── linker
│   ├── stm32f4_linker_flash.ld
│   └── stm32f4_linker_ram.ld
├── lst (populated at build)
│   ├── init.lst
│   ├── main.lst
│   ├── misc.lst
│   ├── startup_stm32f4xx.lst
│   ├── stm32f4_discovery.lst
│   ├── stm32f4xx_exti.lst
│   ├── stm32f4xx_gpio.lst
│   ├── stm32f4xx_it.lst
│   ├── stm32f4xx_rcc.lst
│   ├── stm32f4xx_syscfg.lst
│   └── system_stm32f4xx.lst
├── Makefile
├── obj (populated at build)
│   ├── init.o
│   ├── main.o
│   ├── misc.o
│   ├── startup_stm32f4xx.o
│   ├── stm32f4_discovery.o
│   ├── stm32f4xx_exti.o
│   ├── stm32f4xx_gpio.o
│   ├── stm32f4xx_it.o
│   ├── stm32f4xx_rcc.o
│   ├── stm32f4xx_syscfg.o
│   └── system_stm32f4xx.o
└── src
    ├── init
    │   └── init.c
    ├── Libraries
    │   ├── CMSIS
    │   │   ├── Include
    │   │   │   ├── arm_common_tables.h
    │   │   │   ├── arm_math.h
    │   │   │   ├── core_cm0.h
    │   │   │   ├── core_cm3.h
    │   │   │   ├── core_cm4.h
    │   │   │   ├── core_cm4_simd.h
    │   │   │   ├── core_cmFunc.h
    │   │   │   └── core_cmInstr.h
    │   │   └── ST
    │   │       └── STM32F4xx
    │   │           ├── Include
    │   │           │   ├── stm32f4xx.h
    │   │           │   └── system_stm32f4xx.h
    │   │           └── Source
    │   │               └── Templates
    │   │                   └── system_stm32f4xx.c
    │   └── STM32F4xx_StdPeriph_Driver
    │       ├── inc
    │       │   ├── misc.h
    │       │   ├── stm32f4xx_adc.h
    │       │   ├── stm32f4xx_can.h
    │       │   ├── stm32f4xx_crc.h
    │       │   ├── stm32f4xx_cryp.h
    │       │   ├── stm32f4xx_dac.h
    │       │   ├── stm32f4xx_dbgmcu.h
    │       │   ├── stm32f4xx_dcmi.h
    │       │   ├── stm32f4xx_dma.h
    │       │   ├── stm32f4xx_exti.h
    │       │   ├── stm32f4xx_flash.h
    │       │   ├── stm32f4xx_fsmc.h
    │       │   ├── stm32f4xx_gpio.h
    │       │   ├── stm32f4xx_hash.h
    │       │   ├── stm32f4xx_i2c.h
    │       │   ├── stm32f4xx_iwdg.h
    │       │   ├── stm32f4xx_pwr.h
    │       │   ├── stm32f4xx_rcc.h
    │       │   ├── stm32f4xx_rng.h
    │       │   ├── stm32f4xx_rtc.h
    │       │   ├── stm32f4xx_sdio.h
    │       │   ├── stm32f4xx_spi.h
    │       │   ├── stm32f4xx_syscfg.h
    │       │   ├── stm32f4xx_tim.h
    │       │   ├── stm32f4xx_usart.h
    │       │   └── stm32f4xx_wwdg.h
    │       └── src
    │           ├── misc.c
    │           ├── stm32f4xx_exti.c
    │           ├── stm32f4xx_gpio.c
    │           ├── stm32f4xx_rcc.c
    │           └── stm32f4xx_syscfg.c
    ├── project
    │   ├── main.c
    │   ├── stm32f4xx_conf.h
    │   ├── stm32f4xx_it.c
    │   ├── stm32f4xx_it.h
    │   └── system_stm32f4xx.c
    ├── startup
    │   └── startup_stm32f4xx.s
    └── Utilities
        └── STM32F4-Discovery
            ├── stm32f4_discovery.c
            └── stm32f4_discovery.h
</pre>

1. Create project directory structure with

        cd $HOME/stm32f4/projects/1; mkdir -p bin linker lst obj src/init src/project src/startup
        
2. Copy ST external interrupt example project source codes

        cp $HOME/stm32f4/STM32F4-Discovery_FW_V1.1.0/Project/Peripheral_Examples/EXTI/*.[ch] $HOME/stm32f4/projects/1/src/project
        
3. Create empty _init() function with

        (echo 'void _init(void);'; echo 'void _init() {}') > $HOME/stm32f4/projects/1/src/init/init.c

4. Copy linker script with

         cp $HOME/stm32f4/STM32F4-Discovery_FW_V1.1.0/Project/Peripheral_Examples/EXTI/TrueSTUDIO/EXTI/stm32_flash.ld $HOME/stm32f4/projects/1/linker/stm32f4_linker_flash.ld
         
5. Copy ST source codes with

         cp -a $HOME/stm32f4/STM32F4-Discovery_FW_V1.1.0/Libraries/ $HOME/stm32f4/STM32F4-Discovery_FW_V1.1.0/Utilities/ $HOME/stm32f4/projects/1/src/
Not all source codes are needed from the Libraries and Utilities by this example. Unneeded files can be deleted from $HOME/stm32f4/project/1/src/.

6. Create $HOME/stm32f4/project/1/Makefile with following content:

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
        	$(CCPREFIX)objcopy -O binary -S $(PROJECT).elf $(PROJECT).bin
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

##### 7. Compile prepared example project

        cd $HOME/stm32f4/project/1
        make clean flash

##### 8. Program flash
Connect board, then

        $HOME/stm32f4/stlink/st-flash write $HOME/stm32f4/projects/1/bin/sample.bin 0x08000000

##### 9. Read documentation for sample project and examine its code
From STSW-STM32068: STM32F4DISCOVERY board firmware package, including 22 examples (covering USB Host, audio, MEMS accelerometer and microphone) (AN3983) page ( http://www.st.com/web/en/catalog/tools/PF257904 ) open AN3983: STM32F4DISCOVERY peripheral firmware examples pdf document ( http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00038796.pdf ). The EXTI example description is on page 5.

System key points:
- use standard header includes from cross compiler directories, like $HOME/stm32f4/gcc-arm-none-eabi-4_8-2014q2/arm-none-eabi/include,
- STM32F4Discovery board user manual ( http://www.st.com/st-web-ui/static/active/en/resource/technical/document/user_manual/DM00037368.pdf7 ) can be found on its home page ( http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419 ),
- STM32F407 chip reference manual (RM) ( http://www.st.com/web/en/resource/technical/document/reference_manual/DM00031020.pdf ), datasheet (DS) ( http://www.st.com/web/en/resource/technical/document/datasheet/DM00037051.pdf ) and other documentation can be found at ST's STM32F4 Series page ( http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577 ),
- STM32F407 chip has Cortex M4 ARM core, which has ARMv7-M architecture,
- flash is embedded in STM32F4 chip,
- flash starts at 0x0800_0000 address,
- STM32F4 can boot in several modes, depending on its BOOT0 and BOOT1 pins: flash, system memory, embedded SRAM (RM page 69.),
- if it can not be programmed via stlink then probably booting from embedded SRAM is needed, which can be forced by applying VCC to BOOT0 pin with a spare jumper from the bottom of the discovery board. Unpower board, place the jumper on BOOT0 and VCC legs of the right leg columns of the board, power the board and program it, 
- after reset 0-3 bytes of the interrupt vector table has the initial stack pointer, 4-7 bytes has the reset interrupt handler jump address,
- the interrupt vector table is located in the beginning of flash at 0x0800_0000 address,
- the interrupt vector table can be moved to SRAM,
- reset interrupt handler code is written in ./src/startup/startup_stm32f4xx.s,
- startup code initializes code initialized variables, clears uninitialized variables and sets system clock to 168 MHz which based on the external 8 MHz oscillator,
- without system clock setup STM32F4 runs in 16 MHz which is based on its internal 16 MHz oscillator,
- .isr_vector section maps the stack pointer and the interrupt handlers jump addresses,
- to use a peripheral it has to be "powered" by enabling its clocking on its bus,
- peripheral device block diagram can be found at DS page 18.,
- peripheral clocking can be enabled in RCC block. Its documentation is at RM chapter 7.,
- STM32F4XX and USE_STDPERIPH_DRIVER define macros are necessary for compilation if you want to use ST's peripheral drivers.

##### 10. Debugging in console
Connect board, then start gdb server with

        $HOME/stm32f4/stlink/st-util -v99
        
In another terminal:

        $HOME/stm32f4/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-gdb

At gdb prompt:

        target remote :4242
        load $HOME/stm32f4/projects/1/bin/sample.elf
        file $HOME/stm32f4/projects/1/bin/sample.elf
        y
        bt
        i r
        si
        quit
        y

st-util has to be restarted after each gdb disconnect, which can be automated with

        while [ 1 ]; do $HOME/stm32f4/stlink/st-util -v99; sleep 1; done

##### 11. Setting up eclipse project
1. Start eclipse.
2. File->New->Project...->C/C++->Makefile Project with Existing Code->Project Name=1,Existing Code Location=$HOME/stm32f4/projects/1,Toolchain=Cross GCC->Finish,
3. Project->Properties->C/C++ General->Paths and Symbols->Symbols->GNU C->Add->Name=STM32F4XX->Ok and Add->Name=USE_STDPERIPH_DRIVER->Ok Ok,
4. Choose Debug perspective probably with Window->Open Perspective->Debug.
5. Run->Debug Configurations...->GDB Hardware Debugging->(right click)->New->Main tab=[Project=1,C/C++ Application=bin/sample.elf],Debugger tab=[GDB Command=$HOME/stm32f4/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-gdb,JTAG device=Generic TCP/IP,Hostname=localhost,Port=4242]->Apply Debug

##### 12. Avoiding flash writes during debugging
1. Modify st-util to not to write to flash:

        cd $HOME/stm32f4
        cp -a stlink stlink_without_flash_write

2. Edit "$HOME/stm32f4/stlink_without_flash_write/gdbserver/gdb-server.c":
- Look for "FlashErase", place in comment the "flash_add_block" function call and return with OK reply for all invocation there.
- Look for "FlashWrite", place in comment the "flash_populate" function call and return with OK reply for all invocation there.

        make clean all

3. Create linker script to place everything possible to RAM and let the Reset_Handler to be at a specific location:

        cd $HOME/stm32f4/projects/1

Create "$HOME/stm32f4/projects/1/linker/stm32f4_linker_ram.ld" based on "$HOME/stm32f4/projects/1/linker/stm32f4_linker_flash.ld":
- Replace every ">FLASH" with ">RAM" except for the .isr_vector section,
- Modify the .text section to start with ".text.Reset_Handler" as:

        .text :
        {
          . = ALIGN(4);
          *(.text.Reset_Handler)
          *(.text)           /* .text sections (code) */

4. Recompile code:

        make clean ram

5. Program isr vector only into flash:

Connect board, then 

        $HOME/stm32f4/stlink/st-flash write $HOME/stm32f4/projects/1/bin/sample.isrvector 0x08000000

6. Start modified gdb server:

        $HOME/stm32f4/stlink_without_flash_write/st-util -v99

7. Debug on console or in eclipse as previously described.
