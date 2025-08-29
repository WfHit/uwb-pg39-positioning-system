# Build Configuration for UWB PG3.9 Project

## Required Files

### Core Application Files
```
uwb_main.c
hardware_interface.c
hardware_interface.h
```

### Core Modules
```
core/system_config.h
core/data_types.h
tag_coordinator/tag_coordinator.c
tag_coordinator/tag_coordinator.h
anchor_manager/anchor_manager.c
anchor_manager/anchor_manager.h
communication/uart_protocol.c
communication/uart_protocol.h
ranging/ranging_engine.c
ranging/ranging_engine.h
utils/geometry_utils.c
utils/geometry_utils.h
utils/timing_utils.c
utils/timing_utils.h
```

### Positioning Algorithms
```
positioning/loc.c
positioning/loc.h
positioning/Array.c
positioning/Array.h
positioning/DS-TWR.c
positioning/DS-TWR.h
positioning/HDS-TWR.c
positioning/HDS-TWR.h
positioning/Twr.c
positioning/Twr.h
positioning/Filter.c
positioning/Filter.h
positioning/positioning_adapter.c
positioning/positioning_adapter.h
```

### Platform and Drivers
```
platform/port.c
platform/port.h
platform/spi.h
platform/flash/flash_config.c
platform/flash/flash_config.h
platform/oled/oled_display.c
platform/oled/oled_display.h
platform/delay/Delay.c
platform/delay/Delay.h
drivers/decawave/deca_device_api.h
drivers/decawave/deca_device.c
drivers/decawave/deca_types.h
drivers/decawave/deca_regs.h
drivers/decawave/deca_vals.h
drivers/decawave/platform/deca_spi.c
drivers/decawave/platform/deca_spi.h
drivers/decawave/platform/deca_mutex.c
drivers/decawave/platform/deca_sleep.c
```

### STM32 Libraries
```
libraries/cmsis/core_cm3.h
libraries/cmsis/stm32f10x.h
libraries/cmsis/stm32f10x_conf.h
libraries/cmsis/system_stm32f10x.h
libraries/math_lib/arm_cortexM3l_math.lib
libraries/math_lib/include/arm_math.h
libraries/math_lib/include/arm_common_tables.h
libraries/math_lib/include/arm_const_structs.h
libraries/stm32f10x_std_periph_driver/inc/stm32f10x_rcc.h
libraries/stm32f10x_std_periph_driver/inc/stm32f10x_gpio.h
libraries/stm32f10x_std_periph_driver/inc/stm32f10x_spi.h
libraries/stm32f10x_std_periph_driver/inc/stm32f10x_usart.h
libraries/stm32f10x_std_periph_driver/inc/stm32f10x_exti.h
Libraries/STM32F10x_StdPeriph_Driver/inc/misc.h
```

## Include Paths
```
-I.
-ICore
-ITagCoordinator
-IAnchorManager
-ICommunication
-IRanging
-IUtils
-IPlatform
-IPlatform/Delay
-IDrivers/Decawave
-IDrivers/Decawave/Platform
-ILibraries/CMSIS
-ILibraries/STM32F10x_StdPeriph_Driver/inc
```

## Compiler Defines
```
-DSTM32F10X_MD          # Medium density STM32F103
-DUSE_STDPERIPH_DRIVER  # Use STM32 Standard Peripheral Library
-DHSE_VALUE=8000000     # External crystal frequency (8MHz)
-DSYSCLK_FREQ_72MHz     # System clock frequency
```

## GCC Compiler Flags
```
-mcpu=cortex-m3
-mthumb
-mfloat-abi=soft
-Wall
-Wextra
-O2
-g
-ffunction-sections
-fdata-sections
```

## Linker Flags
```
-mcpu=cortex-m3
-mthumb
-specs=nano.specs
-specs=nosys.specs
-Wl,--gc-sections
-Wl,--print-memory-usage
```

## Sample Makefile Structure
```makefile
# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE = $(PREFIX)size

# Project name
PROJECT = uwb_pg39

# Source directories
SRCDIR = .
COREDIR = core
TAGDIR = tag_coordinator
ANCHORDIR = anchor_manager
COMMDIR = communication
RANGEDIR = ranging
UTILSDIR = utils
PLATDIR = platform
POSDIR = positioning
DRVDIR = drivers/decawave

# Source files
SOURCES = uwb_main.c hardware_interface.c
SOURCES += $(TAGDIR)/tag_coordinator.c
SOURCES += $(ANCHORDIR)/anchor_manager.c
SOURCES += $(COMMDIR)/uart_protocol.c
SOURCES += $(RANGEDIR)/ranging_engine.c
SOURCES += $(UTILSDIR)/geometry_utils.c
SOURCES += $(UTILSDIR)/timing_utils.c
SOURCES += $(PLATDIR)/port.c
SOURCES += $(PLATDIR)/flash/flash_config.c
SOURCES += $(PLATDIR)/oled/oled_display.c
SOURCES += $(PLATDIR)/delay/Delay.c
SOURCES += $(POSDIR)/loc.c
SOURCES += $(POSDIR)/Array.c
SOURCES += $(POSDIR)/DS-TWR.c
SOURCES += $(POSDIR)/HDS-TWR.c
SOURCES += $(POSDIR)/Twr.c
SOURCES += $(POSDIR)/Filter.c
SOURCES += $(POSDIR)/positioning_adapter.c
SOURCES += $(DRVDIR)/deca_device.c
SOURCES += $(DRVDIR)/platform/deca_spi.c
SOURCES += $(DRVDIR)/platform/deca_mutex.c
SOURCES += $(DRVDIR)/platform/deca_sleep.c

# Include paths
INCLUDES = -I$(SRCDIR)
INCLUDES += -I$(COREDIR)
INCLUDES += -I$(TAGDIR)
INCLUDES += -I$(ANCHORDIR)
INCLUDES += -I$(COMMDIR)
INCLUDES += -I$(RANGEDIR)
INCLUDES += -I$(UTILSDIR)
INCLUDES += -I$(PLATDIR)
INCLUDES += -I$(PLATDIR)/flash
INCLUDES += -I$(PLATDIR)/oled
INCLUDES += -I$(PLATDIR)/delay
INCLUDES += -I$(POSDIR)
INCLUDES += -I$(DRVDIR)
INCLUDES += -I$(DRVDIR)/platform
INCLUDES += -Ilibraries/cmsis
INCLUDES += -Ilibraries/math_lib/include
INCLUDES += -Ilibraries/stm32f10x_std_periph_driver/inc

# Compiler definitions
DEFS = -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000UL -DARM_MATH_CM3

# Libraries
LIBS = -Llibraries/math_lib -larm_cortexM3l_math

# Compiler flags
CFLAGS = -mcpu=cortex-m3 -mthumb -Wall -Wextra -O2 -g
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += $(DEFS) $(INCLUDES)

# Linker flags
LDFLAGS = -mcpu=cortex-m3 -mthumb -specs=nano.specs -specs=nosys.specs
LDFLAGS += -Wl,--gc-sections -Wl,--print-memory-usage
LDFLAGS += $(LIBS)

# Build targets
all: $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(PROJECT).elf: $(SOURCES:.c=.o)
	$(CC) $(LDFLAGS) $^ -o $@
	$(SIZE) $@

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f *.o *.elf *.hex *.bin
	rm -f $(TAGDIR)/*.o
	rm -f $(ANCHORDIR)/*.o
	rm -f $(COMMDIR)/*.o
	rm -f $(RANGEDIR)/*.o
	rm -f $(UTILSDIR)/*.o
	rm -f $(PLATDIR)/*.o
	rm -f $(PLATDIR)/flash/*.o
	rm -f $(PLATDIR)/oled/*.o
	rm -f $(PLATDIR)/delay/*.o
	rm -f $(POSDIR)/*.o
	rm -f $(DRVDIR)/*.o
	rm -f $(DRVDIR)/platform/*.o

.PHONY: all clean
```

## Implementation Notes

### Still Needed
1. **STM32 peripheral source files**: The .c files for the STM32 standard peripheral library
2. **Linker script**: Memory layout definition for STM32F103CB
3. **Startup file**: STM32F103 startup assembly code

### Completed Components
1. **hardware_interface.c**: Complete hardware abstraction layer implementation
2. **port.c**: Complete STM32 port implementation with all peripheral configurations
3. **flash_config.c/h**: Internal flash memory management for configuration storage
4. **oled_display.c/h**: SSD1306 OLED display driver with I2C communication
5. **positioning algorithms**: Complete TWR positioning implementation with ARM CMSIS DSP
6. **positioning_adapter.c/h**: Bridge layer for integrating positioning algorithms
7. **ARM CMSIS DSP library**: Mathematical functions for positioning calculations

### Positioning Algorithms Available
1. **2D Center Mass**: Fast 3-anchor positioning for simple scenarios
2. **2D/3D Least Squares**: Multi-anchor positioning with error minimization
3. **2D/3D Taylor Series**: Iterative positioning for high accuracy
4. **DS-TWR/HDS-TWR**: Time-of-flight ranging protocols
5. **Distance Filtering**: Noise reduction and outlier rejection

### Optional Additions
1. **Configuration management**: Flash-based configuration storage
2. **Debug output**: UART-based debug logging
3. **Bootloader support**: Firmware update capability
4. **Power management**: Low-power modes for battery operation
