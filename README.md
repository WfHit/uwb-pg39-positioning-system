# UWB PG3.9 Positioning System

A professional-grade Ultra-Wideband (UWB) positioning system based on STM32F103CB microcontroller with DecaWave DWM3000 transceiver.

## Features

- **Advanced Positioning Algorithms**: 2D/3D positioning with multiple algorithms (Center Mass, Least Squares, Taylor Series)
- **ARM CMSIS DSP Integration**: Optimized mathematical operations for ARM Cortex-M3
- **Hardware Abstraction Layer**: Clean interface for UWB radio, UART, flash storage, and OLED display
- **Flash Configuration Storage**: Persistent configuration and calibration data
- **OLED Display Support**: Real-time system status and measurement display
- **Professional Build System**: CMake-based build with GNU ARM toolchain

## Hardware Requirements

- **MCU**: STM32F103CB (ARM Cortex-M3, 128KB Flash, 20KB RAM)
- **UWB Transceiver**: DecaWave DWM3000
- **Display**: SSD1306 OLED (128x64, I2C)
- **Crystal**: 8MHz HSE
- **Flash Memory**: Internal STM32 flash for configuration storage

## Quick Start with CMake

### 1. Prerequisites
- CMake 3.20+
- GNU ARM Embedded Toolchain
- Make tool (MinGW/MSYS2 for Windows)

### 2. Build (Cross-platform)
```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake ..

# Build
make -j4

# Generated files: uwb_pg39.elf, uwb_pg39.hex, uwb_pg39.bin
```

### 3. Easy Build Scripts
```bash
# Linux/macOS
./build.sh

# Windows
build.bat
```

## Optimized Project Structure

```
uwb_pg39/
├── CMakeLists.txt              # Modern CMake configuration
├── cmake/arm-none-eabi-gcc.cmake # ARM toolchain
├── build.sh / build.bat        # Easy build scripts
├── README.md                   # Project documentation
├── ARCHITECTURE_ANALYSIS.md   # Architecture analysis
├── BUILD.md                    # Detailed build instructions
│
├── uwb_main.c/h               # Main application entry
├── hardware_interface.c/h      # Hardware abstraction layer
│
├── core/                       # System configuration
│   ├── data_types.h           # Core data structures
│   └── system_config.h        # System configuration
│
├── modules/                    # Functional modules
│   ├── communication/         # UART protocol implementation
│   ├── positioning/           # Advanced positioning algorithms
│   ├── ranging/              # UWB ranging engine
│   ├── tag_coordinator/      # Tag management
│   ├── anchor_manager/       # Anchor discovery
│   └── utils/                # Utility functions
│
├── drivers/                   # Hardware drivers
│   └── decawave/             # DecaWave UWB driver (complete)
│       ├── deca_device.c     # ✓ Main driver implementation
│       ├── deca_device_api.h # ✓ API definitions
│       ├── deca_version.h    # ✓ Version information
│       └── platform/         # ✓ Platform abstraction
│
├── platform/                 # Platform abstraction layer
│   ├── port.c/h             # ✓ Core platform interface
│   ├── spi.h                # ✓ SPI interface
│   ├── flash/               # ✓ Flash memory management
│   ├── oled/                # ✓ OLED display driver
│   └── delay/               # ✓ Timing utilities
│
└── libraries/               # Third-party libraries
    ├── cmsis/              # ✓ ARM CMSIS (complete with startup)
    │   ├── startup_stm32f10x_md.s    # ✓ Startup file
    │   ├── STM32F103CBTx_FLASH.ld    # ✓ Linker script
    │   └── system_stm32f10x.c        # ✓ System initialization
    ├── math_lib/           # ✓ ARM CMSIS DSP library
    └── stm32f10x_std_periph_driver/  # STM32 peripheral drivers
```

## Recent Architecture Improvements

### ✅ Completed Optimizations
1. **Removed Redundant Files**: Eliminated duplicate implementations
2. **Snake Case Consistency**: All files follow `snake_case` naming
3. **Added Missing Critical Files**: DecaWave driver, startup file, linker script
4. **Updated All Include Paths**: Consistent file references
5. **Consolidated Implementations**: Single flash and OLED drivers

### 🔧 File Changes
- **Removed**: `stmflash.c/h`, `oled.c/h`, `README_new.md`
- **Renamed**: All positioning files to snake_case (e.g., `DS-TWR.c` → `ds_twr.c`)
- **Added**: `deca_device.c`, `deca_version.h`, startup file, linker script
- **Updated**: All include statements for renamed files

## CMake Features

- **Elegant Include Paths**: Automatically managed
- **GNU ARM Toolchain**: Professional cross-compilation
- **Multiple Build Types**: Debug/Release configurations
- **Static Analysis**: Built-in cppcheck support
- **Code Formatting**: clang-format integration
- **Documentation**: Doxygen support
- **Flash Target**: Integrated OpenOCD flashing

## Build Commands

| Command | Description |
|---------|-------------|
| `./build.sh` | Build release version |
| `./build.sh debug` | Build debug version |
| `./build.sh clean` | Clean build directory |
| `make flash` | Flash to target |
| `make docs` | Generate documentation |
| `make format` | Format source code |

## Development Guidelines

### Code Style
- **Naming**: `snake_case` for files and functions
- **Headers**: Include guards and documentation
- **Comments**: Doxygen-style documentation

### Build System
- **CMake**: Modern build configuration
- **Cross-compilation**: GNU ARM toolchain
- **Dependencies**: Automatically resolved

### Quality Assurance
- **Static Analysis**: cppcheck integration
- **Code Formatting**: clang-format rules
- **Documentation**: Doxygen generation

## Professional Features

- ✅ **Complete DecaWave Integration**: Full UWB driver support
- ✅ **ARM CMSIS DSP**: Optimized mathematical operations
- ✅ **Professional Build System**: CMake with toolchain files
- ✅ **Hardware Abstraction**: Clean platform interfaces
- ✅ **Memory Management**: Efficient flash and RAM usage
- ✅ **Real-time Display**: OLED system status
- ✅ **Consistent Architecture**: Snake case naming throughout

This system is now ready for professional embedded development with modern tooling, comprehensive positioning capabilities, and maintainable code structure.
