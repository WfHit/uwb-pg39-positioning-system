# UWB PG3.9 Architecture Analysis

## Current Issues Identified

### 1. Missing Critical Files
- `drivers/decawave/deca_device.c` - Main DecaWave driver implementation
- `drivers/decawave/deca_version.h` - Version information
- STM32 peripheral library sources (referenced in CMakeLists.txt but missing)
- CMSIS startup file (`startup_stm32f10x_md.s`)
- Linker script (`STM32F103CBTx_FLASH.ld`)

### 2. Unused/Redundant Files
- Multiple README files (`README_new.md` was kept as `README.md`)
- `REFACTORING_SUMMARY.md` (documentation, can be consolidated)
- Duplicate OLED implementations (`platform/oled/oled.c` vs `oled_display.c`)
- Unused flash implementation (`platform/flash/stmflash.c` vs `flash_config.c`)

### 3. Architecture Inconsistencies
- Mixed naming conventions in positioning files (CamelCase like `DS-TWR.c`)
- Inconsistent file organization
- Missing core system files that CMakeLists.txt expects

## Recommended File Architecture

### Core System Files (CRITICAL - Must Add)
```
drivers/decawave/
├── deca_device.c           # MISSING - Copy from original
├── deca_device_api.h       # ✓ Exists
├── deca_regs.h            # ✓ Exists  
├── deca_types.h           # ✓ Exists
├── deca_vals.h            # ✓ Exists
└── deca_version.h         # MISSING - Copy from original

libraries/cmsis/
├── startup_stm32f10x_md.s # MISSING - Need to create/copy
├── system_stm32f10x.c     # ✓ Exists (but may need startup file)
└── STM32F103CBTx_FLASH.ld # MISSING - Need linker script

libraries/stm32f10x_std_periph_driver/
└── src/                   # MISSING - Referenced in CMake but empty
```

### Files to Remove/Consolidate
```
REMOVE:
- REFACTORING_SUMMARY.md     # Consolidate into README
- README_new.md              # Already became README.md
- platform/flash/stmflash.c  # Use flash_config.c instead
- platform/oled/oled.c       # Use oled_display.c instead

RENAME (for consistency):
- positioning/DS-TWR.c       → positioning/ds_twr.c
- positioning/DS-TWR.h       → positioning/ds_twr.h  
- positioning/HDS-TWR.c      → positioning/hds_twr.c
- positioning/HDS-TWR.h      → positioning/hds_twr.h
- positioning/Array.c        → positioning/array.c
- positioning/Array.h        → positioning/array.h
- positioning/Twr.c          → positioning/twr.c
- positioning/Twr.h          → positioning/twr.h
- positioning/Filter.c       → positioning/filter.c
- positioning/Filter.h       → positioning/filter.h
- platform/delay/Delay.c     → platform/delay/delay.c
- platform/delay/Delay.h     → platform/delay/delay.h
```

### Optimal Project Structure
```
uwb_pg39/
├── CMakeLists.txt               # ✓ Modern build system
├── cmake/
│   └── arm-none-eabi-gcc.cmake  # ✓ ARM toolchain
├── build.sh / build.bat         # ✓ Build scripts
├── README.md                    # ✓ Comprehensive documentation
│
├── src/                         # REORGANIZE: Move main sources here
│   ├── uwb_main.c              # ✓ Main application
│   ├── uwb_main.h              # ✓ 
│   └── hardware_interface.c/h   # ✓ Hardware abstraction
│
├── modules/                     # REORGANIZE: Functional modules
│   ├── core/                   # ✓ System configuration
│   ├── communication/          # ✓ UART protocol
│   ├── positioning/            # ✓ Algorithms (rename files)
│   ├── ranging/                # ✓ UWB ranging
│   ├── tag_coordinator/        # ✓ Tag management
│   ├── anchor_manager/         # ✓ Anchor management
│   └── utils/                  # ✓ Utility functions
│
├── drivers/                    # ✓ Hardware drivers
│   └── decawave/               # ✓ UWB driver (add missing .c)
│
├── platform/                  # ✓ Platform abstraction
│   ├── flash/                  # Consolidate implementations
│   ├── oled/                   # Consolidate implementations  
│   ├── delay/                  # Rename files
│   ├── port.c/h               # ✓ Core platform
│   └── spi.h                  # ✓ SPI interface
│
└── libraries/                  # ✓ Third-party libraries
    ├── cmsis/                  # Add missing startup/linker
    ├── math_lib/               # ✓ ARM CMSIS DSP
    └── stm32f10x_std_periph_driver/ # Add peripheral sources
```

## Implementation Priority

### Phase 1: Critical Missing Files (IMMEDIATE)
1. Copy `deca_device.c` and `deca_version.h` from original project
2. Create/obtain STM32F103 startup file and linker script
3. Add essential STM32 peripheral library sources

### Phase 2: File Cleanup (HIGH)
1. Remove redundant files
2. Rename positioning files to snake_case
3. Consolidate flash and OLED implementations
4. Update CMakeLists.txt references

### Phase 3: Architecture Optimization (MEDIUM) 
1. Reorganize into src/ and modules/ structure
2. Update all include paths
3. Create comprehensive documentation

### Phase 4: Validation (HIGH)
1. Test build with new structure
2. Verify all dependencies resolve
3. Ensure no missing references

## Benefits of New Architecture

1. **Consistency**: All files follow snake_case naming
2. **Clarity**: Clear separation of application (src/), modules, drivers, platform
3. **Maintainability**: Consolidated implementations, no duplicates
4. **Buildability**: All missing critical files added
5. **Professional**: Standard embedded project structure
