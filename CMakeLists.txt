cmake_minimum_required(VERSION 3.20)

# Set the toolchain before project() declaration
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)

# Project configuration
project(uwb_pg39 
    VERSION 1.0.0
    DESCRIPTION "UWB PG3.9 Positioning System"
    LANGUAGES C ASM
)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# MCU Configuration
set(MCU_FAMILY STM32F10X_MD)
set(MCU_SERIES STM32F103)
set(MCU_PART STM32F103CB)
set(HSE_VALUE 8000000UL)

#==============================================================================
# Compiler and Linker Settings
#==============================================================================

# Compiler flags
set(CMAKE_C_FLAGS 
    "-mcpu=cortex-m3 -mthumb -Wall -Wextra -fdata-sections -ffunction-sections"
)

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -g0 -DNDEBUG")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS
    "-mcpu=cortex-m3 -mthumb -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage -T${CMAKE_CURRENT_SOURCE_DIR}/libraries/cmsis/STM32F103CBTx_FLASH.ld"
)

# Additional definitions
add_compile_definitions(
    ${MCU_FAMILY}
    USE_STDPERIPH_DRIVER
    HSE_VALUE=${HSE_VALUE}
    ARM_MATH_CM3
)

#==============================================================================
# Source Files Organization
#==============================================================================

# Core application sources
set(CORE_SOURCES
    uwb_main.c
    hardware_interface.c
)

# Core module sources
file(GLOB_RECURSE CORE_MODULE_SOURCES
    "core/*.c"
    "tag_coordinator/*.c"
    "anchor_manager/*.c"
    "communication/*.c"
    "ranging/*.c"
    "utils/*.c"
)

# Platform sources
file(GLOB_RECURSE PLATFORM_SOURCES
    "platform/port.c"
    "platform/flash/*.c"
    "platform/oled/*.c"
    "platform/delay/*.c"
)

# Positioning algorithm sources
file(GLOB_RECURSE POSITIONING_SOURCES
    "positioning/*.c"
)

# DecaWave driver sources
file(GLOB_RECURSE DECAWAVE_SOURCES
    "drivers/decawave/*.c"
)

# STM32 peripheral driver sources (when available)
file(GLOB_RECURSE STM32_PERIPH_SOURCES
    "libraries/stm32f10x_std_periph_driver/src/*.c"
)

# CMSIS sources
set(CMSIS_SOURCES
    libraries/cmsis/system_stm32f10x.c
    libraries/cmsis/startup_stm32f10x_md.s
)

# Combine all sources
set(ALL_SOURCES
    ${CORE_SOURCES}
    ${CORE_MODULE_SOURCES}
    ${PLATFORM_SOURCES}
    ${POSITIONING_SOURCES}
    ${DECAWAVE_SOURCES}
    ${STM32_PERIPH_SOURCES}
    ${CMSIS_SOURCES}
)

#==============================================================================
# Include Directories
#==============================================================================

# Project include directories
set(PROJECT_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    core
    tag_coordinator
    anchor_manager
    communication
    ranging
    utils
    platform
    platform/flash
    platform/oled
    platform/delay
    positioning
    drivers/decawave
    drivers/decawave/platform
)

# Library include directories
set(LIBRARY_INCLUDES
    libraries/cmsis
    libraries/math_lib/include
    libraries/stm32f10x_std_periph_driver/inc
)

# Combine all includes
set(ALL_INCLUDES
    ${PROJECT_INCLUDES}
    ${LIBRARY_INCLUDES}
)

#==============================================================================
# Library Configuration
#==============================================================================

# ARM CMSIS DSP Library
set(ARM_MATH_LIB ${CMAKE_CURRENT_SOURCE_DIR}/libraries/math_lib/arm_cortexM3l_math.lib)

# Check if ARM math library exists
if(EXISTS ${ARM_MATH_LIB})
    message(STATUS "ARM CMSIS DSP Library found: ${ARM_MATH_LIB}")
    set(EXTERNAL_LIBS ${ARM_MATH_LIB})
else()
    message(WARNING "ARM CMSIS DSP Library not found at: ${ARM_MATH_LIB}")
    set(EXTERNAL_LIBS "")
endif()

#==============================================================================
# Target Definition
#==============================================================================

# Create the executable target
add_executable(${PROJECT_NAME} ${ALL_SOURCES})

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${PROJECT_NAME}"
    SUFFIX ".elf"
)

# Add include directories to target
target_include_directories(${PROJECT_NAME} PRIVATE ${ALL_INCLUDES})

# Link libraries
if(EXTERNAL_LIBS)
    target_link_libraries(${PROJECT_NAME} ${EXTERNAL_LIBS})
endif()

# Add math library
target_link_libraries(${PROJECT_NAME} m)

#==============================================================================
# Custom Targets for Output Files
#==============================================================================

# Generate HEX file
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMENT "Generating HEX file"
)

# Generate BIN file
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMENT "Generating BIN file"
)

# Show size information
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Memory usage:"
)

#==============================================================================
# Flash and Debug Targets
#==============================================================================

# Flash target (adjust for your programmer)
add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program ${PROJECT_NAME}.elf verify reset exit"
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing the target"
)

# Debug target
add_custom_target(debug
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f1x.cfg
    COMMENT "Starting OpenOCD debug server"
)

#==============================================================================
# Print Configuration Summary
#==============================================================================

message(STATUS "=== UWB PG3.9 Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "MCU: ${MCU_PART}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C Flags: ${CMAKE_C_FLAGS}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Debug Flags: ${CMAKE_C_FLAGS_DEBUG}")
else()
    message(STATUS "Release Flags: ${CMAKE_C_FLAGS_RELEASE}")
endif()

message(STATUS "Linker Flags: ${CMAKE_EXE_LINKER_FLAGS}")
message(STATUS "External Libraries: ${EXTERNAL_LIBS}")
message(STATUS "=====================================")

#==============================================================================
# Optional: Doxygen Documentation
#==============================================================================

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    
    if(EXISTS ${DOXYGEN_IN})
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        
        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    endif()
endif()

#==============================================================================
# Static Analysis Targets (Optional)
#==============================================================================

# Cppcheck target
find_program(CPPCHECK_EXECUTABLE cppcheck)
if(CPPCHECK_EXECUTABLE)
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK_EXECUTABLE}
        --enable=warning,performance,portability,information,missingInclude
        --std=c11
        --verbose
        --quiet
        ${ALL_INCLUDES}
        ${ALL_SOURCES}
        COMMENT "Running static analysis with cppcheck"
    )
endif()

# Clang-format target
find_program(CLANG_FORMAT_EXECUTABLE clang-format)
if(CLANG_FORMAT_EXECUTABLE)
    add_custom_target(format
        COMMAND ${CLANG_FORMAT_EXECUTABLE}
        -i
        -style=file
        ${ALL_SOURCES}
        COMMENT "Formatting source code with clang-format"
    )
endif()
