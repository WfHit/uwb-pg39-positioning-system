# CMake toolchain file for ARM Cortex-M development with GNU ARM Embedded Toolchain
# 
# Usage:
# cmake -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake ..

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain settings
set(TOOLCHAIN_PREFIX arm-none-eabi-)

# Define toolchain paths
if(WIN32)
    set(TOOLCHAIN_EXT ".exe")
else()
    set(TOOLCHAIN_EXT "")
endif()

# Search for the toolchain in common locations
find_program(TOOLCHAIN_CC 
    NAMES ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXT}
    PATHS
        # Windows common paths
        "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/bin"
        "C:/Program Files/GNU Arm Embedded Toolchain/*/bin"
        "C:/arm-gnu-toolchain/*/bin"
        # Linux/macOS paths
        /usr/bin
        /usr/local/bin
        /opt/gcc-arm-none-eabi/bin
        # User-installed paths
        $ENV{HOME}/.local/bin
        $ENV{ARM_TOOLCHAIN_PATH}/bin
    DOC "ARM GCC Compiler"
)

if(NOT TOOLCHAIN_CC)
    message(FATAL_ERROR "ARM GCC toolchain not found. Please install GNU ARM Embedded Toolchain and ensure it's in your PATH or set ARM_TOOLCHAIN_PATH environment variable.")
endif()

# Get toolchain directory
get_filename_component(TOOLCHAIN_BIN_DIR ${TOOLCHAIN_CC} DIRECTORY)
get_filename_component(TOOLCHAIN_DIR ${TOOLCHAIN_BIN_DIR} DIRECTORY)

message(STATUS "ARM Toolchain found at: ${TOOLCHAIN_DIR}")

# Set compilers
set(CMAKE_C_COMPILER ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXT})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_EXT})
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXT})

# Set utilities
set(CMAKE_OBJCOPY ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_EXT})
set(CMAKE_OBJDUMP ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}objdump${TOOLCHAIN_EXT})
set(CMAKE_SIZE ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}size${TOOLCHAIN_EXT})
set(CMAKE_AR ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}ar${TOOLCHAIN_EXT})
set(CMAKE_RANLIB ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}ranlib${TOOLCHAIN_EXT})
set(CMAKE_STRIP ${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}strip${TOOLCHAIN_EXT})

# Configure for cross-compilation
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_DIR})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Don't run the linker during compiler checks
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Compiler test flags
set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m3 -mthumb")
set(CMAKE_CXX_FLAGS_INIT "-mcpu=cortex-m3 -mthumb")
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m3 -mthumb")

# Prevent CMake from adding -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# Print toolchain info
message(STATUS "ARM GCC Toolchain Configuration:")
message(STATUS "  Toolchain Directory: ${TOOLCHAIN_DIR}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  ASM Compiler: ${CMAKE_ASM_COMPILER}")
message(STATUS "  Objcopy: ${CMAKE_OBJCOPY}")
message(STATUS "  Size: ${CMAKE_SIZE}")

# Check if compiler works
execute_process(
    COMMAND ${CMAKE_C_COMPILER} --version
    OUTPUT_VARIABLE GCC_VERSION_OUTPUT
    ERROR_QUIET
)

if(GCC_VERSION_OUTPUT MATCHES "gcc version ([0-9]+\\.[0-9]+\\.[0-9]+)")
    set(GCC_VERSION ${CMAKE_MATCH_1})
    message(STATUS "  GCC Version: ${GCC_VERSION}")
else()
    message(WARNING "Could not determine GCC version")
endif()
