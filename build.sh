#!/bin/bash
# Build script for UWB PG3.9 project
# 
# Usage:
#   ./build.sh [clean|debug|release|flash|docs]

set -e  # Exit on any error

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Functions
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_prerequisites() {
    print_status "Checking prerequisites..."
    
    # Check if CMake is installed
    if ! command -v cmake &> /dev/null; then
        print_error "CMake is not installed. Please install CMake 3.20 or later."
        exit 1
    fi
    
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d" " -f3)
    print_status "CMake version: $CMAKE_VERSION"
    
    # Check if ARM GCC toolchain is available
    if ! command -v arm-none-eabi-gcc &> /dev/null; then
        print_warning "ARM GCC toolchain not found in PATH."
        print_warning "Please install GNU ARM Embedded Toolchain or set ARM_TOOLCHAIN_PATH environment variable."
        print_warning "Download from: https://developer.arm.com/downloads/-/gnu-rm"
    else
        GCC_VERSION=$(arm-none-eabi-gcc --version | head -n1)
        print_status "ARM GCC: $GCC_VERSION"
    fi
}

clean_build() {
    print_status "Cleaning build directory..."
    if [ -d "$BUILD_DIR" ]; then
        rm -rf "$BUILD_DIR"
        print_success "Build directory cleaned"
    else
        print_status "Build directory doesn't exist, nothing to clean"
    fi
}

configure_cmake() {
    local build_type=$1
    
    print_status "Configuring CMake for $build_type build..."
    
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    cmake \
        -DCMAKE_BUILD_TYPE="$build_type" \
        -DCMAKE_TOOLCHAIN_FILE="${PROJECT_DIR}/cmake/arm-none-eabi-gcc.cmake" \
        "$PROJECT_DIR"
    
    if [ $? -eq 0 ]; then
        print_success "CMake configuration completed"
    else
        print_error "CMake configuration failed"
        exit 1
    fi
}

build_project() {
    print_status "Building project..."
    
    cd "$BUILD_DIR"
    make -j$(nproc 2>/dev/null || echo 4)
    
    if [ $? -eq 0 ]; then
        print_success "Build completed successfully"
        print_status "Generated files:"
        ls -la *.elf *.hex *.bin 2>/dev/null || true
    else
        print_error "Build failed"
        exit 1
    fi
}

flash_target() {
    print_status "Flashing target..."
    
    cd "$BUILD_DIR"
    if [ -f "uwb_pg39.hex" ]; then
        make flash
    else
        print_error "No hex file found. Build the project first."
        exit 1
    fi
}

build_docs() {
    print_status "Building documentation..."
    
    cd "$BUILD_DIR"
    if make docs 2>/dev/null; then
        print_success "Documentation built successfully"
    else
        print_warning "Documentation build failed. Doxygen might not be installed."
    fi
}

show_help() {
    echo "UWB PG3.9 Build Script"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  clean     Clean build directory"
    echo "  debug     Build debug version"
    echo "  release   Build release version (default)"
    echo "  flash     Flash the target device"
    echo "  docs      Build documentation"
    echo "  help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0              # Build release version"
    echo "  $0 debug        # Build debug version"
    echo "  $0 clean debug  # Clean and build debug"
    echo ""
}

# Main script logic
main() {
    local command=${1:-release}
    
    case "$command" in
        clean)
            clean_build
            ;;
        debug)
            check_prerequisites
            configure_cmake "Debug"
            build_project
            ;;
        release)
            check_prerequisites
            configure_cmake "Release"
            build_project
            ;;
        flash)
            flash_target
            ;;
        docs)
            build_docs
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            if [ "$command" == "clean" ] && [ -n "$2" ]; then
                clean_build
                main "$2"
            else
                print_error "Unknown command: $command"
                show_help
                exit 1
            fi
            ;;
    esac
}

# Run main function with all arguments
main "$@"
