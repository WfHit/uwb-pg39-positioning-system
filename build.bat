@echo off
REM Build script for UWB PG3.9 project (Windows)
REM 
REM Usage:
REM   build.bat [clean|debug|release|flash|docs]

setlocal enabledelayedexpansion

set PROJECT_DIR=%~dp0
set BUILD_DIR=%PROJECT_DIR%build

REM Parse command line arguments
set COMMAND=%1
if "%COMMAND%"=="" set COMMAND=release

goto :main

:print_status
echo [INFO] %~1
goto :eof

:print_success
echo [SUCCESS] %~1
goto :eof

:print_warning
echo [WARNING] %~1
goto :eof

:print_error
echo [ERROR] %~1
goto :eof

:check_prerequisites
call :print_status "Checking prerequisites..."

REM Check if CMake is installed
cmake --version >nul 2>&1
if !errorlevel! neq 0 (
    call :print_error "CMake is not installed. Please install CMake 3.20 or later."
    exit /b 1
)

for /f "tokens=3" %%i in ('cmake --version ^| findstr /C:"cmake version"') do (
    call :print_status "CMake version: %%i"
)

REM Check if ARM GCC toolchain is available
arm-none-eabi-gcc --version >nul 2>&1
if !errorlevel! neq 0 (
    call :print_warning "ARM GCC toolchain not found in PATH."
    call :print_warning "Please install GNU ARM Embedded Toolchain or add it to PATH."
    call :print_warning "Download from: https://developer.arm.com/downloads/-/gnu-rm"
) else (
    for /f "tokens=*" %%i in ('arm-none-eabi-gcc --version ^| findstr /N "^" ^| findstr "^1:"') do (
        set GCC_LINE=%%i
        set GCC_LINE=!GCC_LINE:~2!
        call :print_status "ARM GCC: !GCC_LINE!"
    )
)
goto :eof

:clean_build
call :print_status "Cleaning build directory..."
if exist "%BUILD_DIR%" (
    rmdir /s /q "%BUILD_DIR%"
    call :print_success "Build directory cleaned"
) else (
    call :print_status "Build directory doesn't exist, nothing to clean"
)
goto :eof

:configure_cmake
set BUILD_TYPE=%~1
call :print_status "Configuring CMake for %BUILD_TYPE% build..."

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"

cmake -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DCMAKE_TOOLCHAIN_FILE="%PROJECT_DIR%cmake\arm-none-eabi-gcc.cmake" "%PROJECT_DIR%"

if !errorlevel! equ 0 (
    call :print_success "CMake configuration completed"
) else (
    call :print_error "CMake configuration failed"
    exit /b 1
)
goto :eof

:build_project
call :print_status "Building project..."

cd /d "%BUILD_DIR%"

REM Try different make commands
where nmake >nul 2>&1
if !errorlevel! equ 0 (
    nmake
) else (
    where mingw32-make >nul 2>&1
    if !errorlevel! equ 0 (
        mingw32-make -j4
    ) else (
        where make >nul 2>&1
        if !errorlevel! equ 0 (
            make -j4
        ) else (
            call :print_error "No suitable make program found. Please install MinGW, MSYS2, or Visual Studio."
            exit /b 1
        )
    )
)

if !errorlevel! equ 0 (
    call :print_success "Build completed successfully"
    call :print_status "Generated files:"
    dir *.elf *.hex *.bin 2>nul
) else (
    call :print_error "Build failed"
    exit /b 1
)
goto :eof

:flash_target
call :print_status "Flashing target..."

cd /d "%BUILD_DIR%"
if exist "uwb_pg39.hex" (
    make flash
) else (
    call :print_error "No hex file found. Build the project first."
    exit /b 1
)
goto :eof

:build_docs
call :print_status "Building documentation..."

cd /d "%BUILD_DIR%"
make docs >nul 2>&1
if !errorlevel! equ 0 (
    call :print_success "Documentation built successfully"
) else (
    call :print_warning "Documentation build failed. Doxygen might not be installed."
)
goto :eof

:show_help
echo UWB PG3.9 Build Script (Windows)
echo.
echo Usage: %~nx0 [COMMAND]
echo.
echo Commands:
echo   clean     Clean build directory
echo   debug     Build debug version
echo   release   Build release version (default)
echo   flash     Flash the target device
echo   docs      Build documentation
echo   help      Show this help message
echo.
echo Examples:
echo   %~nx0              # Build release version
echo   %~nx0 debug        # Build debug version
echo   %~nx0 clean debug  # Clean and build debug
echo.
goto :eof

:main
if "%COMMAND%"=="clean" (
    call :clean_build
    if "%~2" neq "" (
        set COMMAND=%~2
        goto :process_command
    )
    goto :end
)

:process_command
if "%COMMAND%"=="debug" (
    call :check_prerequisites
    call :configure_cmake "Debug"
    call :build_project
) else if "%COMMAND%"=="release" (
    call :check_prerequisites
    call :configure_cmake "Release"
    call :build_project
) else if "%COMMAND%"=="flash" (
    call :flash_target
) else if "%COMMAND%"=="docs" (
    call :build_docs
) else if "%COMMAND%"=="help" (
    call :show_help
) else if "%COMMAND%"=="--help" (
    call :show_help
) else if "%COMMAND%"=="-h" (
    call :show_help
) else (
    call :print_error "Unknown command: %COMMAND%"
    call :show_help
    exit /b 1
)

:end
cd /d "%PROJECT_DIR%"
