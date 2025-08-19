# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH "C:/Users/malub/FreeRTOS-Kernel")
include(C:/Users/malub/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(SensoresCorELuminosidade C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories( ${CMAKE_SOURCE_DIR}/lib )

# Add executable. Default name is the project name, version 0.1

add_executable(SensoresCorELuminosidade SensoresCorELuminosidade.c lib/ssd1306.c lib/bh1750_light_sensor.c)

pico_set_program_name(SensoresCorELuminosidade "SensoresCorELuminosidade")
pico_set_program_version(SensoresCorELuminosidade "0.1")

# Generate PIO header
pico_generate_pio_header(SensoresCorELuminosidade ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(SensoresCorELuminosidade 0)
pico_enable_stdio_usb(SensoresCorELuminosidade 0)

# Add the standard library to the build
target_link_libraries(SensoresCorELuminosidade
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(SensoresCorELuminosidade PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(SensoresCorELuminosidade 
        hardware_i2c
        hardware_pio
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        )

pico_add_extra_outputs(SensoresCorELuminosidade)

