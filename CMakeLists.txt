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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")
# Define uma variável personalizada
set(SSID_HOST "OMS OI FIBRA")
# Define uma variável personalizada
set(PASSWORD_HOST "oms29916")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PF_2_embarcatech C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
#pico_set_program_name(PF_2_embarcatech "PF_2_embarcatech")
#pico_set_program_version(PF_2_embarcatech "0.1")

add_executable(PF_2_embarcatech PF_2_embarcatech.c display_oled.c inc/ssd1306_i2c.c)

target_compile_definitions(PF_2_embarcatech PRIVATE 
    SSID_HOST="${SSID_HOST}" 
    PASSWORD_HOST="${PASSWORD_HOST}"
    )

# Generate PIO header
pico_generate_pio_header(PF_2_embarcatech ${CMAKE_CURRENT_LIST_DIR}/ws2818b.pio)

# Add any user requested libraries
target_link_libraries(PF_2_embarcatech 
    pico_stdlib
        )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PF_2_embarcatech 0)
pico_enable_stdio_usb(PF_2_embarcatech 1)


# Add the standard include files to the build
target_include_directories(PF_2_embarcatech PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(PF_2_embarcatech
        pico_cyw43_arch_lwip_threadsafe_background
        hardware_i2c
        hardware_dma
        hardware_timer
        hardware_adc
        hardware_pio
        hardware_clocks
)

pico_add_extra_outputs(PF_2_embarcatech)