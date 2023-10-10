# cmake-format: off
cmake_minimum_required(VERSION 3.17.0)

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Must setup the PICO SDK before the project macro call
include(cmake/pico-sdk.cmake)

project(smart-garage C CXX ASM)
string(TIMESTAMP BUILD_TIME "%s")

###################
## CONFIGURATION ##
###################

initialize_pico()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED BUILD_VERSION)
    set(BUILD_VERSION "0.0.0+${BUILD_TIME}")
endif()

if(NOT DEFINED SYSTEM_LED_PIN)
    set(SYSTEM_LED_PIN 13)
endif()

if(NOT DEFINED I2C_SCL_PIN)
    set(I2C_SCL_PIN 5)
endif()

if(NOT DEFINED I2C_SDA_PIN)
    set(I2C_SDA_PIN 4)
endif()

if(NOT DEFINED MQTT_PORT)
    set(MQTT_PORT 1883)
endif()

if(NOT DEFINED MQTT_FEEDBACK_PIN)
    set(MQTT_FEEDBACK_PIN 14)
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/generated/configuration.hpp.in
    ${CMAKE_BINARY_DIR}/generated/configuration.hpp
    @ONLY
)

############
##  MQTT  ##
############

add_subdirectory(pico-mqtt)

#############
##  BUILD  ##
#############

add_executable(${PROJECT_NAME})

target_compile_options(
    ${PROJECT_NAME}
    PRIVATE
        -Wall -Werror -Wno-format -Wno-unused-function -Wno-maybe-uninitialized
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/connectivity/detail
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/detail/core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/detail/platform/include
        ${CMAKE_CURRENT_SOURCE_DIR}/build
        ${CMAKE_CURRENT_SOURCE_DIR}/pico-mqtt/include
)

target_sources(
    ${PROJECT_NAME}
    PRIVATE
        # STM VL53L4CX code
        src/sensors/detail/core/src/vl53lx_api.c
        src/sensors/detail/core/src/vl53lx_api_calibration.c
        src/sensors/detail/core/src/vl53lx_api_core.c
        src/sensors/detail/core/src/vl53lx_api_debug.c
        src/sensors/detail/core/src/vl53lx_api_preset_modes.c
        src/sensors/detail/core/src/vl53lx_core_support.c
        src/sensors/detail/core/src/vl53lx_core.c
        src/sensors/detail/core/src/vl53lx_dmax.c
        src/sensors/detail/core/src/vl53lx_hist_algos_gen3.c
        src/sensors/detail/core/src/vl53lx_hist_algos_gen4.c
        src/sensors/detail/core/src/vl53lx_hist_char.c
        src/sensors/detail/core/src/vl53lx_hist_core.c
        src/sensors/detail/core/src/vl53lx_hist_funcs.c
        src/sensors/detail/core/src/vl53lx_nvm_debug.c
        src/sensors/detail/core/src/vl53lx_nvm.c
        src/sensors/detail/core/src/vl53lx_register_funcs.c
        src/sensors/detail/core/src/vl53lx_sigma_estimate.c
        src/sensors/detail/core/src/vl53lx_silicon_core.c
        src/sensors/detail/core/src/vl53lx_wait.c
        src/sensors/detail/core/src/vl53lx_xtalk.c

        # Pico Platform code for VL53l4CX
        src/sensors/detail/platform/src/vl53lx_platform_ipp.c
        src/sensors/detail/platform/src/vl53lx_platform_log.c
        src/sensors/detail/platform/src/vl53lx_platform.cpp

        src/connectivity/wireless/connection-status.cpp
        src/connectivity/wireless/wifi-connection.cpp

        src/controllers/door-status.cpp
        src/controllers/door.cpp

        src/sensors/vl53l4cx.cpp

        src/utilities.cpp
        src/main.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    pico_multicore
    pico_mqtt
    pico_stdlib
    pico_unique_id
    hardware_adc
    hardware_i2c
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_add_extra_outputs(${PROJECT_NAME})
# cmake-format: on
