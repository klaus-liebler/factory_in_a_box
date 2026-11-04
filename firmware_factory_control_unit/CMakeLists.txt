# Factory Control Unit firmware on rtoos (GCC 16, C++26, STM32H563).
#
#   cmake --preset Release          (or Debug, Release-Rev1, Debug-Rev1: hardware revision)
#   cmake --build --preset Release
#
# The builder (builder/, docs/build-process.md) provides what the firmware embeds:
#   build/assets/   certificates and keys of the board (device_https_*, device_certificate.der,
#                   device_key.der, root_ca.der), the web app (index.html.br)
#   generated/ device identity (device_ids.hh), register map (register_map*.hpp),
#                   WebSocket protocol (ws_protocol.hh), version and git constants
cmake_minimum_required(VERSION 3.28)
project(firmware_factory_control_unit LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(RTOS_SSH OFF)  # no SSH log terminal: wolfSSL without Ed25519/SHA-512
add_subdirectory(rtoos)  # rtos_kernel, rtos_tls, rtos_opcua, rtos_filex, rtos_littlefs, rtos_board_fcu

set(FCU_ASSETS ${CMAKE_SOURCE_DIR}/build/assets)
set(FCU_GENERATED ${CMAKE_SOURCE_DIR}/generated)
foreach(f device_https_certificate.der device_https_key.der device_certificate.der device_key.der root_ca.der
          index.html.br)
    if(NOT EXISTS ${FCU_ASSETS}/${f})
        message(FATAL_ERROR "build/assets/${f} missing: run the builder first (dotnet run --project builder -- PipelineLazy)")
    endif()
endforeach()

file(GLOB_RECURSE FCU_SOURCES CONFIGURE_DEPENDS src/*.cpp)

# The last 256 KB of flash (sectors of bank 2) hold the RoArm missions (littlefs).
rtos_add_firmware(firmware_factory_control_unit
    SOURCES ${FCU_SOURCES}
    LIBRARIES rtos_board_fcu rtos_tls rtos_opcua rtos_filex rtos_littlefs
    FLASH_DATA_START 0x081C0000)
target_include_directories(firmware_factory_control_unit PRIVATE src ${FCU_GENERATED})
target_compile_options(firmware_factory_control_unit PRIVATE -Wall -Wextra -Werror --embed-dir=${FCU_ASSETS})
