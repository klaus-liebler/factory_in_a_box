# Factory Control Unit firmware on rtoos (GCC 16, C++26): STM32H563 (hardware revisions 1/2),
# STM32H743 (revision 3).
#
#   cmake --preset rev3             (or rev1, rev2; rev1-debug, rev2-debug, rev3-debug)
#   cmake --build --preset rev3
#
# Each board revision has its HAL in src/hal/<revision> (the preset's revision = the directory
# name, FCU_HAL): mcu.h, pins.hpp, peripherals.hpp, its own sources, and hal.cmake, which sets
# the chip and rtoos's board revision before project() - the toolchain's CPU flags follow it.
#
# The builder (builder/, docs/build-process.md) provides what the firmware embeds:
#   build/assets/   certificates and keys of the board (device_https_*, device_certificate.der,
#                   device_key.der, root_ca.der), the web app (index.html.br)
#   generated/ device identity (device_ids.hh), register map (register_map*.hpp),
#                   WebSocket protocol (ws_protocol.hh), version and git constants
cmake_minimum_required(VERSION 3.28)

set(FCU_HAL "" CACHE STRING "Board revision: its HAL is src/hal/<FCU_HAL> (set by the CMake preset)")
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/src/hal/${FCU_HAL}/hal.cmake)
    message(FATAL_ERROR "FCU_HAL '${FCU_HAL}': no src/hal/${FCU_HAL}/hal.cmake - configure with a preset (cmake --list-presets)")
endif()
include(${CMAKE_SOURCE_DIR}/src/hal/${FCU_HAL}/hal.cmake)  # RTOS_CHIP, RTOS_FCU_REVISION, FCU_HAL_SOURCES

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

# The common sources; of src/hal and src/features only what the revision's hal.cmake lists.
file(GLOB_RECURSE FCU_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(FILTER FCU_SOURCES EXCLUDE REGEX "/src/(hal|features)/")
list(APPEND FCU_SOURCES ${FCU_HAL_SOURCES})
message(STATUS "FCU board revision ${FCU_HAL}: ${RTOS_CHIP}, rtoos board revision ${RTOS_FCU_REVISION}")

# The last 256 KB of the flash (0x081C0000, both chips) hold the PLC's program slots
# (src/plc.cpp, docs/plc-iec61131.md 4.5) and stay outside the image.
rtos_add_firmware(firmware_factory_control_unit
    SOURCES ${FCU_SOURCES}
    LIBRARIES rtos_board_fcu rtos_tls rtos_opcua rtos_filex rtos_plc
    FLASH_DATA_START 0x081C0000)
target_include_directories(firmware_factory_control_unit PRIVATE src src/hal/${FCU_HAL} ${FCU_GENERATED})
target_compile_options(firmware_factory_control_unit PRIVATE -Wall -Wextra -Werror --embed-dir=${FCU_ASSETS})
