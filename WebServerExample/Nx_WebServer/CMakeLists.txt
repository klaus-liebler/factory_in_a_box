# Set the minimum version of CMake that can be used
cmake_minimum_required(VERSION 3.20)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Project definition
project(Nx_WebServer)

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

set(BUILD_UNIT_0_NAME Nx_WebServer)

# Create an executable object type
add_executable(${BUILD_UNIT_0_NAME})

# STM32CubeIDE for Visual Studio Code managed CMake file
include(cmake/vscode_generated.cmake)

# Compiler options
target_compile_options(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined compiler options
)

# USE_FULL_LL_DRIVER: needed because Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_ll_{ucpd,gpio,dma}.c
# (added below for the stm32_libs usb_pd/UCPD port) are entirely gated behind this macro.
# NX_SECURE_INCLUDE_USER_DEFINE_FILE: pulls in Core/Inc/nx_secure_user.h (NetX Secure config).
# NX_WEB_HTTPS_ENABLE: activates the #ifdef NX_WEB_HTTPS_ENABLE HTTPS code paths inside the
# already-vendored Middlewares/ST/netxduo/addons/web/nx_web_http_server.c (nx_web_http_server_secure_configure()
# etc.) -- without this define those functions don't even exist, plain HTTP-only build.
# NX_SECURE_ENABLE_ECC_CIPHERSUITE / NX_SECURE_ENABLE_AEAD_CIPHER: ohne ECC bietet der Server
# nur noch die (nicht forward-secret) TLS_RSA_WITH_AES_*_CBC_SHA256-Ciphersuiten an, die alle
# gaengigen Browser inzwischen ablehnen (ERR_SSL_VERSION_OR_CIPHER_MISMATCH) -- ECC schaltet
# TLS_ECDHE_ECDSA_WITH_AES_128_{GCM,CBC}_SHA256 frei (ECDHE-Keyexchange + ECDSA-Auth, passt zum
# Geraete-Zertifikat, das per tools/provision-certificate.mjs als ECDSA/P-256 statt RSA-2048
# erzeugt wird -- deutlich billigeres Signing beim ServerKeyExchange auf dem STM32H573).
target_compile_definitions(${BUILD_UNIT_0_NAME} PRIVATE
  USE_FULL_LL_DRIVER
  NX_SECURE_INCLUDE_USER_DEFINE_FILE
  NX_WEB_HTTPS_ENABLE
  NX_SECURE_ENABLE_ECC_CIPHERSUITE
  NX_SECURE_ENABLE_AEAD_CIPHER
)

# Linker options
target_link_options(${BUILD_UNIT_0_NAME} PRIVATE 
  # User defined linker options
)

# Add sources to executable/library
target_sources(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined sources
  # Generiert von web/scripts/embed-into-firmware.mjs ("npm run embed" in web/) -- eingecheckt,
  # nicht Teil dieses CMake-Builds selbst (kein Node.js-Bedarf beim Firmware-Bauen).
  Core/Src/generated/modbus_ui_page.c
  # Generiert von tools/provision-certificate.mjs ("node tools/provision-certificate.mjs")
  # -- eingecheckt, ebenfalls kein automatischer Build-Schritt.
  Core/Src/generated/device_certificate.c
  Middlewares/ST/netxduo/addons/mdns/nxd_mdns.c
)

# NetX Secure (TLS) + Crypto -- vendort aus dem lokal gecachten STM32Cube_FW_H5-Paket (siehe
# Middlewares/ST/netxduo/{nx_secure,crypto_libraries}), wie netxduo/threadx/filex selbst schon
# vendort sind. Wegen der Dateimenge (~360 Dateien) per GLOB statt Hand-Liste eingebunden.
file(GLOB_RECURSE NX_SECURE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/netxduo/nx_secure/src/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/netxduo/crypto_libraries/src/*.c
)
target_sources(${BUILD_UNIT_0_NAME} PRIVATE ${NX_SECURE_SOURCES})

# Add include directories
target_include_directories(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined include directories
  Middlewares/ST/netxduo/nx_secure/inc
  Middlewares/ST/netxduo/nx_secure/ports
  Middlewares/ST/netxduo/crypto_libraries/inc
  # Kein cortex_m33-Port vorhanden; der cortex_m4-Port enthaelt nur reine Typdefinitionen
  # (nx_crypto_port.h, keine Assembly) und ist unveraendert auf M33 nutzbar.
  Middlewares/ST/netxduo/crypto_libraries/ports/cortex_m4/gnu/inc
  Middlewares/ST/netxduo/addons/mdns
)

# --- stm32_libs (common_stm32/tmc2209/usb_pd) integration ---
# The shared libraries' own CMakeLists.txt link against a target named "stm32cubemx",
# matching the classic multi-target STM32CubeIDE CMake layout used by the STM32G4
# projects (firmware_stm32_control_unit/devboard). Nx_WebServer uses the newer
# single-target "vscode_generated.cmake" layout instead, so provide a lightweight
# INTERFACE target here that wraps this project's own HAL/CMSIS include dirs and
# ARM/ABI compile flags (normally PRIVATE to ${BUILD_UNIT_0_NAME} only, and thus
# invisible to out-of-tree add_subdirectory() targets without this).
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
)
target_compile_options(stm32cubemx INTERFACE
  $<$<COMPILE_LANGUAGE:C>: ${CUBE_CMAKE_C_FLAGS}>
  $<$<COMPILE_LANGUAGE:CXX>: ${CUBE_CMAKE_CXX_FLAGS}>
)
# STM32H5 / STM32H5xx: family macros the shared libraries branch on (matching the
# STM32G4 / STM32G4xx pattern already used by the G4 projects' stm32cubemx target).
# USE_FULL_LL_DRIVER: the LL (Low-Layer) driver .c files (needed by usb_pd's UCPD/GPDMA
# code) compile to nothing without this - their entire content is behind #if defined(USE_FULL_LL_DRIVER).
target_compile_definitions(stm32cubemx INTERFACE STM32H5 STM32H5xx USE_FULL_LL_DRIVER)

# Out-of-tree add_subdirectory() needs an explicit binary dir.
add_subdirectory(../../stm32_libs/common_stm32 ${CMAKE_BINARY_DIR}/stm32_libs/common_stm32)
add_subdirectory(../../stm32_libs/tmc2209 ${CMAKE_BINARY_DIR}/stm32_libs/tmc2209)
add_subdirectory(../../stm32_libs/usb_pd ${CMAKE_BINARY_DIR}/stm32_libs/usb_pd)

target_link_libraries(${BUILD_UNIT_0_NAME} PRIVATE common_stm32 tmc2209 usb_pd)
