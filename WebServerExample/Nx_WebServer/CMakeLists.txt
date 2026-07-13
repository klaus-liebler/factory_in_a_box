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
target_compile_definitions(${BUILD_UNIT_0_NAME} PRIVATE USE_FULL_LL_DRIVER)

# Linker options
target_link_options(${BUILD_UNIT_0_NAME} PRIVATE 
  # User defined linker options
)

# Add sources to executable/library
target_sources(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined sources
)

# Add include directories
target_include_directories(${BUILD_UNIT_0_NAME} PRIVATE
  # User defined include directories
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
