cmake_minimum_required(VERSION 3.22)

add_library(common_stm32 STATIC
    common.cc
    log.c
)

# Public headers for consumers
target_include_directories(common_stm32 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Inherit HAL/CMSIS for STM32-specific headers
target_link_libraries(common_stm32 PUBLIC stm32cubemx)
