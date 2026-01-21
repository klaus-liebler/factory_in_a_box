cmake_minimum_required(VERSION 3.22)

add_library(TMC2209 STATIC
    TMC2209.cpp
)

# Public headers for consumers
target_include_directories(TMC2209 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against stm32cubemx (for HAL) and common_stm32 (for GPIO utilities)
target_link_libraries(TMC2209 PUBLIC stm32cubemx common_stm32)

# C++ settings are inherited from top-level
