cmake_minimum_required(VERSION 3.22)

add_library(usb_pd STATIC
    CRC32.cpp
    PDController.cpp
    PDMessageDecoder.cpp
    PDMessageEncoder.cpp
    PDPhySTM32UCPD.cpp
    PDProtocolAnalyzer.cpp
    PDSink.cpp
    PDSourceCapability.cpp
    TaskScheduler.cpp
)

# Public headers for consumers
 target_include_directories(usb_pd PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against stm32cubemx (HAL/CMSIS) and common utilities if needed
 target_link_libraries(usb_pd PUBLIC stm32cubemx common_stm32)
