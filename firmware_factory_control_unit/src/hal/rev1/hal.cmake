# FCU revision 1 (first boards, pcb_factory_control_unit_rev1): STM32H563VIT6, USB-PD by its UCPD1.
# Included by CMakeLists.txt before project(): the chip (CPU flags of the toolchain) and the
# revision of rtoos's board package (boards/fcu) come from here, not from the preset.
set(RTOS_CHIP stm32h563 CACHE STRING "Chip of this build tree" FORCE)
set(RTOS_FCU_REVISION 1 CACHE STRING "Factory Control Unit hardware revision" FORCE)
set(FCU_HAL_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/power_delivery.cpp
    ${CMAKE_SOURCE_DIR}/src/features/malibotti_absent.cpp)
