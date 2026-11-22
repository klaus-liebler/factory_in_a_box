# FCU revision 2 (pcb_factory_control_unit, MCU U1): STM32H563VIT6, USB-PD by a CH224K.
# Included by CMakeLists.txt before project(): the chip (CPU flags of the toolchain) and the
# revision of rtoos's board package (boards/fcu) come from here, not from the preset.
set(RTOS_CHIP stm32h563 CACHE STRING "Chip of this build tree" FORCE)
set(RTOS_FCU_REVISION 2 CACHE STRING "Factory Control Unit hardware revision" FORCE)
set(FCU_HAL_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/power_delivery.cpp
    ${CMAKE_SOURCE_DIR}/src/features/malibotti/malibotti_absent.cpp)
