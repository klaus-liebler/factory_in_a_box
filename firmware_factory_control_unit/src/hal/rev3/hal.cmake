# FCU revision 3 (pcb_factory_control_unit, MCU U17): STM32H743VIT6, USB-PD by a CH224Q.
# Included by CMakeLists.txt before project(): the chip (CPU flags of the toolchain) and the
# revision of rtoos's board package (boards/fcu_rev3) come from here, not from the preset.
set(RTOS_CHIP stm32h743 CACHE STRING "Chip of this build tree" FORCE)
set(RTOS_FCU_REVISION 3 CACHE STRING "Factory Control Unit hardware revision" FORCE)
set(FCU_HAL_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/power_delivery.cpp
    ${CMAKE_SOURCE_DIR}/src/features/malibotti/malibotti.cpp)
