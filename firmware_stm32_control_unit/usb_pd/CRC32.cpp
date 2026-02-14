#include "CRC32.h"
#include "stm32g431xx.h"

static void enableCrcClock() {
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    (void)RCC->AHB1ENR; // ensure write completes before CRC access
}

bool CRC32::isValid(const uint8_t* data, int len) {
    enableCrcClock();

    CRC->CR = (0b01 << CRC_CR_REV_IN_Pos) | CRC_CR_RESET;
    for (int i = 0; i < len; i += 1) {
        *((volatile uint8_t*)&CRC->DR) = data[i];
    }

    return CRC->DR == ExpectedResidual;
}

uint32_t CRC32::compute(const uint8_t* data, int len) {
    enableCrcClock();

    CRC->CR = (0b01 << CRC_CR_REV_IN_Pos) | CRC_CR_REV_OUT | CRC_CR_RESET;
    for (int i = 0; i < len; i += 1) {
        *((volatile uint8_t*)&CRC->DR) = data[i];
    }

    return ~CRC->DR;
}

