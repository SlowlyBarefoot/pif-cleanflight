/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build_config.h"

#include "gpio.h"

#include "bus_spi.h"

#ifdef USE_SPI_DEVICE_1

#ifndef SPI1_GPIO
#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
#define SPI1_NSS_PIN            GPIO_Pin_4
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource4
#define SPI1_SCK_PIN            GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource5
#define SPI1_MISO_PIN           GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource6
#define SPI1_MOSI_PIN           GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource7
#endif

void initSpi1(void)
{
    // Specific to the STM32F103
    // SPI1 Driver
    // PA4    14    SPI1_NSS
    // PA5    15    SPI1_SCK
    // PA6    16    SPI1_MISO
    // PA7    17    SPI1_MOSI

    SPI_InitTypeDef spi;

    // Enable SPI1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);


#ifdef STM32F10X
    gpio_config_t gpio;
    // MOSI + SCK as output
    gpio.mode = Mode_AF_PP;
    gpio.pin = SPI1_MOSI_PIN | SPI1_SCK_PIN;
    gpio.speed = Speed_50MHz;
    gpioInit(GPIOA, &gpio);
    // MISO as input
    gpio.pin = SPI1_MISO_PIN;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOA, &gpio);
#ifdef SPI1_NSS_PIN
    // NSS as gpio slave select
    gpio.pin = SPI1_NSS_PIN;
    gpio.mode = Mode_Out_PP;
    gpioInit(GPIOA, &gpio);
#endif
#endif

    // Init SPI hardware
    SPI_I2S_DeInit(SPI1);

    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    
#ifdef USE_SDCARD_SPI1
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
#else
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
#endif

    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}
#endif

#ifdef USE_SPI_DEVICE_2

#ifndef SPI2_GPIO
#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_NSS_PIN            GPIO_Pin_12
#define SPI2_NSS_PIN_SOURCE     GPIO_PinSource12
#define SPI2_SCK_PIN            GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN           GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN           GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15
#endif

void initSpi2(void)
{
    // Specific to the STM32F103 / STM32F303 (AF5)
    // SPI2 Driver
    // PB12     25      SPI2_NSS
    // PB13     26      SPI2_SCK
    // PB14     27      SPI2_MISO
    // PB15     28      SPI2_MOSI

    SPI_InitTypeDef spi;

    // Enable SPI2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);


#ifdef STM32F10X
    gpio_config_t gpio;

    // MOSI + SCK as output
    gpio.mode = Mode_AF_PP;
    gpio.pin = SPI2_SCK_PIN | SPI2_MOSI_PIN;
    gpio.speed = Speed_50MHz;
    gpioInit(SPI2_GPIO, &gpio);
    // MISO as input
    gpio.pin = SPI2_MISO_PIN;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(SPI2_GPIO, &gpio);

#ifdef SPI2_NSS_PIN
    // NSS as gpio slave select
    gpio.pin = SPI2_NSS_PIN;
    gpio.mode = Mode_Out_PP;
    gpioInit(SPI2_GPIO, &gpio);
#endif
#endif

    // Init SPI2 hardware
    SPI_I2S_DeInit(SPI2);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;

#ifdef USE_SDCARD_SPI2
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
#else
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
#endif

    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);

    // Drive NSS high to disable connected SPI device.
    GPIO_SetBits(SPI2_GPIO, SPI2_NSS_PIN);


}
#endif

bool spiInit(SPI_TypeDef *instance)
{
#if (!(defined(USE_SPI_DEVICE_1) && defined(USE_SPI_DEVICE_2)))
    UNUSED(instance);
#endif

#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1) {
        initSpi1();
        return true;
    }
#endif
#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2) {
        initSpi2();
        return true;
    }
#endif
    return false;
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
    }

#ifdef STM32F10X
    SPI_I2S_SendData(instance, data);
#endif
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET){
    }

#ifdef STM32F10X
    return ((uint8_t)SPI_I2S_ReceiveData(instance));
#endif
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
#ifdef STM32F10X
    return SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
#endif

}

void spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    uint8_t b;
    instance->DR;
    while (len--) {
        b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
        }
#ifdef STM32F10X
        SPI_I2S_SendData(instance, b);
#endif
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
        }
#ifdef STM32F10X
        b = SPI_I2S_ReceiveData(instance);
#endif
        if (out)
            *(out++) = b;
    }
}


void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(instance, DISABLE);

    tempRegister = instance->CR1;

    switch (divisor) {
        case 2:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_2;
            break;

        case 4:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_4;
            break;

        case 8:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_8;
            break;

        case 16:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_16;
            break;

        case 32:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_32;
            break;

        case 64:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_64;
            break;

        case 128:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_128;
            break;

        case 256:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_256;
            break;
    }

    instance->CR1 = tempRegister;

    SPI_Cmd(instance, ENABLE);
}
