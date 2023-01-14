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
#include <stdlib.h>

#include <platform.h>

#include "common/axis.h"
#include "common/maths.h"

#include "sensors/sensors.h"

#include "system.h"
#include "exti.h"
#include "gpio.h"
#include "bus_spi.h"

#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6500.h"

const char* mpu6500_spi_name = "MPU6500";

#define DISABLE_MPU6500       GPIO_SetBits(MPU6500_CS_GPIO,   MPU6500_CS_PIN)
#define ENABLE_MPU6500        GPIO_ResetBits(MPU6500_CS_GPIO, MPU6500_CS_PIN)

bool mpu6500WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg);
    spiTransferByte(MPU6500_SPI_INSTANCE, data);
    DISABLE_MPU6500;

    return true;
}

bool mpu6500ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU6500_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU6500;

    return true;
}

static void mpu6500SpiInit(sensor_link_t* p_sensor_link, void* p_param)
{
    static bool hardwareInitialised = false;

    (void)p_sensor_link;
    (void)p_param;

    if (hardwareInitialised) {
        return;
    }

#ifdef STM32F10X
    RCC_APB2PeriphClockCmd(MPU6500_CS_GPIO_CLK_PERIPHERAL, ENABLE);

    gpio_config_t gpio;
    // CS as output
    gpio.mode = Mode_Out_PP;
    gpio.pin = MPU6500_CS_PIN;
    gpio.speed = Speed_50MHz;
    gpioInit(MPU6500_CS_GPIO, &gpio);
#endif

    GPIO_SetBits(MPU6500_CS_GPIO,   MPU6500_CS_PIN);

    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

    hardwareInitialised = true;
}

bool mpu6500SpiDetect(sensor_link_t* p_sensor_link)
{
    uint8_t sig;

    mpu6500SpiInit(p_sensor_link, NULL);

    mpu6500ReadRegister(MPU_RA_WHO_AM_I, 1, &sig);

    sig &= MPU_INQUIRY_MASK;
    if (sig != MPU6500_WHO_AM_I_CONST) {
        return false;
    }
    return true;
}

bool mpu6500SpiAccDetect(sensor_link_t* p_sensor_link, void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_65xx_SPI) {
        return false;
    }

    p_sensor_link->acc.hw_name = mpu6500_spi_name;
    p_sensor_link->acc.init = mpu6500AccInit;
    p_sensor_link->acc.read = mpuAccRead;

    return true;
}

bool mpu6500SpiGyroDetect(sensor_link_t* p_sensor_link, void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_65xx_SPI) {
        return false;
    }

    p_sensor_link->gyro.hw_name = mpu6500_spi_name;
    p_sensor_link->gyro.init = mpu6500GyroInit;
    p_sensor_link->gyro.read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    p_sensor_link->gyro.scale = 1.0f / 16.4f;

    return true;
}
