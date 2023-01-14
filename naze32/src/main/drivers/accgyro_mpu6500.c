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
#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "exti.h"
#include "gpio.h"
#include "gyro_sync.h"

#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"

const char* mpu6500_name = "MPU6500";

bool mpu6500AccDetect(sensor_link_t* p_sensor_link, void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    p_sensor_link->acc.hw_name = mpu6500_name;
    p_sensor_link->acc.init = mpu6500AccInit;
    p_sensor_link->acc.read = mpuAccRead;

    return true;
}

bool mpu6500GyroDetect(sensor_link_t* p_sensor_link, void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    p_sensor_link->gyro.hw_name = mpu6500_name;
    p_sensor_link->gyro.init = mpu6500GyroInit;
    p_sensor_link->gyro.read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    p_sensor_link->gyro.scale = 1.0f / 16.4f;

    return true;
}

void mpu6500AccInit(sensor_link_t* p_sensor_link, void* p_param)
{
    (void)p_param;

    mpuIntExtiInit(NULL);

    p_sensor_link->acc.acc_1G = 512 * 8;
}

void mpu6500GyroInit(sensor_link_t* p_sensor_link, void* p_param)
{
    uint8_t lpf = 1;        // default


    if (mpuIntExtiInit(p_sensor_link)) p_sensor_link->gyro.can_sync = true;

    if (p_param) lpf = ((gyro_param_t*)p_param)->lpf;
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
    mpuConfiguration.write(MPU_RA_SIGNAL_PATH_RESET, 0x07);
    delay(100);
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0);
    delay(100);
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    mpuConfiguration.write(MPU_RA_CONFIG, lpf);
    mpuConfiguration.write(MPU_RA_SMPLRT_DIV, gyroMPU6xxxCalculateDivider()); // Get Divider

    // Data ready interrupt configuration
#ifdef USE_MPU9250_MAG
    mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#else
    mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#endif
#ifdef USE_MPU_DATA_READY_SIGNAL
    mpuConfiguration.write(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif
}
