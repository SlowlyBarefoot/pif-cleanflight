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
#include "bus_i2c.h"
#include "gyro_sync.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"

#include "sensor/pif_mpu6500.h"

static PifMpu6500 s_mpu6500;

extern uint16_t acc_1G;

bool mpu6500AccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    acc->init = mpu6500AccInit;
    acc->read = pifMpuAccRead;

    return true;
}

bool mpu6500GyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    gyro->init = mpu6500GyroInit;
    gyro->read = pifMpuGyroRead;
    gyro->isDataReady = mpuIsDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

void mpu6500AccInit(void)
{
    mpuIntExtiInit();

    acc_1G = 512 * 8;
}

void mpu6500GyroInit(uint8_t lpf)
{
    PifMpu6500PwrMgmt1 pwr_mgmt_1;
    PifMpu6500GyroConfig gyro_config;
    PifMpu6500Config config;
    PifMpu6500AccelConfig accel_config;

    mpuIntExtiInit();

    pifMpu6500_Init(&s_mpu6500, PIF_ID_AUTO, &g_i2c_port, MPU6500_I2C_ADDR(0), &g_imu_sensor);

    pifI2cDevice_WriteRegByte(s_mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, 0x80); // Device reset
    delay(100);
    pifI2cDevice_WriteRegByte(s_mpu6500._p_i2c, MPU6500_REG_SIGNAL_PATH_RESET, 0x07); // Signal path reset
    delay(100);
    pifI2cDevice_WriteRegByte(s_mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, 0x00); // Device reset
    delay(100);
    pwr_mgmt_1.byte = 0;
    pwr_mgmt_1.bit.clksel = 1; // Clock source = 1 (Auto-select PLL or else intrc)
    pifI2cDevice_WriteRegByte(s_mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, pwr_mgmt_1.byte);

    gyro_config.byte = 0;
    gyro_config.bit.gyro_fs_sel = MPU6500_GYRO_FS_SEL_2000DPS;
    pifMpu6500_SetGyroConfig(&s_mpu6500, gyro_config);

    accel_config.byte = 0;
    accel_config.bit.accel_fs_sel = MPU6500_ACCEL_FS_SEL_8G;
    pifMpu6500_SetAccelConfig(&s_mpu6500, accel_config);

    config.byte = 0;
    config.bit.dlpf_cfg = lpf;
    pifI2cDevice_WriteRegByte(s_mpu6500._p_i2c, MPU6500_REG_CONFIG, config.byte);

    pifI2cDevice_WriteRegByte(s_mpu6500._p_i2c, MPU6500_REG_SMPLRT_DIV, gyroMPU6xxxCalculateDivider());   // Get Divider

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
