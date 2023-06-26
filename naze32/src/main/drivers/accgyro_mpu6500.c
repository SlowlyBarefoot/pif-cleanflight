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

#include "sensor/pif_mpu6500.h"

#include "system.h"
#include "gyro_sync.h"
#include "bus_i2c.h"

#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"

const char* mpu6500_name = "MPU6500";

static PifMpu6500 mpu6500;

bool mpu6500AccDetect(void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    sensor_link.acc.hw_name = mpu6500_name;
    sensor_link.acc.init = mpu6500AccInit;
    sensor_link.acc.read = mpuAccRead;

    return true;
}

bool mpu6500GyroDetect(void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    if (!pifMpu6500_Detect(&g_i2c_port, MPU6500_I2C_ADDR(0))) return false;

    if (!pifMpu6500_Init(&mpu6500, PIF_ID_AUTO, &g_i2c_port, MPU6500_I2C_ADDR(0), &sensor_link.imu_sensor)) return false;

    sensor_link.gyro.hw_name = mpu6500_name;
    sensor_link.gyro.init = mpu6500GyroInit;
    sensor_link.gyro.read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    sensor_link.gyro.scale = 1.0f / 16.4f;

    return true;
}

void mpu6500AccInit(void* p_param)
{
    (void)p_param;

    mpuIntExtiInit(NULL);

    sensor_link.acc.acc_1G = 512 * 8;
}

void mpu6500GyroInit(void* p_param)
{
    uint8_t lpf = 1;        // default
    PifMpu6500PwrMgmt1 pwr_mgmt_1;
    PifMpu6500GyroConfig gyro_config;
    PifMpu6500Config config;
    PifMpu6500AccelConfig accel_config;

    if (mpuIntExtiInit()) sensor_link.gyro.can_sync = true;

    if (p_param) lpf = ((gyro_param_t*)p_param)->lpf;

    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, MPU6500_BIT_RESET); // Device reset
    pif_Delay1ms(100);

    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_SIGNAL_PATH_RESET, 0x07); // Signal path reset
    pif_Delay1ms(100);

    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, 0);
    pif_Delay1ms(100);

    pwr_mgmt_1.byte = 0;
    pwr_mgmt_1.bit.clksel = MPU6500_CLKSEL_PLL;
    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, pwr_mgmt_1.byte);

    gyro_config.byte = 0;
    gyro_config.bit.gyro_fs_sel = MPU6500_GYRO_FS_SEL_2000DPS;
    pifMpu6500_SetGyroConfig(&mpu6500, gyro_config);

    accel_config.byte = 0;
    accel_config.bit.accel_fs_sel = MPU6500_ACCEL_FS_SEL_8G;
    pifMpu6500_SetAccelConfig(&mpu6500, accel_config);

    config.byte = 0;
    config.bit.dlpf_cfg = lpf;
    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_CONFIG, config.byte);

    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_SMPLRT_DIV, gyroMPU6xxxCalculateDivider());   // Get Divider

    // Data ready interrupt configuration
#ifdef USE_MPU9250_MAG
    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#else
    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#endif
#ifdef USE_MPU_DATA_READY_SIGNAL
    pifI2cDevice_WriteRegByte(mpu6500._p_i2c, MPU6500_REG_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif
}
