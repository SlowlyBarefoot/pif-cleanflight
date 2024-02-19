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

#include "sensor/pif_mpu60x0.h"

#include "system.h"
#include "bus_i2c.h"
#include "gyro_sync.h"

#include "accgyro_mpu.h"
#include "accgyro_mpu6050.h"

const char* mpu6050_name = "MPU6050";

static PifMpu60x0 mpu60x0;

static void mpu6050AccInit(void* p_param);
static void mpu6050GyroInit(void* p_param);

bool mpu6050AccDetect(void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }

    sensor_link.acc.hw_name = mpu6050_name;
    sensor_link.acc.init = mpu6050AccInit;
    sensor_link.acc.read = mpuAccRead;
    sensor_link.acc.revisionCode = (mpuDetectionResult.resolution == MPU_HALF_RESOLUTION ? 'o' : 'n'); // es/non-es variance between MPU6050 sensors, half of the naze boards are mpu6000ES.

    return true;
}

bool mpu6050GyroDetect(void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }

    if (!pifMpu60x0_Detect(&g_i2c_port, MPU60X0_I2C_ADDR(0))) return false;

    sensor_link.gyro.hw_name = mpu6050_name;
    sensor_link.gyro.init = mpu6050GyroInit;
    sensor_link.gyro.read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    sensor_link.gyro.scale = 1.0f / 16.4f;

    return true;
}

static void mpu6050AccInit(void* p_param)
{
    (void)p_param;

    mpuIntExtiInit(NULL);

    switch (mpuDetectionResult.resolution) {
        case MPU_HALF_RESOLUTION:
            sensor_link.acc.acc_1G = 256 * 8;
            break;
        case MPU_FULL_RESOLUTION:
            sensor_link.acc.acc_1G = 512 * 8;
            break;
    }
}

static void mpu6050GyroInit(void* p_param)
{
    bool ack;
    uint8_t lpf = 1;        // default
    PifMpu60x0PwrMgmt1 pwr_mgmt_1;
    PifMpu60x0Config config;
    PifMpu60x0GyroConfig gyro_config;
    PifMpu60x0AccelConfig accel_config;

    if (mpuIntExtiInit()) sensor_link.gyro.can_sync = true;

    pifMpu60x0_Init(&mpu60x0, PIF_ID_AUTO, &g_i2c_port, MPU60X0_I2C_ADDR(0), &sensor_link.imu_sensor);

    if (p_param) lpf = ((gyro_param_t*)p_param)->lpf;

    pwr_mgmt_1.byte = 0;
    pwr_mgmt_1.bit.device_reset = TRUE;
    ack = pifI2cDevice_WriteRegByte(mpu60x0._p_i2c, MPU60X0_REG_PWR_MGMT_1, pwr_mgmt_1.byte);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(100);
    pwr_mgmt_1.byte = 0;
    pwr_mgmt_1.bit.clksel = MPU60X0_CLKSEL_PLL_ZGYRO;
    ack = pifI2cDevice_WriteRegByte(mpu60x0._p_i2c, MPU60X0_REG_PWR_MGMT_1, pwr_mgmt_1.byte); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    ack = pifI2cDevice_WriteRegByte(mpu60x0._p_i2c, MPU60X0_REG_SMPLRT_DIV, gyroMPU6xxxCalculateDivider()); //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    delay(15); //PLL Settling time when changing CLKSEL is max 10ms.  Use 15ms to be sure 
    config.byte = 0;
    config.bit.dlpf_cfg = lpf;
    ack = pifI2cDevice_WriteRegByte(mpu60x0._p_i2c, MPU60X0_REG_CONFIG, config.byte); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    gyro_config.byte = 0;
    gyro_config.bit.fs_sel = MPU60X0_FS_SEL_2000DPS;
    ack = pifMpu60x0_SetGyroConfig(&mpu60x0, gyro_config);   //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

    // ACC Init stuff.
    // Accel scale 8g (4096 LSB/g)
    accel_config.byte = 0;
    accel_config.bit.afs_sel = MPU60X0_AFS_SEL_8G;
    ack = pifMpu60x0_SetAccelConfig(&mpu60x0, accel_config);

    ack = pifI2cDevice_WriteRegByte(mpu60x0._p_i2c, MPU60X0_REG_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0); // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS

#ifdef USE_MPU_DATA_READY_SIGNAL
    ack = pifI2cDevice_WriteRegByte(mpu60x0._p_i2c, MPU60X0_REG_INT_ENABLE, MPU_RF_DATA_RDY_EN);
#endif
    UNUSED(ack);
}
