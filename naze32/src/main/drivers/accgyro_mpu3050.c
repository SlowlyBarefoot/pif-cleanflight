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

#include "sensor/pif_mpu30x0.h"

#include "system.h"
#include "exti.h"
#include "bus_i2c.h"

#include "gyro_sync.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu3050.h"

const char* mpu3050_name = "MPU3050";

static PifMpu30x0 mpu30x0;

static void mpu3050Init(void* p_param);
static bool mpu3050ReadTemp(float* p_tempData);

bool mpu3050Detect(void* p_param)
{
    (void)p_param;

    if (mpuDetectionResult.sensor != MPU_3050) {
        return false;
    }

    if (!pifMpu30x0_Detect(&g_i2c_port, MPU30X0_I2C_ADDR)) return false;

    sensor_link.gyro.hw_name = mpu3050_name;
    sensor_link.gyro.init = mpu3050Init;
    sensor_link.gyro.read = mpuGyroRead;
    sensor_link.gyro.temperature = mpu3050ReadTemp;

    // 16.4 dps/lsb scalefactor
    sensor_link.gyro.scale = 1.0f / 16.4f;

    return true;
}

static void mpu3050Init(void* p_param)
{
    bool ack;
    uint8_t lpf = 1;        // default
    PifMpu30x0DlpfFsSync dlpf_fs_sync;
    PifMpu30x0UserCtrl user_ctrl;
    PifMpu30x0PwrMgmt pwr_mgmt;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    if (p_param) lpf = *(uint8_t*)p_param;

    ack = pifI2cDevice_WriteRegByte(mpu30x0._p_i2c, MPU30X0_REG_SMPLRT_DIV, 0);
    if (!ack)
        failureMode(FAILURE_ACC_INIT);

    if (!pifMpu30x0_Init(&mpu30x0, PIF_ID_AUTO, &g_i2c_port, MPU30X0_I2C_ADDR, &sensor_link.imu_sensor)) return;

    dlpf_fs_sync.byte = 0;
    dlpf_fs_sync.bit.fs_sel = MPU30X0_FS_SEL_2000DPS;
    dlpf_fs_sync.bit.dlpf_cfg = lpf;
    pifMpu30x0_SetDlpfFsSync(&mpu30x0, dlpf_fs_sync);
    pifI2cDevice_WriteRegByte(mpu30x0._p_i2c, MPU30X0_REG_INT_CFG, 0);
    user_ctrl.byte = 0;
    user_ctrl.bit.gyro_rst = TRUE;
    pifI2cDevice_WriteRegByte(mpu30x0._p_i2c, MPU30X0_REG_USER_CTRL, user_ctrl.byte);
    pwr_mgmt.byte = 0;
    pwr_mgmt.bit.clk_sel = MPU30X0_CLK_SEL_PLL_XGYRO;
    pifI2cDevice_WriteRegByte(mpu30x0._p_i2c, MPU30X0_REG_PWR_MGMT, pwr_mgmt.byte);
}

static bool mpu3050ReadTemp(float* p_tempData)
{
    return pifMpu30x0_ReadTemperature(&mpu30x0, p_tempData);
}
