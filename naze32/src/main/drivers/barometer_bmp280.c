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

#include "core/pif_i2c.h"
#include "sensor/pif_bmp280.h"

#include "sensors/sensors.h"

#include "bus_i2c.h"

#ifdef BARO

const char* bmp280_name = "BMP280";

static PifBmp280 bmp280;
static bool bmp280InitDone = false;

bool bmp280Detect(void* p_param)
{
    (void)p_param;

    if (bmp280InitDone)
        return true;

    pif_Delay1ms(20);

    if (!pifBmp280_Detect(&g_i2c_port, BMP280_I2C_ADDR(0))) return false;

    if (!pifBmp280_Init(&bmp280, PIF_ID_AUTO, &g_i2c_port, BMP280_I2C_ADDR(0))) return false;

    // set oversampling + power mode (forced), and start sampling
    pifBmp280_SetOverSamplingRate(&bmp280, BMP280_OSRS_X8, BMP280_OSRS_X1);

    if (!pifBmp280_AddTaskForReading(&bmp280, 50, sensor_link.baro.evt_read, TRUE)) return false;   // 25ms : 40hz update rate (20hz LPF on acc)
    bmp280._p_task->disallow_yield_id = DISALLOW_YIELD_ID_I2C;

    bmp280InitDone = true;

    sensor_link.baro.hw_name = bmp280_name;
    sensor_link.baro.p_task = bmp280._p_task;

    return true;
}

#endif
