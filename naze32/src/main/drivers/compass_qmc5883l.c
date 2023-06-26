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

#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "sensor/pif_qmc5883.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "nvic.h"
#include "gpio.h"
#include "bus_i2c.h"
#include "light_led.h"

#include "sensors/sensors.h"

#include "compass_qmc5883l.h"

const char* qmc5883_name = "QMC5883";

static PifQmc5883 qmc5883;

static void qmc5883lInit(void* p_param);
static bool qmc5883lRead(float* p_mag);

bool qmc5883lDetect(void* p_param)
{
    UNUSED(p_param);

    if (!pifQmc5883_Detect(&g_i2c_port)) return false;

    sensor_link.mag.hw_name = qmc5883_name;
    sensor_link.mag.init = qmc5883lInit;
    sensor_link.mag.read = qmc5883lRead;

    return true;
}

static void qmc5883lInit(void* p_param)
{
    PifQmc5883Control1 control_1;

    UNUSED(p_param);

    pifImuSensor_SetMagAlign(&sensor_link.imu_sensor, sensor_link.mag.align);

    if (!pifQmc5883_Init(&qmc5883, PIF_ID_AUTO, &g_i2c_port, &sensor_link.imu_sensor)) return;

    control_1.bit.mode = QMC5883_MODE_CONTIMUOUS;
    control_1.bit.odr = QMC5883_ODR_10HZ;
    control_1.bit.rng = QMC5883_RNG_2G;
    control_1.bit.osr = QMC5883_OSR_64;
    pifQmc5883_SetControl1(&qmc5883, control_1);
}

static bool qmc5883lRead(float* p_mag)
{
    return pifImuSensor_ReadRawMag(&sensor_link.imu_sensor, p_mag);
}
