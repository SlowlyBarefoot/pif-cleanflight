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

#ifdef USE_MAG_QMC5883

#include "pif_linker.h"

#include "common/axis.h"

#include "bus_i2c.h"
#include "sensor.h"
#include "compass.h"
#include "compass_qmc5883l.h"

#include "sensor/pif_qmc5883.h"

static PifQmc5883 qmc5883;

bool qmc5883lDetect(mag_t* mag)
{
    if (!pifQmc5883_Detect(&g_i2c_port)) return false;

    mag->init = qmc5883lInit;
    mag->read = qmc5883lRead;

    return true;
}

bool qmc5883lInit(void)
{
    PifQmc5883Control1 control_1;

    if (!pifQmc5883_Init(&qmc5883, PIF_ID_AUTO, &g_i2c_port, &g_imu_sensor)) return false;

    control_1.bit.mode = QMC5883_MODE_CONTIMUOUS;
    control_1.bit.odr = QMC5883_ODR_10HZ;
    control_1.bit.rng = QMC5883_RNG_2G;
    control_1.bit.osr = QMC5883_OSR_64;
    pifQmc5883_SetControl1(&qmc5883, control_1);
    return true;
}

bool qmc5883lRead(int16_t *magData)
{
    float buf[3];

    if (!pifImuSensor_ReadRawMag(&g_imu_sensor, buf)) {
        return false;
    }

    magData[X] = (int16_t)buf[0];
    magData[Z] = (int16_t)buf[1];
    magData[Y] = (int16_t)buf[2];

    return true;
}

#endif