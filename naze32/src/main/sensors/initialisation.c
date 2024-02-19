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
#include <string.h>

#include <platform.h>

#include "common/utils.h"

#include "sensors/sensors.h"

#include "drivers/gyro_sync.h"

#include "config/runtime_config.h"


static bool detectGyro()
{
    extern const sensorDetect_t gyro_detect[];
    const sensorDetect_t* p_detect = gyro_detect;
    sensor_link.gyro.align = IMUS_ALIGN_DEFAULT;

    while (p_detect->p_func) {
    	if ((*p_detect->p_func)(p_detect->p_param)) {
            sensor_link.gyro.align = p_detect->align;
    		break;
    	}
    	p_detect++;
    }
    if (!sensor_link.gyro.hw_name) return false;

    sensorsSet(SENSOR_GYRO);

    return true;
}

static void detectAcc(int accHardwareToUse)
{
    extern const sensorDetect_t acc_detect[];
    const sensorDetect_t* p_detect;

    sensor_link.acc.align = IMUS_ALIGN_DEFAULT;

    if (accHardwareToUse > SENSOR_DEFAULT) {
        p_detect = acc_detect;
        while (p_detect->p_func) {
            if (p_detect->sensor_no >= accHardwareToUse) {
                if ((*p_detect->p_func)(p_detect->p_param)) {
                    sensor_link.acc.align = p_detect->align;
                    break;
                }
            }
            p_detect++;
        }
    }

    // Found anything? Check if error or ACC is really missing.
    if (!sensor_link.acc.hw_name) {
        // Nothing was found and we have a forced sensor that isn't present.
        p_detect = acc_detect;
        while (p_detect->p_func) {
            if ((*p_detect->p_func)(p_detect->p_param)) {
                sensor_link.acc.align = p_detect->align;
                break;
            }
            p_detect++;
        }
    }
    if (!sensor_link.acc.hw_name) return;

    sensorsSet(SENSOR_ACC);
}

static void detectBaro(int baroHardwareToUse)
{
#ifndef BARO
    UNUSED(baroHardwareToUse);
#else
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    extern const sensorDisable_t baro_disable[];
    const sensorDisable_t* p_disable = baro_disable;

    extern const sensorDetect_t baro_detect[];
    const sensorDetect_t* p_detect;

    while (p_disable->p_func) {
        (*p_disable->p_func)(p_disable->p_param);
        p_disable++;
    }

    if (baroHardwareToUse > SENSOR_DEFAULT) {
        p_detect = baro_detect;
        while (p_detect->p_func) {
            if (p_detect->sensor_no >= baroHardwareToUse) {
                if ((*p_detect->p_func)(p_detect->p_param)) break;
            }
            p_detect++;
        }
    }

    // Found anything? Check if error or ACC is really missing.
    if (!sensor_link.baro.hw_name) {
        // Nothing was found and we have a forced sensor that isn't present.
        p_detect = baro_detect;
        while (p_detect->p_func) {
            if ((*p_detect->p_func)(p_detect->p_param)) break;
            p_detect++;
        }
    }
    if (!sensor_link.baro.hw_name) return;

    sensorsSet(SENSOR_BARO);
#endif
}

#ifdef MAG
static void detectMag(int magHardwareToUse)
{
    extern const sensorDetect_t mag_detect[];
    const sensorDetect_t* p_detect;

    sensor_link.mag.align = IMUS_ALIGN_DEFAULT;

    if (magHardwareToUse > SENSOR_DEFAULT) {
        p_detect = mag_detect;
		while (p_detect->p_func) {
            if (p_detect->sensor_no >= magHardwareToUse) {
                if ((*p_detect->p_func)(p_detect->p_param)) {
                    sensor_link.mag.align = p_detect->align;
                    break;
                }
            }
			p_detect++;
		}
    }

    if (!sensor_link.mag.hw_name) {
        // Nothing was found and we have a forced sensor that isn't present.
        p_detect = mag_detect;
		while (p_detect->p_func) {
            if ((*p_detect->p_func)(p_detect->p_param)) {
                sensor_link.mag.align = p_detect->align;
                break;
            }
			p_detect++;
		}
    }
    if (!sensor_link.mag.hw_name) return;

    sensorsSet(SENSOR_MAG);
}
#endif

static void reconfigureAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    if (sensorAlignmentConfig->gyro_align != IMUS_ALIGN_DEFAULT) {
        sensor_link.gyro.align = sensorAlignmentConfig->gyro_align;
    }
    if (sensorAlignmentConfig->acc_align != IMUS_ALIGN_DEFAULT) {
        sensor_link.acc.align = sensorAlignmentConfig->acc_align;
    }
#ifdef MAG
    if (sensorAlignmentConfig->mag_align != IMUS_ALIGN_DEFAULT) {
        sensor_link.mag.align = sensorAlignmentConfig->mag_align;
    }
#endif
}

bool sensorsAutodetect(sensorAlignmentConfig_t *sensorAlignmentConfig, gyro_param_t* p_gyro_param, 
        uint8_t accHardwareToUse, uint8_t magHardwareToUse, uint8_t baroHardwareToUse,
        int16_t magDeclinationFromConfig) {

    int16_t deg, min;

#ifndef MAG
    UNUSED(magHardwareToUse);
#endif

    if (!detectGyro()) {
        return false;
    }
    detectAcc(accHardwareToUse);
    detectBaro(baroHardwareToUse);


    // Now time to init things, acc first
    if (sensors(SENSOR_ACC))
        sensor_link.acc.init(NULL);
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyroUpdateSampleRate(p_gyro_param);   // Set gyro sampling rate divider before initialization
    sensor_link.gyro.init(p_gyro_param);

#ifdef MAG
    detectMag(magHardwareToUse);
#endif

    reconfigureAlignment(sensorAlignmentConfig);

    // FIXME extract to a method to reduce dependencies, maybe move to sensors_compass.c
    if (sensors(SENSOR_MAG)) {
        // calculate magnetic declination
        deg = magDeclinationFromConfig / 100;
        min = magDeclinationFromConfig % 100;

        sensor_link.mag.declination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
    } else {
        sensor_link.mag.declination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
    }

    return true;
}

