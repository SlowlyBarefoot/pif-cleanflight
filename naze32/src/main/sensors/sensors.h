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

#pragma once

#include "core/pif_task.h"
#include "sensor/pif_imu_sensor.h"
#include "sensor/pif_sensor_event.h"

#include "common/link_driver.h"

#define SENSOR_DEFAULT  1

#define GYRO_MAX    4
#define ACC_MAX     6
#define MAG_MAX     2
#define BARO_MAX    4

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} flightDynamicsTrims_def_t;

typedef union {
    int16_t raw[3];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400
#define CALIBRATING_BARO_CYCLES             200 // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6,
} sensors_e;

typedef struct boardAlignment_s {
    int16_t rollDegrees;
    int16_t pitchDegrees;
    int16_t yawDegrees;
} boardAlignment_t;

typedef struct sensorAlignmentConfig_s {
    PifImuSensorAlign gyro_align;              // gyro alignment
    PifImuSensorAlign acc_align;               // acc alignment
    PifImuSensorAlign mag_align;               // mag alignment
} sensorAlignmentConfig_t;

typedef struct gyro_param_s {
    uint8_t lpf;
    uint8_t sync;
    uint8_t sync_denominator;
    uint32_t looptime;
} gyro_param_t;

extern void initSensorLink(boardAlignment_t *boardAlignment);
