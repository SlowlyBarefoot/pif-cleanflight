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

#include "sensors/boardalignment.h"

#include "core/pif_task.h"
#include "sensor/pif_imu_sensor.h"
#include "sensor/pif_sensor_event.h"

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

struct sensor_link_s;
typedef struct sensor_link_s sensor_link_t;

typedef void (*imuInitFuncPtr)(void* p_param);    // imu init prototype
typedef bool (*imuReadFuncPtr)(int32_t *data);    // imu read prototype
typedef void (*baroOpFuncPtr)(void);                                            // baro start operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);  // baro calculation (filled params are pressure and temperature)
typedef void (*sonarInitFuncPtr)(void* p_param);                                // sonar init prototype

struct sensor_link_s {
    // in
    PifTask* p_task_altitude;

    // in/out
    PifImuSensor imu_sensor;

    struct {
        // in
        PifTask* p_task;

        // out
        const char* hw_name;
        PifImuSensorAlign align;
        imuInitFuncPtr init;                            // initialize function
        imuReadFuncPtr read;                            // read 3 axis data function
        imuReadFuncPtr temperature;                     // read temperature if available
        bool can_sync;
        float scale;                                    // scalefactor
    } gyro;

    struct {
        // out
        const char* hw_name;
        PifImuSensorAlign align;
        imuInitFuncPtr init;                            // initialize function
        imuReadFuncPtr read;                            // read 3 axis data function
        char revisionCode;                              // a revision code for the sensor, if known

        // in/out
        uint16_t acc_1G;
    } acc;

    struct {
        // out
        const char* hw_name;
        PifImuSensorAlign align;
        imuInitFuncPtr init;                            // initialize function
        imuReadFuncPtr read;                            // read 3 axis data function
        float declination;                              // calculated at startup from config
    } mag;

    struct {
        // in
        PifEvtBaroRead evt_read;

        // out
        const char* hw_name;
        PifTask* p_task;
        uint16_t ut_delay;
        uint16_t up_delay;
        baroOpFuncPtr start_ut;
        baroOpFuncPtr get_ut;
        baroOpFuncPtr start_up;
        baroOpFuncPtr get_up;
        baroCalculateFuncPtr calculate;
        float temperature;                            // Use temperature for telemetry. unit degrees C
        int32_t BaroAlt;
    } baro;

    struct {
        // in
        uint16_t period;

        // out
        const char* hw_name;
        int16_t maxRangeCm;
        // these are full detection cone angles, maximum tilt is half of this
        int16_t detectionConeDeciDegrees; // detection cone angle as in HC-SR04 device spec
        int16_t detectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone
        PifTask* p_task;
        int32_t distance;
    } sonar;
};

typedef void (*sensorDisableFuncPtr)(void* p_param);

typedef struct sensor_disable_s {
	sensorDisableFuncPtr p_func;
	void* p_param;
} sensorDisable_t;

typedef bool (*sensorDetectFuncPtr)(void* p_param);

typedef struct sensor_detect_s {
    int sensor_no;
    PifImuSensorAlign align;
	sensorDetectFuncPtr p_func;
	void* p_param;
} sensorDetect_t;

extern sensor_link_t sensor_link;

extern void initSensorLink(boardAlignment_t *boardAlignment);
