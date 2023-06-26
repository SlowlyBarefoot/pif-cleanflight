#pragma once

#include "core/pif_task.h"
#include "sensor/pif_imu_sensor.h"
#include "sensor/pif_sensor_event.h"

#define PIF_ID_UART(N)              (0x100 + (N))   // N : 0=UART1, 1=UART2, 2=UART3
#define PIF_ID_UART_2_IDX(N)        ((N) - 0x100)   // N : 0x100=UART1, 0x101=UART2, 0x102=UART3

#define PIF_ID_MSP(N)               (0x110 + (N))   // N : 0=main port, 1=flex port
#define PIF_ID_MSP_2_IDX(N)         ((N) - 0x110)   // N : 0x110=main port, 0x111=flex port

struct sensor_link_s;
typedef struct sensor_link_s sensor_link_t;

typedef void (*imuInitFuncPtr)(void* p_param);    // imu init prototype
typedef bool (*imuReadFuncPtr)(float* p_data);    // imu read prototype
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
