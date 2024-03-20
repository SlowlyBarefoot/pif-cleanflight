#pragma once

#include "core/pif_timer.h"
#include "sensor/pif_imu_sensor.h"


#define TASK_SIZE				30
#define TIMER_1MS_SIZE			3

#define DISALLOW_YIELD_ID_I2C   1


extern PifImuSensor g_imu_sensor;
extern PifTimerManager g_timer_1ms;
