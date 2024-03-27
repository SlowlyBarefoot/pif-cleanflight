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

#include "common/time.h"

#include "core/pif_task.h"

typedef struct {
    const char * taskName;
    const char * subTaskName;
    bool         isEnabled;
    timeDelta_t  latestDeltaTime;
    timeUs_t     maxExecutionTime;
    timeUs_t     averageExecutionTime;
} cfTaskInfo_t;

typedef enum {
    /* Actual tasks */
    TASK_GYROPID = 0,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_RX_CHECK,
    TASK_RX,
    TASK_SERIAL_CHECK,
    TASK_SERIAL,
    TASK_DISPATCH,
    TASK_BATTERY_VOLTAGE,
    TASK_BATTERY_CURRENT,
    TASK_BATTERY_ALERTS,
#ifdef BEEPER
    TASK_BEEPER,
#endif
#ifdef GPS
    TASK_GPS,
#endif
#ifdef MAG
    TASK_COMPASS,
#endif
#ifdef BARO
    TASK_BARO,
#endif
#ifdef SONAR
    TASK_SONAR,
#endif
#if defined(BARO) || defined(SONAR)
    TASK_ALTITUDE,
#endif
#ifdef USE_DASHBOARD
    TASK_DASHBOARD,
#endif
#ifdef TELEMETRY
    TASK_TELEMETRY,
#endif
#ifdef LED_STRIP
    TASK_LEDSTRIP,
#endif
#ifdef TRANSPONDER
    TASK_TRANSPONDER,
#endif
#ifdef STACK_CHECK
    TASK_STACK_CHECK,
#endif
#ifdef OSD
    TASK_OSD,
#endif
#ifdef USE_OSD_SLAVE
    TASK_OSD_SLAVE,
#endif
#ifdef USE_BST
    TASK_BST_MASTER_PROCESS,
#endif
#ifdef USE_ESC_SENSOR
    TASK_ESC_SENSOR,
#endif
#ifdef CMS
    TASK_CMS,
#endif
#ifdef VTX_CONTROL
    TASK_VTXCTRL,
#endif
#ifdef USE_CAMERA_CONTROL
    TASK_CAMCTRL,
#endif

#ifdef USE_RCDEVICE
    TASK_RCDEVICE,
#endif

    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} cfTaskId_e;

typedef struct {
    // Configuration
    const char * taskName;
    const char * subTaskName;
    PifEvtTaskLoop taskFunc;
    uint16_t desiredPeriod;     // target period of execution
    PifTaskMode taskMode;

    /* PIF */
    PifTask* p_task;
} cfTask_t;

extern cfTask_t cfTasks[TASK_COUNT];

void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t *taskInfo);
void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodMicros);
bool setTaskEnabled(cfTaskId_e taskId, bool newEnabledState);
timeDelta_t getTaskDeltaTime(cfTaskId_e taskId);

void schedulerInit(void);
uint16_t taskSystem(PifTask *p_task);

#define LOAD_PERCENTAGE_ONE 100

#define isSystemOverloaded() (pif_performance._use_rate >= LOAD_PERCENTAGE_ONE)
