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

#include "pif_linker.h"

//#define SCHEDULER_DEBUG

typedef struct {
    const char * taskName;
    bool         isEnabled;
    uint32_t     desiredPeriod;
    uint32_t     latestDeltaTime;
    uint32_t     maxExecutionTime;
    uint32_t     averageExecutionTime;
} cfTaskInfo_t;

typedef enum {
    /* Actual tasks */
    TASK_GYROPID = 0,
    TASK_ACCEL,
    TASK_SERIAL,
#ifdef BEEPER
    TASK_BEEPER,
#endif
    TASK_BATTERY,
    TASK_RX,
#ifdef GPS
    TASK_GPS,
#endif
#ifdef MAG
    TASK_COMPASS,
#endif
#if defined(BARO) && !defined(BARO_PIF)
    TASK_BARO,
#endif
#if defined(SONAR) && !defined(SONAR_PIF)
    TASK_SONAR,
#endif
#if defined(BARO) || defined(SONAR)
    TASK_ALTITUDE,
#endif
#ifdef DISPLAY
    TASK_DISPLAY,
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

    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} cfTaskId_e;

typedef struct {
    /* Configuration */
    const char * taskName;
    PifEvtTaskLoop taskFunc;
    PifTaskMode taskMode;
    uint8_t disallow_yield_id;
    uint32_t desiredPeriod;     // target period of execution

    /* PIF */
    PifTask* p_task;
} cfTask_t;

extern cfTask_t cfTasks[TASK_COUNT];

void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t * taskInfo);
void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodMicros);
bool setTaskEnabled(cfTaskId_e taskId, bool newEnabledState);

#define LOAD_PERCENTAGE_ONE 100

#define isSystemOverloaded() (pif_performance._use_rate >= LOAD_PERCENTAGE_ONE)
