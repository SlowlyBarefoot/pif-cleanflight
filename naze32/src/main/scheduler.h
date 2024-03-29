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

//#define SCHEDULER_DEBUG

typedef enum {
    /* Actual tasks */
    TASK_GYROPID = 0,
    TASK_SYSTEM,
    TASK_ACCEL,
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
#ifdef BARO
    TASK_BARO,
#endif
#ifdef SONAR
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
    TASK_NONE = TASK_COUNT
} cfTaskId_e;

typedef struct {
    /* Configuration */
    const char* taskName;
    PifEvtTaskLoop taskFunc;
    uint16_t desiredPeriod;     // target period of execution
    PifTaskMode taskMode;
    uint8_t disallow_yield_id;
    bool isCreate;

    /* PIF */
    PifTask* p_task;
} cfTask_t;

extern cfTask_t cfTasks[TASK_COUNT];

void calcurateTaskTime();
BOOL changeTask(cfTaskId_e taskId, PifTaskMode newMode, uint16_t newPeriod);
BOOL createTask(cfTaskId_e taskId, BOOL newEnabledState);

#define LOAD_PERCENTAGE_ONE 100

#define isSystemOverloaded() (pif_performance._use_rate >= LOAD_PERCENTAGE_ONE)
