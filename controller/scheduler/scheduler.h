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

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

#define TASK_SELF -1

typedef struct {
    const char * taskName;
    bool         isEnabled;
    uint32_t     desiredPeriod;
    uint32_t     maxExecutionTime;
    uint32_t     averageExecutionTime;
    uint32_t     latestDeltaTime;
} cfTaskInfo_t;

typedef struct {
    /* Configuration */
    const char * taskName;
    PifEvtTaskLoop taskFunc;
    PifTaskMode taskMode;
    uint32_t desiredPeriod;         // target period of execution

    /* PIF */
    PifTask* p_task;
} cfTask_t;

extern cfTask_t* taskQueueArray[];
extern const uint32_t taskQueueArraySize;
extern const uint32_t taskCount;
extern cfTask_t cfTasks[];

void getTaskInfo(const int taskId, cfTaskInfo_t *taskInfo);
void rescheduleTask(const int taskId, uint32_t newPeriodMicros, PifTaskMode mode);
void setTaskEnabled(const int taskId, bool newEnabledState);
uint32_t getTaskDeltaTime(const int taskId);

#define isSystemOverloaded() (pif_performance._use_rate > 100)
