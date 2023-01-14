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

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <platform.h>

#include "build_config.h"
#include "scheduler.h"
#include "debug.h"

#include "common/maths.h"

#include "drivers/system.h"
#include "config/config_unittest.h"

#define REALTIME_GUARD_INTERVAL_MIN     10
#define REALTIME_GUARD_INTERVAL_MAX     300
#define REALTIME_GUARD_INTERVAL_MARGIN  25

static uint32_t realtimeGuardInterval = REALTIME_GUARD_INTERVAL_MAX;


uint16_t taskSystem(PifTask *p_task)
{
    uint8_t taskId;

    UNUSED(p_task);

    /* Calculate guard interval */
    uint32_t maxNonRealtimeTaskTime = 0;
    for (taskId = 0; taskId < TASK_COUNT; taskId++) {
        maxNonRealtimeTaskTime = MAX(maxNonRealtimeTaskTime, cfTasks[taskId].p_task->_total_execution_time / cfTasks[taskId].p_task->_execution_count);
    }

    realtimeGuardInterval = constrain(maxNonRealtimeTaskTime, REALTIME_GUARD_INTERVAL_MIN, REALTIME_GUARD_INTERVAL_MAX) + REALTIME_GUARD_INTERVAL_MARGIN;
#if defined SCHEDULER_DEBUG
    debug[2] = realtimeGuardInterval;
#endif
    return 0;
}

#ifndef SKIP_TASK_STATISTICS
void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t * taskInfo)
{
    taskInfo->taskName = cfTasks[taskId].taskName;
    taskInfo->mode = cfTasks[taskId].p_task->_mode;
    taskInfo->isEnabled= !cfTasks[taskId].p_task->pause;
    taskInfo->maxExecutionTime = cfTasks[taskId].p_task->_max_execution_time;
    taskInfo->totalExecutionTime = cfTasks[taskId].p_task->_total_execution_time;
    taskInfo->averageExecutionTime = cfTasks[taskId].p_task->_total_execution_time / cfTasks[taskId].p_task->_execution_count;
    taskInfo->averagePeriodTime = cfTasks[taskId].p_task->_total_period_time / cfTasks[taskId].p_task->_execution_count;
}
#endif

BOOL changeTask(cfTaskId_e taskId, PifTaskMode newMode, uint16_t newPeriod)
{
    if (taskId < TASK_COUNT) {
        if (pifTask_ChangeMode(cfTasks[taskId].p_task, newMode, newPeriod)) {
            cfTasks[taskId].taskMode = newMode;
            cfTasks[taskId].desiredPeriod = newPeriod;
            return TRUE;
        }
    }
    return FALSE;
}

BOOL createTask(cfTaskId_e taskId, BOOL newEnabledState)
{
    if (taskId < TASK_COUNT) {
		cfTasks[taskId].p_task = pifTaskManager_Add(cfTasks[taskId].taskMode, cfTasks[taskId].desiredPeriod, cfTasks[taskId].taskFunc, NULL, newEnabledState);
        cfTasks[taskId].p_task->disallow_yield_id = cfTasks[taskId].disallow_yield_id;
	    return cfTasks[taskId].p_task != 0;
	}
	return FALSE;
}
