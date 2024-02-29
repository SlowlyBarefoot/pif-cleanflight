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
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "scheduler/scheduler.h"

#include "config/config_unittest.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

// DEBUG_SCHEDULER, timings for:
// 0 - gyroUpdate()
// 1 - pidController()
// 2 - time spent in scheduler
// 3 - time spent executing check function


#ifndef SKIP_TASK_STATISTICS
void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t * taskInfo)
{
    taskInfo->taskName = cfTasks[taskId].taskName;
    taskInfo->subTaskName = cfTasks[taskId].subTaskName;
    taskInfo->isEnabled = cfTasks[taskId].p_task != NULL;
    taskInfo->desiredPeriod = cfTasks[taskId].desiredPeriod;
    if (cfTasks[taskId].taskMode != TM_PERIOD_US) taskInfo->desiredPeriod *= 1000;
    if (taskInfo->isEnabled) {
        taskInfo->maxExecutionTime = cfTasks[taskId].p_task->_max_execution_time;
        taskInfo->averageExecutionTime = pifTask_GetAverageExecuteTime(cfTasks[taskId].p_task);
        if (cfTasks[taskId].taskMode != TM_PERIOD_US) {
            taskInfo->latestDeltaTime = 1000 * pifTask_GetAverageDeltaTime(cfTasks[taskId].p_task);
        }
        else {
            taskInfo->latestDeltaTime = pifTask_GetAverageDeltaTime(cfTasks[taskId].p_task);
        }
    }
}
#endif

void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodMicros)
{
    cfTask_t *task = NULL;

    if (taskId == TASK_SELF) {
        if (pifTaskManager_CurrentTask()) {
            task = pifTaskManager_CurrentTask()->_p_client;
            if (task) task->desiredPeriod = MAX(SCHEDULER_DELAY_LIMIT, (timeDelta_t)newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
        }
    } else if (taskId < TASK_COUNT) {
        task = &cfTasks[taskId];
        task->desiredPeriod = MAX(SCHEDULER_DELAY_LIMIT, (timeDelta_t)newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
    if (task && task->p_task) {
        if (task->taskMode == TM_PERIOD_US) {
            pifTask_ChangePeriod(task->p_task, task->desiredPeriod);
        }
        else {
            pifTask_ChangePeriod(task->p_task, task->desiredPeriod / 1000);
        }
    }
}

bool setTaskEnabled(cfTaskId_e taskId, bool enabled)
{
    if (taskId < TASK_COUNT) {
        cfTask_t *task = &cfTasks[taskId];
        if (enabled && task->taskFunc) {
            task->p_task = pifTaskManager_Add(task->taskMode, task->desiredPeriod, task->taskFunc, task, TRUE);
            if (!task->p_task) return false;
        } else {
            if (task->p_task) {
                pifTaskManager_Remove(task->p_task);
                task->p_task = NULL;
            }
        }
    }
    pif_Delay1us(1861);
    return true;
}

timeDelta_t getTaskDeltaTime(cfTaskId_e taskId)
{
    if (taskId == TASK_SELF) {
        return pifTaskManager_CurrentTask()->_delta_time;
    } else if (taskId < TASK_COUNT) {
        return cfTasks[taskId].p_task->_delta_time;
    } else {
        return 0;
    }
}

void schedulerInit(void)
{
    int i;

    for (i = 0; i < TASK_COUNT; i++) {
        cfTasks[i].p_task = NULL;
    }
}
