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

#include "scheduler/scheduler.h"
#include "build/debug.h"
#include "build/build_config.h"

#include "common/maths.h"

#include "drivers/system.h"
#include "config/config_unittest.h"

#ifndef SKIP_TASK_STATISTICS
void getTaskInfo(const int taskId, cfTaskInfo_t * taskInfo)
{
    taskInfo->taskName = cfTasks[taskId].taskName;
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

void rescheduleTask(const int taskId, uint32_t newPeriodMicros)
{
    if (taskId < (int)taskCount) {
        cfTasks[taskId].desiredPeriod = MAX(100, newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
        if (cfTasks[taskId].p_task) {
            if (!(cfTasks[taskId].p_task->_mode & TM_UNIT_MASK)) newPeriodMicros /= 1000;
            pifTask_ChangePeriod(cfTasks[taskId].p_task, newPeriodMicros);
        }
    }
}

void setTaskEnabled(const int taskId, bool newEnabledState, PifTaskMode mode)
{
    if (taskId == TASK_SELF || taskId < (int)taskCount) {
        cfTask_t *task = &cfTasks[taskId];
        if (newEnabledState && task->taskFunc) {
            if (!task->p_task) {
                if (mode) task->taskMode = mode;
                task->p_task = pifTaskManager_Add(task->taskMode, task->desiredPeriod, task->taskFunc, NULL, true);
            }
            else {
                task->p_task->pause = false;
            }
        } else {
            if (task->p_task) task->p_task->pause = true;
        }
    }
    pif_Delay1us(1319);
}

uint32_t getTaskDeltaTime(const int taskId)
{
    if (taskId == TASK_SELF || taskId < (int)taskCount) {
        PifTask *p_task = taskId == TASK_SELF ? pifTaskManager_CurrentTask() : cfTasks[taskId].p_task;
        return p_task->_delta_time;
    } else {
        return 0;
    }
}
