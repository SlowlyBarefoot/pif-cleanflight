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

#include "scheduler.h"
#include "debug.h"

#include "common/maths.h"

#include "drivers/system.h"
#include "config/config_unittest.h"

cfTaskId_e currentTaskId = TASK_NONE;

 
#ifndef SKIP_TASK_STATISTICS
void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t * taskInfo)
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

void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodMicros)
{
    if (taskId < TASK_COUNT) {
        cfTasks[taskId].desiredPeriod = MAX((uint32_t)100, newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
        if (cfTasks[taskId].p_task->_mode & TM_UNIT_MASK) {
            pifTask_ChangePeriod(cfTasks[taskId].p_task, newPeriodMicros);
        }
        else {
            pifTask_ChangePeriod(cfTasks[taskId].p_task, newPeriodMicros / 1000);
        }
    }
}

void setTaskEnabled(cfTaskId_e taskId, bool newEnabledState)
{
    if (taskId < TASK_COUNT) {
        if (newEnabledState) {
            if (!cfTasks[taskId].p_task) {
                cfTasks[taskId].p_task = pifTaskManager_Add(cfTasks[taskId].taskMode, cfTasks[taskId].desiredPeriod, cfTasks[taskId].taskFunc, NULL, newEnabledState);
            }
        }
        else {
            if (cfTasks[taskId].p_task) {
                pifTaskManager_Remove(cfTasks[taskId].p_task);
                cfTasks[taskId].p_task = NULL;
            }
        }
    }
    pif_Delay1us(1031);
}
