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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "fc/fc_tasks.h"

#include "scheduler/scheduler.h"

// No need for a linked list for the queue, since items are only inserted at startup
#ifdef UNIT_TEST
#define TASK_QUEUE_ARRAY_SIZE (TASK_COUNT + 2) // 1 extra space so test code can check for buffer overruns
#else
#define TASK_QUEUE_ARRAY_SIZE (TASK_COUNT + 1) // extra item for NULL pointer at end of queue
#endif

const uint32_t taskQueueArraySize = TASK_QUEUE_ARRAY_SIZE;
const uint32_t taskCount = TASK_COUNT;
cfTask_t* taskQueueArray[TASK_QUEUE_ARRAY_SIZE];

cfTask_t cfTasks[] = {
    [TASK_GYRO] = {
        .taskName = "GYRO",
        .taskFunc = taskGyro,
        .desiredPeriod = TASK_PERIOD_HZ(8000),
        .taskMode = TM_PERIOD_US,
    },
    [TASK_PID] = {
        .taskName = "PID",
        .taskFunc = taskPid,
        .desiredPeriod = TASK_PERIOD_HZ(8000),
        .taskMode = TM_PERIOD_US,
    },

    [TASK_ACCEL] = {
        .taskName = "ACCEL",
        .taskFunc = taskUpdateAccelerometer,
        .desiredPeriod = TASK_PERIOD_MS(1),
        .taskMode = TM_PERIOD_US,
    },

    [TASK_ATTITUDE] = {
        .taskName = "ATTITUDE",
        .taskFunc = taskUpdateAttitude,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .taskMode = TM_PERIOD_US,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = TASK_PERIOD_HZ(50) / 1000,          // If event-based scheduling doesn't work, fallback to periodic scheduling
        .taskMode = TM_PERIOD_MS,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = TASK_PERIOD_HZ(100),         // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .taskMode = TM_PERIOD_US,
    },

    [TASK_HARDWARE_WATCHDOG] = {
        .taskName = "HW_WATCHDOG",
        .taskFunc = taskHardwareWatchdog,
        .desiredPeriod = TASK_PERIOD_HZ(1) / 1000,
        .taskMode = TM_PERIOD_MS,
    },

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = TASK_PERIOD_MS(20) / 1000,
        .taskMode = TM_PERIOD_MS,
    },


#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = taskUpdateBeeper,
        .desiredPeriod = TASK_PERIOD_MS(10) / 1000,
        .taskMode = TM_PERIOD_MS,
    },
#endif

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = TASK_PERIOD_HZ(100),         // 115 (<256) bytes/call @ 115K
        .taskMode = TM_PERIOD_US,
    },
#endif

#ifdef MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = TASK_PERIOD_MS(100),
        .taskMode = TM_PERIOD_US,
    },
#endif

#ifdef BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = TASK_PERIOD_MS(50) / 1000,
        .taskMode = TM_CHANGE_MS,
    },
#endif

#ifdef SONAR
    [TASK_SONAR] = {
        .taskName = "SONAR",
        .taskFunc = taskUpdateSonar,
        .desiredPeriod = TASK_PERIOD_MS(70) / 1000,
        .taskMode = TM_PERIOD_MS,
    },
#endif

#if defined(BARO) || defined(SONAR)
    [TASK_ALTITUDE] = {
        .taskName = "ALTITUDE",
        .taskFunc = taskCalculateAltitude,
        .desiredPeriod = TASK_PERIOD_MS(25) / 1000,
        .taskMode = TM_PERIOD_MS,
    },
#endif

#ifdef TRANSPONDER
    [TASK_TRANSPONDER] = {
        .taskName = "TRANSPONDER",
        .taskFunc = taskTransponder,
        .desiredPeriod = TASK_PERIOD_MS(4),
        .taskMode = TM_PERIOD_US,
    },
#endif

#ifdef OSD
    [TASK_DRAW_SCREEN] = {
        .taskName = "DRAW_SCREEN",
        .taskFunc = taskDrawScreen,
        .desiredPeriod = 1000 / 30,           // 30 Hz
        .taskMode = TM_PERIOD_MS,
    },
#endif

#ifdef DISPLAY
    [TASK_DISPLAY] = {
        .taskName = "DISPLAY",
        .taskFunc = taskUpdateDisplay,
        .desiredPeriod = TASK_PERIOD_MS(100),
        .taskMode = TM_PERIOD_US,
    },
#endif

#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = TASK_PERIOD_MS(4),
        .taskMode = TM_IDLE_US,
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = TASK_PERIOD_MS(10),
        .taskMode = TM_IDLE_US,
    },
#endif
};
