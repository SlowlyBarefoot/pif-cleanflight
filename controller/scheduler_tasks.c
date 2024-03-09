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
#include <stdlib.h>
#include <stdint.h>

#include <platform.h>
#include "scheduler.h"

uint16_t taskMainPidLoopChecker(PifTask *p_task);
uint16_t taskUpdateAccelerometer(PifTask *p_task);
uint16_t taskHandleSerial(PifTask *p_task);
uint16_t taskUpdateBeeper(PifTask *p_task);
uint16_t taskUpdateBattery(PifTask *p_task);
uint16_t taskUpdateRxMain(PifTask *p_task);
uint16_t taskProcessGPS(PifTask *p_task);
uint16_t taskUpdateCompass(PifTask *p_task);
uint16_t taskUpdateBaro(PifTask *p_task);
uint16_t taskUpdateSonar(PifTask *p_task);
uint16_t taskCalculateAltitude(PifTask *p_task);
uint16_t taskUpdateDisplay(PifTask *p_task);
uint16_t taskTelemetry(PifTask *p_task);
uint16_t taskLedStrip(PifTask *p_task);
uint16_t taskTransponder(PifTask *p_task);

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_GYROPID] = {
        .taskName = "GYRO/PID",
        .taskFunc = taskMainPidLoopChecker,
        .desiredPeriod = 1000,
        .taskMode = TM_PERIOD_US,
        .disallow_yield_id = DISALLOW_YIELD_ID_I2C,
    },

    [TASK_ACCEL] = {
        .taskName = "ACCEL",
        .taskFunc = taskUpdateAccelerometer,
        .desiredPeriod = 1000000 / 100,     // every 10ms
        .taskMode = TM_PERIOD_US,
        .disallow_yield_id = DISALLOW_YIELD_ID_I2C,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = 1000000 / 100,     // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .taskMode = TM_PERIOD_US,
    },

#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = taskUpdateBeeper,
        .desiredPeriod = 1000000 / 100,     // 100 Hz
        .taskMode = TM_PERIOD_US,
    },
#endif

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = 1000 / 50,      // 50 Hz
        .taskMode = TM_PERIOD_MS,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = 1000 / 50,      // If event-based scheduling doesn't work, fallback to periodic scheduling
        .taskMode = TM_PERIOD_MS,
    },

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = 1000 / 10,      // GPS usually don't go raster than 10Hz
        .taskMode = TM_PERIOD_MS,
    },
#endif

#ifdef MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = 1000 / 10,      // Compass is updated at 10 Hz
        .taskMode = TM_PERIOD_MS,
        .disallow_yield_id = DISALLOW_YIELD_ID_I2C,
    },
#endif

#ifndef BARO_PIF
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = 1000 / 20,
        .taskMode = TM_CHANGE_MS,
        .disallow_yield_id = DISALLOW_YIELD_ID_I2C,
    },
#endif

#ifdef SONAR
    [TASK_SONAR] = {
        .taskName = "SONAR",
        .taskFunc = taskUpdateSonar,
        .desiredPeriod = 1000 / 20,
        .taskMode = TM_PERIOD_MS,
    },
#endif

#if defined(BARO) || defined(SONAR)
    [TASK_ALTITUDE] = {
        .taskName = "ALTITUDE",
        .taskFunc = taskCalculateAltitude,
        .desiredPeriod = 1000 / 40,
        .taskMode = TM_PERIOD_MS,
    },
#endif

#ifdef TRANSPONDER
    [TASK_TRANSPONDER] = {
        .taskName = "TRANSPONDER",
        .taskFunc = taskTransponder,
        .desiredPeriod = 1000000 / 250,         // 250 Hz
        .taskMode = TM_PERIOD_US,
    },
#endif

#ifdef DISPLAY
    [TASK_DISPLAY] = {
        .taskName = "DISPLAY",
        .taskFunc = taskUpdateDisplay,
        .desiredPeriod = 1000 / 10,
        .taskMode = TM_PERIOD_MS,
    },
#endif

#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = 1000000 / 250,         // 250 Hz
        .taskMode = TM_IDLE_US,
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = 1000000 / 100,         // 100 Hz
        .taskMode = TM_IDLE_US,
    },
#endif
};
