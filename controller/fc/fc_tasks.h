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

typedef enum {
    /* Actual tasks */
    TASK_GYRO = 0,
    TASK_PID,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_RX_CHECK,
    TASK_RX,
    TASK_SERIAL,
    TASK_BATTERY,
    TASK_HARDWARE_WATCHDOG,
#ifdef BEEPER
    TASK_BEEPER,
#endif
#ifdef GPS
    TASK_GPS,
#endif
#ifdef MAG
    TASK_COMPASS,
#endif
#ifndef BARO_PIF
    TASK_BARO,
#endif
#ifndef SONAR_PIF
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
#ifdef OSD
    TASK_DRAW_SCREEN,
#endif

    /* Count of real tasks */
    TASK_COUNT
} cfTaskId_e;

uint16_t taskGyro(PifTask *p_task);
uint16_t taskPid(PifTask *p_task);
uint16_t taskUpdateAccelerometer(PifTask *p_task);
uint16_t taskUpdateAttitude(PifTask *p_task);
uint16_t taskUpdateRxCheck(PifTask *p_task);
uint16_t taskUpdateRxMain(PifTask *p_task);
uint16_t taskHandleSerial(PifTask *p_task);
uint16_t taskHardwareWatchdog(PifTask *p_task);
uint16_t taskUpdateBeeper(PifTask *p_task);
uint16_t taskUpdateBattery(PifTask *p_task);
uint16_t taskProcessGPS(PifTask *p_task);
uint16_t taskUpdateCompass(PifTask *p_task);
uint16_t taskUpdateBaro(PifTask *p_task);
uint16_t taskUpdateSonar(PifTask *p_task);
uint16_t taskCalculateAltitude(PifTask *p_task);
uint16_t taskUpdateDisplay(PifTask *p_task);
uint16_t taskTelemetry(PifTask *p_task);
uint16_t taskLedStrip(PifTask *p_task);
uint16_t taskTransponder(PifTask *p_task);
uint16_t taskDrawScreen(PifTask *p_task);
