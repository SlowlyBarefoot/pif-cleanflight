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

extern int32_t accADC[XYZ_AXIS_COUNT];

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims);
void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse);
