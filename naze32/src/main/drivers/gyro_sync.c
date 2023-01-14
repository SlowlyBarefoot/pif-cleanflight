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
#include <stdlib.h>

#include <platform.h>
#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/gyro_sync.h"

#include "config/runtime_config.h"
#include "config/config.h"

uint32_t targetLooptime;
static uint8_t mpuDividerDrops = 0;

void gyroUpdateSampleRate(gyro_param_t* p_gyro_param) {
    int gyroSamplePeriod;

    if (p_gyro_param->sync) {
        if (!p_gyro_param->lpf) {
            gyroSamplePeriod = 125;

        } else {
            gyroSamplePeriod = 1000;
        }

        mpuDividerDrops  = p_gyro_param->sync_denominator - 1;
        targetLooptime = (mpuDividerDrops + 1) * gyroSamplePeriod;
    } else {
    	mpuDividerDrops = 0;
    	targetLooptime = p_gyro_param->looptime;
    }
}

uint8_t gyroMPU6xxxCalculateDivider(void) {
    return mpuDividerDrops;
}
