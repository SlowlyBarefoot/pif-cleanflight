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

#ifdef USE_OSD_SLAVE
#include "core/pif_task.h"

#include "common/time.h"

struct displayPort_s;

extern bool osdSlaveIsLocked;

// init
void osdSlaveInit(struct displayPort_s *osdDisplayPort);

// task api
uint16_t osdSlaveUpdate(PifTask *p_task);

// msp api
void osdSlaveHeartbeat(void);
void osdSlaveClearScreen(void);
void osdSlaveWriteChar(const uint8_t x, const uint8_t y, const uint8_t c);
void osdSlaveWrite(const uint8_t x, const uint8_t y, const char *s);

void osdSlaveDrawScreen(void);

#endif
