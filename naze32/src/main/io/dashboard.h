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

#include "common/time.h"
#include "config/parameter_group.h"
#include "drivers/bus_i2c.h"

#define ENABLE_DEBUG_DASHBOARD_PAGE

#ifdef OLED_I2C_INSTANCE
#define DASHBOARD_I2C_INSTANCE  OLED_I2C_INSTANCE
#else
#define DASHBOARD_I2C_INSTANCE  I2CDEV_1
#endif

#define DASHBOARD_I2C_ADDRESS   0x3C     // OLED at address 0x3C in 7bit

typedef struct dashboardConfig_s {
    I2CDevice device;
    uint8_t   address;
} dashboardConfig_t;

PG_DECLARE(dashboardConfig_t, dashboardConfig);

typedef enum {
    PAGE_WELCOME,
    PAGE_ARMED,
    PAGE_BATTERY,
    PAGE_SENSORS,
    PAGE_RX,
    PAGE_PROFILE,
#ifndef SKIP_TASK_STATISTICS
    PAGE_TASKS,
#endif
#ifdef GPS
    PAGE_GPS,
#endif
#ifdef ENABLE_DEBUG_DASHBOARD_PAGE
    PAGE_DEBUG,
#endif

    PAGE_COUNT
} pageId_e;

void dashboardInit(void);
void dashboardUpdate(timeUs_t currentTimeUs);

void dashboardShowFixedPage(pageId_e pageId);

void dashboardEnablePageCycling(void);
void dashboardDisablePageCycling(void);
void dashboardResetPageCycling(void);
void dashboardSetNextPageChangeAt(timeUs_t futureMicros);
