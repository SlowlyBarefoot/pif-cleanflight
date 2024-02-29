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

#include "build/debug.h"

#include "cms/cms.h"

#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/stack_check.h"
#include "drivers/transponder_ir.h"
#include "drivers/vtx_common.h"

#include "fc/cli.h"
#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/fc_dispatch.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/osd_slave.h"
#include "io/serial.h"
#include "io/transponder_ir.h"
#include "io/vtx_tramp.h" // Will be gone
#include "io/rcdevice_cam.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/sonar.h"

#include "scheduler/scheduler.h"

#include "telemetry/telemetry.h"

#ifdef USE_BST
uint16_t taskBstMasterProcess(PifTask *p_task);
#endif

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

static uint16_t taskHandleSerial(PifTask *p_task)
{
    UNUSED(p_task);

    if (!mspSerialWaiting()) return 0;

#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return 0;
    }
#endif
#ifndef OSD_SLAVE
    bool evaluateMspData = ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;
#else
    bool evaluateMspData = osdSlaveIsLocked ?  MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;;
#endif
    mspSerialProcess(evaluateMspData, mspFcProcessCommand, mspFcProcessReply);
    return 0;
}

uint16_t taskBatteryAlerts(PifTask *p_task)
{
    UNUSED(p_task);

    if (!ARMING_FLAG(ARMED)) {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates((*pif_act_timer1us)());
    batteryUpdateAlarms();
    return 0;
}

#ifndef USE_OSD_SLAVE
static uint16_t taskUpdateAccelerometer(PifTask *p_task)
{
    UNUSED(p_task);

    accUpdate(&accelerometerConfigMutable()->accelerometerTrims);
    return 0;
}

static uint16_t taskUpdateRxMain(PifTask *p_task)
{
    timeUs_t currentTimeUs = (*pif_act_timer1us)();

    UNUSED(p_task);

    if (!rxUpdateCheck(currentTimeUs)) return 0;

    processRx(currentTimeUs);
    isRXDataNew = true;

#if !defined(BARO) && !defined(SONAR)
    // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
    updateRcCommands();
#endif
    updateArmingStatus();

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        updateAltHoldState();
    }
#endif

#ifdef SONAR
    if (sensors(SENSOR_SONAR)) {
        updateSonarAltHoldState();
    }
#endif
    return 0;
}
#endif

#ifdef MAG
static uint16_t taskUpdateCompass(PifTask *p_task)
{
    UNUSED(p_task);

    if (sensors(SENSOR_MAG)) {
        compassUpdate((*pif_act_timer1us)(), &compassConfigMutable()->magZero);
    }
    return 0;
}
#endif

#ifdef BARO
static uint16_t taskUpdateBaro(PifTask *p_task)
{
    UNUSED(p_task);

    if (sensors(SENSOR_BARO)) {
        const uint32_t newDeadline = baroUpdate();
        if (newDeadline != 0) {
            rescheduleTask(TASK_SELF, newDeadline);
        }
    }
    return 0;
}
#endif

#if defined(BARO) || defined(SONAR)
static uint16_t taskCalculateAltitude(PifTask *p_task)
{
    UNUSED(p_task);

    if (false
#if defined(BARO)
        || (sensors(SENSOR_BARO) && isBaroReady())
#endif
#if defined(SONAR)
        || sensors(SENSOR_SONAR)
#endif
        ) {
        calculateEstimatedAltitude((*pif_act_timer1us)());
    }
    return 0;
}
#endif

#ifdef TELEMETRY
static uint16_t taskTelemetry(PifTask *p_task)
{
    UNUSED(p_task);

    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess((*pif_act_timer1us)());
    }
    return 0;
}
#endif

#ifdef VTX_CONTROL
// Everything that listens to VTX devices
uint16_t taskVtxControl(PifTask *p_task)
{
    UNUSED(p_task);

    if (ARMING_FLAG(ARMED))
        return 0;

#ifdef VTX_COMMON
    vtxCommonProcess((*pif_act_timer1us)());
#endif
    return 0;
}
#endif


#ifdef USE_CAMERA_CONTROL
uint16_t taskCameraControl(PifTask *p_task)
{
    UNUSED(p_task);

    if (ARMING_FLAG(ARMED)) {
        return 0;
    }

    cameraControlProcess((*pif_act_timer1us)());
    return 0;
}
#endif

void fcTasksInit(void)
{
    schedulerInit();
    setTaskEnabled(TASK_SERIAL, true);

    const bool useBatteryVoltage = batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);
    const bool useBatteryCurrent = batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);
#ifdef USE_OSD_SLAVE
    const bool useBatteryAlerts = batteryConfig()->useVBatAlerts || batteryConfig()->useConsumptionAlerts;
#else
    const bool useBatteryAlerts = batteryConfig()->useVBatAlerts || batteryConfig()->useConsumptionAlerts || feature(FEATURE_OSD);
#endif
    setTaskEnabled(TASK_BATTERY_ALERTS, (useBatteryVoltage || useBatteryCurrent) && useBatteryAlerts);

#ifdef TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, feature(FEATURE_TRANSPONDER));
#endif

#ifdef STACK_CHECK
    setTaskEnabled(TASK_STACK_CHECK, true);
#endif

#ifdef USE_OSD_SLAVE
    setTaskEnabled(TASK_OSD_SLAVE, true);
#else
    if (sensors(SENSOR_GYRO)) {
        rescheduleTask(TASK_GYROPID, gyro.targetLooptime);
        setTaskEnabled(TASK_GYROPID, true);
    }

    if (sensors(SENSOR_ACC)) {
        setTaskEnabled(TASK_ACCEL, true);
        rescheduleTask(TASK_ACCEL, acc.accSamplingInterval);
    }

    setTaskEnabled(TASK_ATTITUDE, sensors(SENSOR_ACC));

    rescheduleTask(TASK_SERIAL, TASK_PERIOD_HZ(serialConfig()->serial_update_rate_hz));


    setTaskEnabled(TASK_RX, true);

    setTaskEnabled(TASK_DISPATCH, dispatchIsEnabled());

#ifdef BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif
#ifdef GPS
    setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
#endif
#ifdef MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#endif
#ifdef BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif
#ifdef SONAR
    setTaskEnabled(TASK_SONAR, sensors(SENSOR_SONAR));
#endif
#if defined(BARO) || defined(SONAR)
    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || sensors(SENSOR_SONAR));
#endif
#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, feature(FEATURE_DASHBOARD));
#endif
#ifdef TELEMETRY
    setTaskEnabled(TASK_TELEMETRY, feature(FEATURE_TELEMETRY));
    if (feature(FEATURE_TELEMETRY)) {
        if (rxConfig()->serialrx_provider == SERIALRX_JETIEXBUS) {
            // Reschedule telemetry to 500hz for Jeti Exbus
            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        } else if (rxConfig()->serialrx_provider == SERIALRX_CRSF) {
            // Reschedule telemetry to 500hz, 2ms for CRSF
            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        }
    }
#endif
#ifdef LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
#endif
#ifdef TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, feature(FEATURE_TRANSPONDER));
#endif
#ifdef OSD
    setTaskEnabled(TASK_OSD, feature(FEATURE_OSD));
#endif
#ifdef USE_BST
    setTaskEnabled(TASK_BST_MASTER_PROCESS, true);
#endif
#ifdef USE_ESC_SENSOR
    setTaskEnabled(TASK_ESC_SENSOR, feature(FEATURE_ESC_SENSOR));
#endif
#ifdef CMS
#ifdef USE_MSP_DISPLAYPORT
    setTaskEnabled(TASK_CMS, true);
#else
    setTaskEnabled(TASK_CMS, feature(FEATURE_OSD) || feature(FEATURE_DASHBOARD));
#endif
#endif
#ifdef VTX_CONTROL
#if defined(VTX_RTC6705) || defined(VTX_SMARTAUDIO) || defined(VTX_TRAMP)
    setTaskEnabled(TASK_VTXCTRL, true);
#endif
#endif
#ifdef USE_CAMERA_CONTROL
    setTaskEnabled(TASK_CAMCTRL, true);
#endif
#ifdef USE_RCDEVICE
    setTaskEnabled(TASK_RCDEVICE, rcdeviceIsEnabled());
#endif
#endif
}

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
#ifdef USE_OSD_SLAVE
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .taskMode = TM_PERIOD_US
#else
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .taskMode = TM_PERIOD_US
#endif
    },

    [TASK_BATTERY_ALERTS] = {
        .taskName = "BATTERY_ALERTS",
        .taskFunc = taskBatteryAlerts,
        .desiredPeriod = TASK_PERIOD_HZ(5) / 1000,  // 5 Hz
        .taskMode = TM_PERIOD_MS
    },

    [TASK_BATTERY_VOLTAGE] = {
        .taskName = "BATTERY_VOLTAGE",
        .taskFunc = batteryUpdateVoltage,
        .desiredPeriod = TASK_PERIOD_HZ(50) / 1000,
        .taskMode = TM_PERIOD_MS
    },
    [TASK_BATTERY_CURRENT] = {
        .taskName = "BATTERY_CURRENT",
        .taskFunc = batteryUpdateCurrentMeter,
        .desiredPeriod = TASK_PERIOD_HZ(50) / 1000,
        .taskMode = TM_PERIOD_MS
    },

#ifdef TRANSPONDER
    [TASK_TRANSPONDER] = {
        .taskName = "TRANSPONDER",
        .taskFunc = transponderUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(250),       // 250 Hz, 4ms
        .taskMode = TM_PERIOD_US
    },
#endif

#ifdef STACK_CHECK
    [TASK_STACK_CHECK] = {
        .taskName = "STACKCHECK",
        .taskFunc = taskStackCheck,
        .desiredPeriod = TASK_PERIOD_HZ(10) / 1000, // 10 Hz
        .taskMode = TM_IDLE_MS
    },
#endif

#ifdef USE_OSD_SLAVE
    [TASK_OSD_SLAVE] = {
        .taskName = "OSD_SLAVE",
        .taskFunc = osdSlaveUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(60) / 1000, // 60 Hz
        .taskMode = TM_PERIOD_MS
    },

#else

    [TASK_GYROPID] = {
        .taskName = "PID",
        .subTaskName = "GYRO",
        .taskFunc = taskMainPidLoop,
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,
        .taskMode = TM_PERIOD_US
    },

    [TASK_ACCEL] = {
        .taskName = "ACCEL",
        .taskFunc = taskUpdateAccelerometer,
        .desiredPeriod = TASK_PERIOD_HZ(1000),      // 1000Hz, every 1ms
        .taskMode = TM_PERIOD_US
    },

    [TASK_ATTITUDE] = {
        .taskName = "ATTITUDE",
        .taskFunc = imuUpdateAttitude,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .taskMode = TM_PERIOD_US
    },

    [TASK_RX] = {
        .taskName = "RX",
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = TASK_PERIOD_HZ(50) / 1000, // If event-based scheduling doesn't work, fallback to periodic scheduling
        .taskMode = TM_PERIOD_MS
    },

    [TASK_DISPATCH] = {
        .taskName = "DISPATCH",
        .taskFunc = dispatchProcess,
        .desiredPeriod = TASK_PERIOD_HZ(1000),
        .taskMode = TM_PERIOD_US
    },

#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = beeperUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz
        .taskMode = TM_PERIOD_US
    },
#endif

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = gpsUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),        // Required to prevent buffer overruns if running at 115200 baud (115 bytes / period < 256 bytes buffer)
        .taskMode = TM_PERIOD_US
    },
#endif

#ifdef MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = TASK_PERIOD_HZ(10) / 1000,  // Compass is updated at 10 Hz
        .taskMode = TM_PERIOD_MS
    },
#endif

#ifdef BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = TASK_PERIOD_HZ(20) / 1000,
        .taskMode = TM_PERIOD_MS
    },
#endif

#ifdef SONAR
    [TASK_SONAR] = {
        .taskName = "SONAR",
        .taskFunc = sonarUpdate,
        .desiredPeriod = TASK_PERIOD_MS(70) / 1000,  // 70ms required so that SONAR pulses do not interfere with each other
        .taskMode = TM_PERIOD_MS
    },
#endif

#if defined(BARO) || defined(SONAR)
    [TASK_ALTITUDE] = {
        .taskName = "ALTITUDE",
        .taskFunc = taskCalculateAltitude,
        .desiredPeriod = TASK_PERIOD_HZ(40) / 1000,
        .taskMode = TM_PERIOD_MS
    },
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = {
        .taskName = "DASHBOARD",
        .taskFunc = dashboardUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10) / 1000,
        .taskMode = TM_PERIOD_MS
    },
#endif

#ifdef OSD
    [TASK_OSD] = {
        .taskName = "OSD",
        .taskFunc = osdUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(60) / 1000,  // 60 Hz
        .taskMode = TM_PERIOD_MS
    },
#endif

#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = TASK_PERIOD_HZ(250),       // 250 Hz, 4ms
        .taskMode = TM_PERIOD_US
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = ledStripUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz, 10ms
        .taskMode = TM_PERIOD_US
    },
#endif

#ifdef USE_BST
    [TASK_BST_MASTER_PROCESS] = {
        .taskName = "BST_MASTER_PROCESS",
        .taskFunc = taskBstMasterProcess,
        .desiredPeriod = TASK_PERIOD_HZ(50) / 1000, // 50 Hz, 20ms
        .taskMode = TM_IDLE_MS
    },
#endif

#ifdef USE_ESC_SENSOR
    [TASK_ESC_SENSOR] = {
        .taskName = "ESC_SENSOR",
        .taskFunc = escSensorProcess,
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz, 10ms
        .taskMode = TM_PERIOD_US
    },
#endif

#ifdef CMS
    [TASK_CMS] = {
        .taskName = "CMS",
        .taskFunc = cmsHandler,
        .desiredPeriod = TASK_PERIOD_HZ(60) / 1000, // 60 Hz
        .taskMode = TM_PERIOD_MS
    },
#endif

#ifdef VTX_CONTROL
    [TASK_VTXCTRL] = {
        .taskName = "VTXCTRL",
        .taskFunc = taskVtxControl,
        .desiredPeriod = TASK_PERIOD_HZ(5) / 1000,  // 5 Hz, 200ms
        .taskMode = TM_IDLE_MS
    },
#endif

#ifdef USE_RCDEVICE
    [TASK_RCDEVICE] = {
        .taskName = "RCDEVICE",
        .taskFunc = rcdeviceUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10) / 1000, // 10 Hz, 100ms
        .taskMode = TM_PERIOD_MS
    },
#endif

#ifdef USE_CAMERA_CONTROL
    [TASK_CAMCTRL] = {
        .taskName = "CAMCTRL",
        .taskFunc = taskCameraControl,
        .desiredPeriod = TASK_PERIOD_HZ(5) / 1000,  // 5 Hz, 200ms
        .taskMode = TM_IDLE_MS
    },
#endif
#endif
};
