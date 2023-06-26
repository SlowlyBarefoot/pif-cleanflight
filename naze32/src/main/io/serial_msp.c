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
#include <string.h>
#include <math.h>

#include "build_config.h"
#include "debug.h"
#include <platform.h>
#include "scheduler.h"

#include "core/pif_i2c.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "rx/rx.h"
#include "rx/msp.h"

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/transponder_ir.h"
#include "io/asyncfatfs/asyncfatfs.h"

#include "telemetry/telemetry.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"

#include "blackbox/blackbox.h"

#include "mw.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "version.h"
#ifdef NAZE
#include "hardware_revision.h"
#endif

#include "serial_msp.h"

extern uint16_t rssi; // FIXME dependency on mw.c
extern void resetPidProfile(pidProfile_t *pidProfile);

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse);

const char * const flightControllerIdentifier = CLEANFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

typedef struct box_e {
    const uint8_t boxId;         // see boxId_e
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      //
} box_t;

// FIXME remove ;'s
static const box_t boxes[CHECKBOX_ITEM_COUNT + 1] = {
    { BOXARM, "ARM;", 0 },
    { BOXANGLE, "ANGLE;", 1 },
    { BOXHORIZON, "HORIZON;", 2 },
    { BOXBARO, "BARO;", 3 },
    //{ BOXVARIO, "VARIO;", 4 },
    { BOXMAG, "MAG;", 5 },
    { BOXHEADFREE, "HEADFREE;", 6 },
    { BOXHEADADJ, "HEADADJ;", 7 },
    { BOXCAMSTAB, "CAMSTAB;", 8 },
    { BOXCAMTRIG, "CAMTRIG;", 9 },
    { BOXGPSHOME, "GPS HOME;", 10 },
    { BOXGPSHOLD, "GPS HOLD;", 11 },
    { BOXPASSTHRU, "PASSTHRU;", 12 },
    { BOXBEEPERON, "BEEPER;", 13 },
    { BOXLEDMAX, "LEDMAX;", 14 },
    { BOXLEDLOW, "LEDLOW;", 15 },
    { BOXLLIGHTS, "LLIGHTS;", 16 },
    { BOXCALIB, "CALIB;", 17 },
    { BOXGOV, "GOVERNOR;", 18 },
    { BOXOSD, "OSD SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { BOXGTUNE, "GTUNE;", 21 },
    { BOXSONAR, "SONAR;", 22 },
    { BOXSERVO1, "SERVO1;", 23 },
    { BOXSERVO2, "SERVO2;", 24 },
    { BOXSERVO3, "SERVO3;", 25 },
    { BOXBLACKBOX, "BLACKBOX;", 26 },
    { BOXFAILSAFE, "FAILSAFE;", 27 },
    { BOXAIRMODE, "AIR MODE;", 28 },
    { CHECKBOX_ITEM_COUNT, NULL, 0xFF }
};

// this is calculated at startup based on enabled features.
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];
// this is the number of filled indexes in above array
static uint8_t activeBoxIdCount = 0;
// from mixer.c
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

// cause reboot after MSP processing complete
static bool isRebootScheduled = false;

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;


STATIC_UNIT_TESTED mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

static void evtMspReceive(PifMsp* p_owner, PifMspPacket* p_packet, PifIssuerP p_issuer);

static const box_t *findBoxByActiveBoxId(uint8_t activeBoxId)
{
    uint8_t boxIndex;
    const box_t *candidate;
    for (boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        candidate = &boxes[boxIndex];
        if (candidate->boxId == activeBoxId) {
            return candidate;
        }
    }
    return NULL;
}

static const box_t *findBoxByPermenantId(uint8_t permenantId)
{
    uint8_t boxIndex;
    const box_t *candidate;
    for (boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        candidate = &boxes[boxIndex];
        if (candidate->permanentId == permenantId) {
            return candidate;
        }
    }
    return NULL;
}

static void serializeBoxNamesReply(PifMsp* p_owner)
{
    int i, activeBoxId;
    const box_t *box;

    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (i = 0; i < activeBoxIdCount; i++) {
        activeBoxId = activeBoxIds[i];

        box = findBoxByActiveBoxId(activeBoxId);
        if (!box) {
            continue;
        }

        pifMsp_AddAnswer(p_owner, (uint8_t*)box->boxName, strlen(box->boxName));
    }
}

static void serializeSDCardSummaryReply(PifMsp* p_owner)
{
#ifdef USE_SDCARD
    uint8_t flags = 1 /* SD card supported */ ;
    uint8_t state;

    pifMsp_AddAnswer8(p_owner, flags);

    // Merge the card and filesystem states together
    if (!sdcard_isInserted()) {
        state = MSP_SDCARD_STATE_NOT_PRESENT;
    } else if (!sdcard_isFunctional()) {
        state = MSP_SDCARD_STATE_FATAL;
    } else {
        switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
            break;
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                if (sdcard_isInitialized()) {
                    state = MSP_SDCARD_STATE_FS_INIT;
                } else {
                    state = MSP_SDCARD_STATE_CARD_INIT;
                }
            break;
            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
                state = MSP_SDCARD_STATE_FATAL;
            break;
        }
    }

    pifMsp_AddAnswer8(p_owner, state);
    pifMsp_AddAnswer8(p_owner, afatfs_getLastError());
    // Write free space and total space in kilobytes
    pifMsp_AddAnswer32(p_owner, afatfs_getContiguousFreeSpace() / 1024);
    pifMsp_AddAnswer32(p_owner, sdcard_getMetadata()->numBlocks / 2); // Block size is half a kilobyte
#else
    pifMsp_AddAnswer8(p_owner, 0);
    pifMsp_AddAnswer8(p_owner, 0);
    pifMsp_AddAnswer8(p_owner, 0);
    pifMsp_AddAnswer32(p_owner, 0);
    pifMsp_AddAnswer32(p_owner, 0);
#endif
}

static void serializeDataflashSummaryReply(PifMsp* p_owner)
{
#ifdef USE_FLASHFS
    const flashGeometry_t *geometry = flashfsGetGeometry();
    uint8_t flags = (flashfsIsReady() ? 1 : 0) | 2 /* FlashFS is supported */;

    pifMsp_AddAnswer8(p_owner, flags);
    pifMsp_AddAnswer32(p_owner, geometry->sectors);
    pifMsp_AddAnswer32(p_owner, geometry->totalSize);
    pifMsp_AddAnswer32(p_owner, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
#else
    pifMsp_AddAnswer8(p_owner, 0); // FlashFS is neither ready nor supported
    pifMsp_AddAnswer32(p_owner, 0);
    pifMsp_AddAnswer32(p_owner, 0);
    pifMsp_AddAnswer32(p_owner, 0);
#endif
}

#ifdef USE_FLASHFS
static void serializeDataflashReadReply(PifMsp* p_owner, uint32_t address, uint8_t size)
{
    uint8_t buffer[128];
    int bytesRead;

    if (size > sizeof(buffer)) {
        size = sizeof(buffer);
    }

    pifMsp_AddAnswer32(p_owner, address);

    // bytesRead will be lower than that requested if we reach end of volume
    bytesRead = flashfsReadAbs(address, buffer, size);

    for (int i = 0; i < bytesRead; i++) {
        pifMsp_AddAnswer8(p_owner, buffer[i]);
    }
}
#endif

static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->port = serialPort;
}

void mspAllocateSerialPorts(serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);

    serialPort_t *serialPort;

    uint8_t portIndex = 0;

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);

    while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
        mspPort_t *mspPort = &mspPorts[portIndex];
        if (mspPort->port) {
            portIndex++;
            continue;
        }

        serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED, 10);
        if (serialPort) {
            resetMspPort(mspPort, serialPort);
            if (pifMsp_Init(&mspPort->pif_msp, &g_timer_1ms, PIF_ID_MSP(portIndex))) {
                pifMsp_AttachEvtReceive(&mspPort->pif_msp, evtMspReceive, evtMspOtherPacket, serialPort);
                pifMsp_AttachComm(&mspPort->pif_msp, &serialPort->comm);
            }
            portIndex++;
        }

        portConfig = findNextSerialPortConfig(FUNCTION_MSP);
    }
}

void mspReleasePortIfAllocated(serialPort_t *serialPort)
{
    uint8_t portIndex;
    for (portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->port == serialPort) {
            closeSerialPort(serialPort);
            memset(candidateMspPort, 0, sizeof(mspPort_t));
            pifMsp_Clear(&candidateMspPort->pif_msp);
        }
    }
}

void mspInit(serialConfig_t *serialConfig)
{
    // calculate used boxes based on features and fill availableBoxes[] array
    memset(activeBoxIds, 0xFF, sizeof(activeBoxIds));

    activeBoxIdCount = 0;
    activeBoxIds[activeBoxIdCount++] = BOXARM;

    if (sensors(SENSOR_ACC)) {
        activeBoxIds[activeBoxIdCount++] = BOXANGLE;
        activeBoxIds[activeBoxIdCount++] = BOXHORIZON;
    }

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        activeBoxIds[activeBoxIdCount++] = BOXBARO;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXAIRMODE;

    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        activeBoxIds[activeBoxIdCount++] = BOXMAG;
        activeBoxIds[activeBoxIdCount++] = BOXHEADFREE;
        activeBoxIds[activeBoxIdCount++] = BOXHEADADJ;
    }

    if (feature(FEATURE_SERVO_TILT))
        activeBoxIds[activeBoxIdCount++] = BOXCAMSTAB;

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        activeBoxIds[activeBoxIdCount++] = BOXGPSHOME;
        activeBoxIds[activeBoxIdCount++] = BOXGPSHOLD;
    }
#endif

    if (masterConfig.mixerMode == MIXER_FLYING_WING || masterConfig.mixerMode == MIXER_AIRPLANE || masterConfig.mixerMode == MIXER_CUSTOM_AIRPLANE)
        activeBoxIds[activeBoxIdCount++] = BOXPASSTHRU;

    activeBoxIds[activeBoxIdCount++] = BOXBEEPERON;

#ifdef LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        activeBoxIds[activeBoxIdCount++] = BOXLEDLOW;
    }
#endif

    if (feature(FEATURE_INFLIGHT_ACC_CAL))
        activeBoxIds[activeBoxIdCount++] = BOXCALIB;

    activeBoxIds[activeBoxIdCount++] = BOXOSD;

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY) && masterConfig.telemetryConfig.telemetry_switch)
        activeBoxIds[activeBoxIdCount++] = BOXTELEMETRY;
#endif

    if (feature(FEATURE_SONAR)){
        activeBoxIds[activeBoxIdCount++] = BOXSONAR;
    }

#ifdef USE_SERVOS
    if (masterConfig.mixerMode == MIXER_CUSTOM_AIRPLANE) {
        activeBoxIds[activeBoxIdCount++] = BOXSERVO1;
        activeBoxIds[activeBoxIdCount++] = BOXSERVO2;
        activeBoxIds[activeBoxIdCount++] = BOXSERVO3;
    }
#endif

#ifdef BLACKBOX
    if (feature(FEATURE_BLACKBOX)){
        activeBoxIds[activeBoxIdCount++] = BOXBLACKBOX;
    }
#endif

    if (feature(FEATURE_FAILSAFE)){
        activeBoxIds[activeBoxIdCount++] = BOXFAILSAFE;
    }

#ifdef GTUNE
    activeBoxIds[activeBoxIdCount++] = BOXGTUNE;
#endif

    memset(mspPorts, 0x00, sizeof(mspPorts));
    mspAllocateSerialPorts(serialConfig);
}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

static uint32_t packFlightModeFlags(void)
{
    uint32_t i, junk, tmp;

    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.
    junk = 0;
    tmp = IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << BOXANGLE |
        IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << BOXHORIZON |
        IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << BOXBARO |
        IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << BOXMAG |
        IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)) << BOXHEADFREE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)) << BOXHEADADJ |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)) << BOXCAMSTAB |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMTRIG)) << BOXCAMTRIG |
        IS_ENABLED(FLIGHT_MODE(GPS_HOME_MODE)) << BOXGPSHOME |
        IS_ENABLED(FLIGHT_MODE(GPS_HOLD_MODE)) << BOXGPSHOLD |
        IS_ENABLED(FLIGHT_MODE(PASSTHRU_MODE)) << BOXPASSTHRU |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)) << BOXBEEPERON |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDMAX)) << BOXLEDMAX |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)) << BOXLEDLOW |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLLIGHTS)) << BOXLLIGHTS |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCALIB)) << BOXCALIB |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGOV)) << BOXGOV |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)) << BOXOSD |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)) << BOXTELEMETRY |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGTUNE)) << BOXGTUNE |
        IS_ENABLED(FLIGHT_MODE(SONAR_MODE)) << BOXSONAR |
        IS_ENABLED(ARMING_FLAG(ARMED)) << BOXARM |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)) << BOXBLACKBOX |
        IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << BOXFAILSAFE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)) << BOXAIRMODE;

    for (i = 0; i < activeBoxIdCount; i++) {
        int flag = (tmp & (1 << activeBoxIds[i]));
        if (flag)
            junk |= 1 << i;
    }
    
    return junk;
}

static bool processOutCommand(PifMsp* p_owner, PifMspPacket* p_packet)
{
    uint32_t i;
    uint8_t scale;
#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0;
#endif

    switch (p_packet->command) {
    case MSP_API_VERSION:
        pifMsp_AddAnswer8(p_owner, MSP_PROTOCOL_VERSION);

        pifMsp_AddAnswer8(p_owner, API_VERSION_MAJOR);
        pifMsp_AddAnswer8(p_owner, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        for (i = 0; i < FLIGHT_CONTROLLER_IDENTIFIER_LENGTH; i++) {
            pifMsp_AddAnswer8(p_owner, flightControllerIdentifier[i]);
        }
        break;

    case MSP_FC_VERSION:
        pifMsp_AddAnswer8(p_owner, FC_VERSION_MAJOR);
        pifMsp_AddAnswer8(p_owner, FC_VERSION_MINOR);
        pifMsp_AddAnswer8(p_owner, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
        for (i = 0; i < BOARD_IDENTIFIER_LENGTH; i++) {
            pifMsp_AddAnswer8(p_owner, boardIdentifier[i]);
        }
#ifdef NAZE
        pifMsp_AddAnswer16(p_owner, hardwareRevision);
#else
        pifMsp_AddAnswer16(p_owner, 0); // No other build targets currently have hardware revision detection.
#endif
        break;

    case MSP_BUILD_INFO:
        for (i = 0; i < BUILD_DATE_LENGTH; i++) {
            pifMsp_AddAnswer8(p_owner, buildDate[i]);
        }
        for (i = 0; i < BUILD_TIME_LENGTH; i++) {
            pifMsp_AddAnswer8(p_owner, buildTime[i]);
        }

        for (i = 0; i < GIT_SHORT_REVISION_LENGTH; i++) {
            pifMsp_AddAnswer8(p_owner, shortGitRevision[i]);
        }
        break;

    // DEPRECATED - Use MSP_API_VERSION
    case MSP_IDENT:
        pifMsp_AddAnswer8(p_owner, MW_VERSION);
        pifMsp_AddAnswer8(p_owner, masterConfig.mixerMode);
        pifMsp_AddAnswer8(p_owner, MSP_PROTOCOL_VERSION);
        pifMsp_AddAnswer32(p_owner, CAP_DYNBALANCE); // "capability"
        break;

    case MSP_STATUS_EX:
        pifMsp_AddAnswer16(p_owner, cycleTime);
#ifdef USE_I2C
        pifMsp_AddAnswer16(p_owner, i2cGetErrorCounter());
#else
        pifMsp_AddAnswer16(p_owner, 0);
#endif
        pifMsp_AddAnswer16(p_owner, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        pifMsp_AddAnswer32(p_owner, packFlightModeFlags());
        pifMsp_AddAnswer8(p_owner, masterConfig.current_profile_index);
        pifMsp_AddAnswer16(p_owner, pif_performance._use_rate);
        break;

    case MSP_STATUS:
        pifMsp_AddAnswer16(p_owner, cycleTime);
#ifdef USE_I2C
        pifMsp_AddAnswer16(p_owner, i2cGetErrorCounter());
#else
        pifMsp_AddAnswer16(p_owner, 0);
#endif
        pifMsp_AddAnswer16(p_owner, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        pifMsp_AddAnswer32(p_owner, packFlightModeFlags());
        pifMsp_AddAnswer8(p_owner, masterConfig.current_profile_index);
        break;
    case MSP_RAW_IMU:
        // Hack scale due to choice of units for sensor data in multiwii
        scale = (sensor_link.acc.acc_1G > 1024) ? 8 : 1;

        for (i = 0; i < 3; i++)
            pifMsp_AddAnswer16(p_owner, accSmooth[i] / scale);
        for (i = 0; i < 3; i++)
            pifMsp_AddAnswer16(p_owner, gyroADC[i]);
        for (i = 0; i < 3; i++)
            pifMsp_AddAnswer16(p_owner, magADC[i]);
        break;
#ifdef USE_SERVOS
    case MSP_SERVO:
        pifMsp_AddAnswer(p_owner, (uint8_t *)&servo, MAX_SUPPORTED_SERVOS * 2);
        break;
    case MSP_SERVO_CONFIGURATIONS:
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            pifMsp_AddAnswer16(p_owner, currentProfile->servoConf[i].min);
            pifMsp_AddAnswer16(p_owner, currentProfile->servoConf[i].max);
            pifMsp_AddAnswer16(p_owner, currentProfile->servoConf[i].middle);
            pifMsp_AddAnswer8(p_owner, currentProfile->servoConf[i].rate);
            pifMsp_AddAnswer8(p_owner, currentProfile->servoConf[i].angleAtMin);
            pifMsp_AddAnswer8(p_owner, currentProfile->servoConf[i].angleAtMax);
            pifMsp_AddAnswer8(p_owner, currentProfile->servoConf[i].forwardFromChannel);
            pifMsp_AddAnswer32(p_owner, currentProfile->servoConf[i].reversedSources);
        }
        break;
    case MSP_SERVO_MIX_RULES:
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            pifMsp_AddAnswer8(p_owner, masterConfig.customServoMixer[i].targetChannel);
            pifMsp_AddAnswer8(p_owner, masterConfig.customServoMixer[i].inputSource);
            pifMsp_AddAnswer8(p_owner, masterConfig.customServoMixer[i].rate);
            pifMsp_AddAnswer8(p_owner, masterConfig.customServoMixer[i].speed);
            pifMsp_AddAnswer8(p_owner, masterConfig.customServoMixer[i].min);
            pifMsp_AddAnswer8(p_owner, masterConfig.customServoMixer[i].max);
            pifMsp_AddAnswer8(p_owner, masterConfig.customServoMixer[i].box);
        }
        break;
#endif
    case MSP_MOTOR:
        for (i = 0; i < 8; i++) {
            pifMsp_AddAnswer16(p_owner, i < MAX_SUPPORTED_MOTORS ? motor[i] : 0);
        }
        break;
    case MSP_RC:
        for (i = 0; i < rxRuntimeConfig.channelCount; i++)
            pifMsp_AddAnswer16(p_owner, rcData[i]);
        break;
    case MSP_ATTITUDE:
        pifMsp_AddAnswer16(p_owner, attitude.values.roll);
        pifMsp_AddAnswer16(p_owner, attitude.values.pitch);
        pifMsp_AddAnswer16(p_owner, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;
    case MSP_ALTITUDE:
#if defined(BARO) || defined(SONAR)
        pifMsp_AddAnswer32(p_owner, altitudeHoldGetEstimatedAltitude());
#else
        pifMsp_AddAnswer32(p_owner, 0);
#endif
        pifMsp_AddAnswer16(p_owner, vario);
        break;
    case MSP_SONAR_ALTITUDE:
#if defined(SONAR)
        pifMsp_AddAnswer32(p_owner, sonarGetLatestAltitude());
#else
        pifMsp_AddAnswer32(p_owner, 0);
#endif
        break;
    case MSP_ANALOG:
        pifMsp_AddAnswer8(p_owner, (uint8_t)constrain(vbat, 0, 255));
        pifMsp_AddAnswer16(p_owner, (uint16_t)constrain(mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
        pifMsp_AddAnswer16(p_owner, rssi);
        if(masterConfig.batteryConfig.multiwiiCurrentMeterOutput) {
            pifMsp_AddAnswer16(p_owner, (uint16_t)constrain(amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
        } else
            pifMsp_AddAnswer16(p_owner, (int16_t)constrain(amperage, -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
        break;
    case MSP_ARMING_CONFIG:
        pifMsp_AddAnswer8(p_owner, masterConfig.auto_disarm_delay); 
        pifMsp_AddAnswer8(p_owner, masterConfig.disarm_kill_switch);
        break;
    case MSP_LOOP_TIME:
        pifMsp_AddAnswer16(p_owner, masterConfig.looptime);
        break;
    case MSP_RC_TUNING:
        pifMsp_AddAnswer8(p_owner, currentControlRateProfile->rcRate8);
        pifMsp_AddAnswer8(p_owner, currentControlRateProfile->rcExpo8);
        for (i = 0 ; i < 3; i++) {
            pifMsp_AddAnswer8(p_owner, currentControlRateProfile->rates[i]); // R,P,Y see flight_dynamics_index_t
        }
        pifMsp_AddAnswer8(p_owner, currentControlRateProfile->dynThrPID);
        pifMsp_AddAnswer8(p_owner, currentControlRateProfile->thrMid8);
        pifMsp_AddAnswer8(p_owner, currentControlRateProfile->thrExpo8);
        pifMsp_AddAnswer16(p_owner, currentControlRateProfile->tpa_breakpoint);
        pifMsp_AddAnswer8(p_owner, currentControlRateProfile->rcYawExpo8);
        break;
    case MSP_PID:
        if (IS_PID_CONTROLLER_FP_BASED(currentProfile->pidProfile.pidController)) { // convert float stuff into uint8_t to keep backwards compatability with all 8-bit shit with new pid
            for (i = 0; i < 3; i++) {
                pifMsp_AddAnswer8(p_owner, constrain(lrintf(currentProfile->pidProfile.P_f[i] * 10.0f), 0, 255));
                pifMsp_AddAnswer8(p_owner, constrain(lrintf(currentProfile->pidProfile.I_f[i] * 100.0f), 0, 255));
                pifMsp_AddAnswer8(p_owner, constrain(lrintf(currentProfile->pidProfile.D_f[i] * 1000.0f), 0, 255));
            }
            for (i = 3; i < PID_ITEM_COUNT; i++) {
                if (i == PIDLEVEL) {
                    pifMsp_AddAnswer8(p_owner, constrain(lrintf(currentProfile->pidProfile.A_level * 10.0f), 0, 255));
                    pifMsp_AddAnswer8(p_owner, constrain(lrintf(currentProfile->pidProfile.H_level * 10.0f), 0, 255));
                    pifMsp_AddAnswer8(p_owner, constrain(lrintf(currentProfile->pidProfile.H_sensitivity), 0, 255));
                } else {
                    pifMsp_AddAnswer8(p_owner, currentProfile->pidProfile.P8[i]);
                    pifMsp_AddAnswer8(p_owner, currentProfile->pidProfile.I8[i]);
                    pifMsp_AddAnswer8(p_owner, currentProfile->pidProfile.D8[i]);
                }
            }
        } else {
            for (i = 0; i < PID_ITEM_COUNT; i++) {
                pifMsp_AddAnswer8(p_owner, currentProfile->pidProfile.P8[i]);
                pifMsp_AddAnswer8(p_owner, currentProfile->pidProfile.I8[i]);
                pifMsp_AddAnswer8(p_owner, currentProfile->pidProfile.D8[i]);
            }
        }
        break;
    case MSP_PIDNAMES:
        pifMsp_AddAnswer(p_owner, (uint8_t*)pidnames, strlen(pidnames));
        break;
    case MSP_PID_CONTROLLER:
        pifMsp_AddAnswer8(p_owner, currentProfile->pidProfile.pidController);
        break;
    case MSP_MODE_RANGES:
        for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            const box_t *box = &boxes[mac->modeId];
            pifMsp_AddAnswer8(p_owner, box->permanentId);
            pifMsp_AddAnswer8(p_owner, mac->auxChannelIndex);
            pifMsp_AddAnswer8(p_owner, mac->range.startStep);
            pifMsp_AddAnswer8(p_owner, mac->range.endStep);
        }
        break;
    case MSP_ADJUSTMENT_RANGES:
        for (i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            adjustmentRange_t *adjRange = &currentProfile->adjustmentRanges[i];
            pifMsp_AddAnswer8(p_owner, adjRange->adjustmentIndex);
            pifMsp_AddAnswer8(p_owner, adjRange->auxChannelIndex);
            pifMsp_AddAnswer8(p_owner, adjRange->range.startStep);
            pifMsp_AddAnswer8(p_owner, adjRange->range.endStep);
            pifMsp_AddAnswer8(p_owner, adjRange->adjustmentFunction);
            pifMsp_AddAnswer8(p_owner, adjRange->auxSwitchChannelIndex);
        }
        break;
    case MSP_BOXNAMES:
        serializeBoxNamesReply(p_owner);
        break;
    case MSP_BOXIDS:
        for (i = 0; i < activeBoxIdCount; i++) {
            const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
            if (!box) {
                continue;
            }
            pifMsp_AddAnswer8(p_owner, box->permanentId);
        }
        break;
    case MSP_MISC:
        pifMsp_AddAnswer16(p_owner, masterConfig.rxConfig.midrc);

        pifMsp_AddAnswer16(p_owner, masterConfig.escAndServoConfig.minthrottle);
        pifMsp_AddAnswer16(p_owner, masterConfig.escAndServoConfig.maxthrottle);
        pifMsp_AddAnswer16(p_owner, masterConfig.escAndServoConfig.mincommand);

        pifMsp_AddAnswer16(p_owner, masterConfig.failsafeConfig.failsafe_throttle);

#ifdef GPS
        pifMsp_AddAnswer8(p_owner, masterConfig.gpsConfig.provider); // gps_type
        pifMsp_AddAnswer8(p_owner, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        pifMsp_AddAnswer8(p_owner, masterConfig.gpsConfig.sbasMode); // gps_ubx_sbas
#else
        pifMsp_AddAnswer8(p_owner, 0); // gps_type
        pifMsp_AddAnswer8(p_owner, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        pifMsp_AddAnswer8(p_owner, 0); // gps_ubx_sbas
#endif
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.multiwiiCurrentMeterOutput);
        pifMsp_AddAnswer8(p_owner, masterConfig.rxConfig.rssi_channel);
        pifMsp_AddAnswer8(p_owner, 0);

        pifMsp_AddAnswer16(p_owner, currentProfile->mag_declination / 10);

        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatscale);
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatmincellvoltage);
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatmaxcellvoltage);
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatwarningcellvoltage);
        break;

    case MSP_MOTOR_PINS:
        // FIXME This is hardcoded and should not be.
        for (i = 0; i < 8; i++)
            pifMsp_AddAnswer8(p_owner, i + 1);
        break;
#ifdef GPS
    case MSP_RAW_GPS:
        pifMsp_AddAnswer8(p_owner, STATE(GPS_FIX));
        pifMsp_AddAnswer8(p_owner, GPS_numSat);
        pifMsp_AddAnswer32(p_owner, GPS_coord[LAT]);
        pifMsp_AddAnswer32(p_owner, GPS_coord[LON]);
        pifMsp_AddAnswer16(p_owner, GPS_altitude);
        pifMsp_AddAnswer16(p_owner, GPS_speed);
        pifMsp_AddAnswer16(p_owner, GPS_ground_course);
        break;
    case MSP_COMP_GPS:
        pifMsp_AddAnswer16(p_owner, GPS_distanceToHome);
        pifMsp_AddAnswer16(p_owner, GPS_directionToHome);
        pifMsp_AddAnswer8(p_owner, GPS_update & 1);
        break;
    case MSP_WP:
        wp_no = pifMsp_ReadData8(p_packet);    // get the wp number
        if (wp_no == 0) {
            lat = GPS_home[LAT];
            lon = GPS_home[LON];
        } else if (wp_no == 16) {
            lat = GPS_hold[LAT];
            lon = GPS_hold[LON];
        }
        pifMsp_AddAnswer8(p_owner, wp_no);
        pifMsp_AddAnswer32(p_owner, lat);
        pifMsp_AddAnswer32(p_owner, lon);
        pifMsp_AddAnswer32(p_owner, AltHold);           // altitude (cm) will come here -- temporary implementation to test feature with apps
        pifMsp_AddAnswer16(p_owner, 0);                 // heading  will come here (deg)
        pifMsp_AddAnswer16(p_owner, 0);                 // time to stay (ms) will come here
        pifMsp_AddAnswer8(p_owner, 0);                  // nav flag will come here
        break;
    case MSP_GPSSVINFO:
        pifMsp_AddAnswer8(p_owner, GPS_numCh);
           for (i = 0; i < GPS_numCh; i++){
               pifMsp_AddAnswer8(p_owner, GPS_svinfo_chn[i]);
               pifMsp_AddAnswer8(p_owner, GPS_svinfo_svid[i]);
               pifMsp_AddAnswer8(p_owner, GPS_svinfo_quality[i]);
               pifMsp_AddAnswer8(p_owner, GPS_svinfo_cno[i]);
           }
        break;
#endif
    case MSP_DEBUG:
        // output some useful QA statistics
        // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

        for (i = 0; i < DEBUG16_VALUE_COUNT; i++)
            pifMsp_AddAnswer16(p_owner, debug[i]);      // 4 variables are here for general monitoring purpose
        break;

    // Additional commands that are not compatible with MultiWii
    case MSP_ACC_TRIM:
        pifMsp_AddAnswer16(p_owner, currentProfile->accelerometerTrims.values.pitch);
        pifMsp_AddAnswer16(p_owner, currentProfile->accelerometerTrims.values.roll);
        break;

    case MSP_UID:
        pifMsp_AddAnswer32(p_owner, U_ID_0);
        pifMsp_AddAnswer32(p_owner, U_ID_1);
        pifMsp_AddAnswer32(p_owner, U_ID_2);
        break;

    case MSP_FEATURE:
        pifMsp_AddAnswer32(p_owner, featureMask());
        break;

    case MSP_BOARD_ALIGNMENT:
        pifMsp_AddAnswer16(p_owner, masterConfig.boardAlignment.rollDegrees);
        pifMsp_AddAnswer16(p_owner, masterConfig.boardAlignment.pitchDegrees);
        pifMsp_AddAnswer16(p_owner, masterConfig.boardAlignment.yawDegrees);
        break;

    case MSP_VOLTAGE_METER_CONFIG:
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatscale);
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatmincellvoltage);
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatmaxcellvoltage);
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.vbatwarningcellvoltage);
        break;

    case MSP_CURRENT_METER_CONFIG:
        pifMsp_AddAnswer16(p_owner, masterConfig.batteryConfig.currentMeterScale);
        pifMsp_AddAnswer16(p_owner, masterConfig.batteryConfig.currentMeterOffset);
        pifMsp_AddAnswer8(p_owner, masterConfig.batteryConfig.currentMeterType);
        pifMsp_AddAnswer16(p_owner, masterConfig.batteryConfig.batteryCapacity);
        break;

    case MSP_MIXER:
        pifMsp_AddAnswer8(p_owner, masterConfig.mixerMode);
        break;

    case MSP_RX_CONFIG:
        pifMsp_AddAnswer8(p_owner, masterConfig.rxConfig.serialrx_provider);
        pifMsp_AddAnswer16(p_owner, masterConfig.rxConfig.maxcheck);
        pifMsp_AddAnswer16(p_owner, masterConfig.rxConfig.midrc);
        pifMsp_AddAnswer16(p_owner, masterConfig.rxConfig.mincheck);
        pifMsp_AddAnswer8(p_owner, masterConfig.rxConfig.spektrum_sat_bind);
        pifMsp_AddAnswer16(p_owner, masterConfig.rxConfig.rx_min_usec);
        pifMsp_AddAnswer16(p_owner, masterConfig.rxConfig.rx_max_usec);
        break;

    case MSP_FAILSAFE_CONFIG:
        pifMsp_AddAnswer8(p_owner, masterConfig.failsafeConfig.failsafe_delay);
        pifMsp_AddAnswer8(p_owner, masterConfig.failsafeConfig.failsafe_off_delay);
        pifMsp_AddAnswer16(p_owner, masterConfig.failsafeConfig.failsafe_throttle);
        pifMsp_AddAnswer8(p_owner, masterConfig.failsafeConfig.failsafe_kill_switch);
        pifMsp_AddAnswer16(p_owner, masterConfig.failsafeConfig.failsafe_throttle_low_delay);
        pifMsp_AddAnswer8(p_owner, masterConfig.failsafeConfig.failsafe_procedure);
        break;

    case MSP_RXFAIL_CONFIG:
        for (i = 0; i < rxRuntimeConfig.channelCount; i++) {
            pifMsp_AddAnswer8(p_owner, masterConfig.rxConfig.failsafe_channel_configurations[i].mode);
            pifMsp_AddAnswer16(p_owner, RXFAIL_STEP_TO_CHANNEL_VALUE(masterConfig.rxConfig.failsafe_channel_configurations[i].step));
        }
        break;

    case MSP_RSSI_CONFIG:
        pifMsp_AddAnswer8(p_owner, masterConfig.rxConfig.rssi_channel);
        break;

    case MSP_RX_MAP:
        for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++)
            pifMsp_AddAnswer8(p_owner, masterConfig.rxConfig.rcmap[i]);
        break;

    case MSP_BF_CONFIG:
        pifMsp_AddAnswer8(p_owner, masterConfig.mixerMode);

        pifMsp_AddAnswer32(p_owner, featureMask());

        pifMsp_AddAnswer8(p_owner, masterConfig.rxConfig.serialrx_provider);

        pifMsp_AddAnswer16(p_owner, masterConfig.boardAlignment.rollDegrees);
        pifMsp_AddAnswer16(p_owner, masterConfig.boardAlignment.pitchDegrees);
        pifMsp_AddAnswer16(p_owner, masterConfig.boardAlignment.yawDegrees);

        pifMsp_AddAnswer16(p_owner, masterConfig.batteryConfig.currentMeterScale);
        pifMsp_AddAnswer16(p_owner, masterConfig.batteryConfig.currentMeterOffset);
        break;

    case MSP_CF_SERIAL_CONFIG:
        for (i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(masterConfig.serialConfig.portConfigs[i].identifier)) {
                continue;
            };
            pifMsp_AddAnswer8(p_owner, masterConfig.serialConfig.portConfigs[i].identifier);
            pifMsp_AddAnswer16(p_owner, masterConfig.serialConfig.portConfigs[i].functionMask);
            pifMsp_AddAnswer8(p_owner, masterConfig.serialConfig.portConfigs[i].msp_baudrateIndex);
            pifMsp_AddAnswer8(p_owner, masterConfig.serialConfig.portConfigs[i].gps_baudrateIndex);
            pifMsp_AddAnswer8(p_owner, masterConfig.serialConfig.portConfigs[i].telemetry_baudrateIndex);
            pifMsp_AddAnswer8(p_owner, masterConfig.serialConfig.portConfigs[i].blackbox_baudrateIndex);
        }
        break;

#ifdef LED_STRIP
    case MSP_LED_COLORS:
        for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &masterConfig.colors[i];
            pifMsp_AddAnswer16(p_owner, color->h);
            pifMsp_AddAnswer8(p_owner, color->s);
            pifMsp_AddAnswer8(p_owner, color->v);
        }
        break;

    case MSP_LED_STRIP_CONFIG:
        for (i = 0; i < MAX_LED_STRIP_LENGTH; i++) {
            ledConfig_t *ledConfig = &masterConfig.ledConfigs[i];
            pifMsp_AddAnswer16(p_owner, (ledConfig->flags & LED_DIRECTION_MASK) >> LED_DIRECTION_BIT_OFFSET);
            pifMsp_AddAnswer16(p_owner, (ledConfig->flags & LED_FUNCTION_MASK) >> LED_FUNCTION_BIT_OFFSET);
            pifMsp_AddAnswer8(p_owner, GET_LED_X(ledConfig));
            pifMsp_AddAnswer8(p_owner, GET_LED_Y(ledConfig));
            pifMsp_AddAnswer8(p_owner, ledConfig->color);
        }
        break;
#endif

    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(p_owner);
        break;

#ifdef USE_FLASHFS
    case MSP_DATAFLASH_READ:
        {
            uint32_t readAddress = pifMsp_ReadData32(p_packet);

            serializeDataflashReadReply(p_owner, readAddress, 128);
        }
        break;
#endif

    case MSP_BLACKBOX_CONFIG:
#ifdef BLACKBOX
        pifMsp_AddAnswer8(p_owner, 1); //Blackbox supported
        pifMsp_AddAnswer8(p_owner, masterConfig.blackbox_device);
        pifMsp_AddAnswer8(p_owner, masterConfig.blackbox_rate_denom);
#else
        pifMsp_AddAnswer8(p_owner, 0); // Blackbox not supported
        pifMsp_AddAnswer8(p_owner, 0);
        pifMsp_AddAnswer8(p_owner, 0);
        pifMsp_AddAnswer8(p_owner, 0);
#endif
        break;

    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(p_owner);
        break;

    case MSP_TRANSPONDER_CONFIG:
#ifdef TRANSPONDER
        pifMsp_AddAnswer8(p_owner, 1); //Transponder supported

        for (i = 0; i < sizeof(masterConfig.transponderData); i++) {
            pifMsp_AddAnswer8(p_owner, masterConfig.transponderData[i]);
        }
#else
        pifMsp_AddAnswer8(p_owner, 0); // Transponder not supported
#endif
        break;

    case MSP_BF_BUILD_INFO:
        for (i = 0; i < 11; i++)
            pifMsp_AddAnswer8(p_owner, buildDate[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
        pifMsp_AddAnswer32(p_owner, 0); // future exp
        pifMsp_AddAnswer32(p_owner, 0); // future exp
        break;

    case MSP_3D:
        pifMsp_AddAnswer16(p_owner, masterConfig.flight3DConfig.deadband3d_low);
        pifMsp_AddAnswer16(p_owner, masterConfig.flight3DConfig.deadband3d_high);
        pifMsp_AddAnswer16(p_owner, masterConfig.flight3DConfig.neutral3d);
        pifMsp_AddAnswer16(p_owner, masterConfig.flight3DConfig.deadband3d_throttle);
        break;

    case MSP_RC_DEADBAND:
        pifMsp_AddAnswer8(p_owner, currentProfile->rcControlsConfig.deadband);
        pifMsp_AddAnswer8(p_owner, currentProfile->rcControlsConfig.yaw_deadband);
        pifMsp_AddAnswer8(p_owner, currentProfile->rcControlsConfig.alt_hold_deadband);
        break;
    case MSP_SENSOR_ALIGNMENT:
        pifMsp_AddAnswer8(p_owner, masterConfig.sensorAlignmentConfig.gyro_align);
        pifMsp_AddAnswer8(p_owner, masterConfig.sensorAlignmentConfig.acc_align);
        pifMsp_AddAnswer8(p_owner, masterConfig.sensorAlignmentConfig.mag_align);
        break;

    default:
        return false;
    }
    return true;
}

static bool processInCommand(PifMspPacket* p_packet)
{
    uint32_t i;
    uint16_t tmp;
    uint8_t rate;
#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0, alt = 0;
#endif

    switch (p_packet->command) {
    case MSP_SELECT_SETTING:
        if (!ARMING_FLAG(ARMED)) {
            masterConfig.current_profile_index = pifMsp_ReadData8(p_packet);
            if (masterConfig.current_profile_index > 2) {
                masterConfig.current_profile_index = 0;
            }
            writeEEPROM();
            readEEPROM();
        }
        break;
    case MSP_SET_HEAD:
        magHold = pifMsp_ReadData16(p_packet);
        break;
    case MSP_SET_RAW_RC:
        {
            uint8_t channelCount = p_packet->data_count / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return false;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];

                for (i = 0; i < channelCount; i++) {
                    frame[i] = pifMsp_ReadData16(p_packet);
                }

                rxMspFrameReceive(frame, channelCount);
            }
        }
        break;
    case MSP_SET_ACC_TRIM:
        currentProfile->accelerometerTrims.values.pitch = pifMsp_ReadData16(p_packet);
        currentProfile->accelerometerTrims.values.roll  = pifMsp_ReadData16(p_packet);
        break;
    case MSP_SET_ARMING_CONFIG:
        masterConfig.auto_disarm_delay = pifMsp_ReadData8(p_packet);
        masterConfig.disarm_kill_switch = pifMsp_ReadData8(p_packet);
        break;
    case MSP_SET_LOOP_TIME:
        masterConfig.looptime = pifMsp_ReadData16(p_packet);
        break;
    case MSP_SET_PID_CONTROLLER:
        currentProfile->pidProfile.pidController = pifMsp_ReadData8(p_packet);
        pidSetController(currentProfile->pidProfile.pidController);
        break;
    case MSP_SET_PID:
        if (IS_PID_CONTROLLER_FP_BASED(currentProfile->pidProfile.pidController)) {
            for (i = 0; i < 3; i++) {
                currentProfile->pidProfile.P_f[i] = (float)pifMsp_ReadData8(p_packet) / 10.0f;
                currentProfile->pidProfile.I_f[i] = (float)pifMsp_ReadData8(p_packet) / 100.0f;
                currentProfile->pidProfile.D_f[i] = (float)pifMsp_ReadData8(p_packet) / 1000.0f;
            }
            for (i = 3; i < PID_ITEM_COUNT; i++) {
                if (i == PIDLEVEL) {
                    currentProfile->pidProfile.A_level = (float)pifMsp_ReadData8(p_packet) / 10.0f;
                    currentProfile->pidProfile.H_level = (float)pifMsp_ReadData8(p_packet) / 10.0f;
                    currentProfile->pidProfile.H_sensitivity = pifMsp_ReadData8(p_packet);
                } else {
                    currentProfile->pidProfile.P8[i] = pifMsp_ReadData8(p_packet);
                    currentProfile->pidProfile.I8[i] = pifMsp_ReadData8(p_packet);
                    currentProfile->pidProfile.D8[i] = pifMsp_ReadData8(p_packet);
                }
            }
        } else {
            for (i = 0; i < PID_ITEM_COUNT; i++) {
                currentProfile->pidProfile.P8[i] = pifMsp_ReadData8(p_packet);
                currentProfile->pidProfile.I8[i] = pifMsp_ReadData8(p_packet);
                currentProfile->pidProfile.D8[i] = pifMsp_ReadData8(p_packet);
            }
        }
        break;
    case MSP_SET_MODE_RANGE:
        i = pifMsp_ReadData8(p_packet);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            i = pifMsp_ReadData8(p_packet);
            const box_t *box = findBoxByPermenantId(i);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = pifMsp_ReadData8(p_packet);
                mac->range.startStep = pifMsp_ReadData8(p_packet);
                mac->range.endStep = pifMsp_ReadData8(p_packet);

                useRcControlsConfig(currentProfile->modeActivationConditions, &masterConfig.escAndServoConfig, &currentProfile->pidProfile);
            } else {
                return false;
            }
        } else {
            return false;
        }
        break;
    case MSP_SET_ADJUSTMENT_RANGE:
        i = pifMsp_ReadData8(p_packet);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *adjRange = &currentProfile->adjustmentRanges[i];
            i = pifMsp_ReadData8(p_packet);
            if (i < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                adjRange->adjustmentIndex = i;
                adjRange->auxChannelIndex = pifMsp_ReadData8(p_packet);
                adjRange->range.startStep = pifMsp_ReadData8(p_packet);
                adjRange->range.endStep = pifMsp_ReadData8(p_packet);
                adjRange->adjustmentFunction = pifMsp_ReadData8(p_packet);
                adjRange->auxSwitchChannelIndex = pifMsp_ReadData8(p_packet);
            } else {
                return false;
            }
        } else {
            return false;
        }
        break;

    case MSP_SET_RC_TUNING:
        if (p_packet->data_count >= 10) {
            currentControlRateProfile->rcRate8 = pifMsp_ReadData8(p_packet);
            currentControlRateProfile->rcExpo8 = pifMsp_ReadData8(p_packet);
            for (i = 0; i < 3; i++) {
                rate = pifMsp_ReadData8(p_packet);
                currentControlRateProfile->rates[i] = MIN(rate, i == FD_YAW ? CONTROL_RATE_CONFIG_YAW_RATE_MAX : CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            }
            rate = pifMsp_ReadData8(p_packet);
            currentControlRateProfile->dynThrPID = MIN(rate, CONTROL_RATE_CONFIG_TPA_MAX);
            currentControlRateProfile->thrMid8 = pifMsp_ReadData8(p_packet);
            currentControlRateProfile->thrExpo8 = pifMsp_ReadData8(p_packet);
            currentControlRateProfile->tpa_breakpoint = pifMsp_ReadData16(p_packet);
            if (p_packet->data_count >= 11) {
                currentControlRateProfile->rcYawExpo8 = pifMsp_ReadData8(p_packet);
            }
        } else {
            return false;
        }
        break;
    case MSP_SET_MISC:
        tmp = pifMsp_ReadData16(p_packet);
        if (tmp < 1600 && tmp > 1400)
            masterConfig.rxConfig.midrc = tmp;

        masterConfig.escAndServoConfig.minthrottle = pifMsp_ReadData16(p_packet);
        masterConfig.escAndServoConfig.maxthrottle = pifMsp_ReadData16(p_packet);
        masterConfig.escAndServoConfig.mincommand = pifMsp_ReadData16(p_packet);

        masterConfig.failsafeConfig.failsafe_throttle = pifMsp_ReadData16(p_packet);

#ifdef GPS
        masterConfig.gpsConfig.provider = pifMsp_ReadData8(p_packet); // gps_type
        pifMsp_ReadData8(p_packet); // gps_baudrate
        masterConfig.gpsConfig.sbasMode = pifMsp_ReadData8(p_packet); // gps_ubx_sbas
#else
        pifMsp_ReadData8(p_packet); // gps_type
        pifMsp_ReadData8(p_packet); // gps_baudrate
        pifMsp_ReadData8(p_packet); // gps_ubx_sbas
#endif
        masterConfig.batteryConfig.multiwiiCurrentMeterOutput = pifMsp_ReadData8(p_packet);
        masterConfig.rxConfig.rssi_channel = pifMsp_ReadData8(p_packet);
        pifMsp_ReadData8(p_packet);

        currentProfile->mag_declination = pifMsp_ReadData16(p_packet) * 10;

        masterConfig.batteryConfig.vbatscale = pifMsp_ReadData8(p_packet);           // actual vbatscale as intended
        masterConfig.batteryConfig.vbatmincellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel_warn1 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatmaxcellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel_warn2 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatwarningcellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel when buzzer starts to alert
        break;
    case MSP_SET_MOTOR:
        for (i = 0; i < 8; i++) {
            const int16_t disarmed = pifMsp_ReadData16(p_packet);
            if (i < MAX_SUPPORTED_MOTORS) {
                motor_disarmed[i] = disarmed;
            }
        }
        break;
    case MSP_SET_SERVO_CONFIGURATION:
#ifdef USE_SERVOS
        if (p_packet->data_count != 1 + sizeof(servoParam_t)) {
            return false;
        }
        i = pifMsp_ReadData8(p_packet);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return false;
        } else {
            currentProfile->servoConf[i].min = pifMsp_ReadData16(p_packet);
            currentProfile->servoConf[i].max = pifMsp_ReadData16(p_packet);
            currentProfile->servoConf[i].middle = pifMsp_ReadData16(p_packet);
            currentProfile->servoConf[i].rate = pifMsp_ReadData8(p_packet);
            currentProfile->servoConf[i].angleAtMin = pifMsp_ReadData8(p_packet);
            currentProfile->servoConf[i].angleAtMax = pifMsp_ReadData8(p_packet);
            currentProfile->servoConf[i].forwardFromChannel = pifMsp_ReadData8(p_packet);
            currentProfile->servoConf[i].reversedSources = pifMsp_ReadData32(p_packet);
        }
#endif
        break;
        
    case MSP_SET_SERVO_MIX_RULE:
#ifdef USE_SERVOS
        i = pifMsp_ReadData8(p_packet);
        if (i >= MAX_SERVO_RULES) {
            return false;
        } else {
            masterConfig.customServoMixer[i].targetChannel = pifMsp_ReadData8(p_packet);
            masterConfig.customServoMixer[i].inputSource = pifMsp_ReadData8(p_packet);
            masterConfig.customServoMixer[i].rate = pifMsp_ReadData8(p_packet);
            masterConfig.customServoMixer[i].speed = pifMsp_ReadData8(p_packet);
            masterConfig.customServoMixer[i].min = pifMsp_ReadData8(p_packet);
            masterConfig.customServoMixer[i].max = pifMsp_ReadData8(p_packet);
            masterConfig.customServoMixer[i].box = pifMsp_ReadData8(p_packet);
            loadCustomServoMixer();
        }
#endif
        break;

    case MSP_SET_3D:
        masterConfig.flight3DConfig.deadband3d_low = pifMsp_ReadData16(p_packet);
        masterConfig.flight3DConfig.deadband3d_high = pifMsp_ReadData16(p_packet);
        masterConfig.flight3DConfig.neutral3d = pifMsp_ReadData16(p_packet);
        masterConfig.flight3DConfig.deadband3d_throttle = pifMsp_ReadData16(p_packet);
        break;

    case MSP_SET_RC_DEADBAND:
        currentProfile->rcControlsConfig.deadband = pifMsp_ReadData8(p_packet);
        currentProfile->rcControlsConfig.yaw_deadband = pifMsp_ReadData8(p_packet);
        currentProfile->rcControlsConfig.alt_hold_deadband = pifMsp_ReadData8(p_packet);
        break;

    case MSP_SET_RESET_CURR_PID:
        resetPidProfile(&currentProfile->pidProfile);
        break;    

    case MSP_SET_SENSOR_ALIGNMENT:
        masterConfig.sensorAlignmentConfig.gyro_align = pifMsp_ReadData8(p_packet);
        masterConfig.sensorAlignmentConfig.acc_align = pifMsp_ReadData8(p_packet);
        masterConfig.sensorAlignmentConfig.mag_align = pifMsp_ReadData8(p_packet);
        break;
        
    case MSP_RESET_CONF:
        if (!ARMING_FLAG(ARMED)) {
            resetEEPROM();
            readEEPROM();
        }
        break;
    case MSP_ACC_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
        break;
    case MSP_MAG_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            ENABLE_STATE(CALIBRATE_MAG);
        break;
    case MSP_EEPROM_WRITE:
        if (ARMING_FLAG(ARMED)) {
            return false;
        }
        writeEEPROM();
        readEEPROM();
        break;

#ifdef BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (blackboxMayEditConfig()) {
            masterConfig.blackbox_device = pifMsp_ReadData8(p_packet);
            masterConfig.blackbox_rate_num = pifMsp_ReadData8(p_packet);
            masterConfig.blackbox_rate_denom = pifMsp_ReadData8(p_packet);
        }
        break;
#endif

#ifdef TRANSPONDER
    case MSP_SET_TRANSPONDER_CONFIG:
        if (currentPort->dataSize != sizeof(masterConfig.transponderData)) {
            return false;
        }

        for (i = 0; i < sizeof(masterConfig.transponderData); i++) {
            masterConfig.transponderData[i] = pifMsp_ReadData8(p_packet);
        }

        transponderUpdateData(masterConfig.transponderData);
        break;
#endif

#ifdef USE_FLASHFS
    case MSP_DATAFLASH_ERASE:
        flashfsEraseCompletely();
        break;
#endif

#ifdef GPS
    case MSP_SET_RAW_GPS:
        if (pifMsp_ReadData8(p_packet)) {
            ENABLE_STATE(GPS_FIX);
        } else {
            DISABLE_STATE(GPS_FIX);
        }
        GPS_numSat = pifMsp_ReadData8(p_packet);
        GPS_coord[LAT] = pifMsp_ReadData32(p_packet);
        GPS_coord[LON] = pifMsp_ReadData32(p_packet);
        GPS_altitude = pifMsp_ReadData16(p_packet);
        GPS_speed = pifMsp_ReadData16(p_packet);
        GPS_update |= 2;        // New data signalisation to GPS functions // FIXME Magic Numbers
        break;
    case MSP_SET_WP:
        wp_no = pifMsp_ReadData8(p_packet);    //get the wp number
        lat = pifMsp_ReadData32(p_packet);
        lon = pifMsp_ReadData32(p_packet);
        alt = pifMsp_ReadData32(p_packet);     // to set altitude (cm)
        pifMsp_ReadData16(p_packet);           // future: to set heading (deg)
        pifMsp_ReadData16(p_packet);           // future: to set time to stay (ms)
        pifMsp_ReadData8(p_packet);            // future: to set nav flag
        if (wp_no == 0) {
            GPS_home[LAT] = lat;
            GPS_home[LON] = lon;
            DISABLE_FLIGHT_MODE(GPS_HOME_MODE);        // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
            ENABLE_STATE(GPS_FIX_HOME);
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
        } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
            GPS_hold[LAT] = lat;
            GPS_hold[LON] = lon;
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
            nav_mode = NAV_MODE_WP;
            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
        }
        break;
#endif
    case MSP_SET_FEATURE:
        featureClearAll();
        featureSet(pifMsp_ReadData32(p_packet)); // features bitmap
        break;

    case MSP_SET_BOARD_ALIGNMENT:
        masterConfig.boardAlignment.rollDegrees = pifMsp_ReadData16(p_packet);
        masterConfig.boardAlignment.pitchDegrees = pifMsp_ReadData16(p_packet);
        masterConfig.boardAlignment.yawDegrees = pifMsp_ReadData16(p_packet);
        break;

    case MSP_SET_VOLTAGE_METER_CONFIG:
        masterConfig.batteryConfig.vbatscale = pifMsp_ReadData8(p_packet);           // actual vbatscale as intended
        masterConfig.batteryConfig.vbatmincellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel_warn1 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatmaxcellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel_warn2 in MWC2.3 GUI
        masterConfig.batteryConfig.vbatwarningcellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel when buzzer starts to alert
        break;

    case MSP_SET_CURRENT_METER_CONFIG:
        masterConfig.batteryConfig.currentMeterScale = pifMsp_ReadData16(p_packet);
        masterConfig.batteryConfig.currentMeterOffset = pifMsp_ReadData16(p_packet);
        masterConfig.batteryConfig.currentMeterType = pifMsp_ReadData8(p_packet);
        masterConfig.batteryConfig.batteryCapacity = pifMsp_ReadData16(p_packet);
        break;

#ifndef USE_QUAD_MIXER_ONLY
    case MSP_SET_MIXER:
        masterConfig.mixerMode = pifMsp_ReadData8(p_packet);
        break;
#endif

    case MSP_SET_RX_CONFIG:
        masterConfig.rxConfig.serialrx_provider = pifMsp_ReadData8(p_packet);
        masterConfig.rxConfig.maxcheck = pifMsp_ReadData16(p_packet);
        masterConfig.rxConfig.midrc = pifMsp_ReadData16(p_packet);
        masterConfig.rxConfig.mincheck = pifMsp_ReadData16(p_packet);
        masterConfig.rxConfig.spektrum_sat_bind = pifMsp_ReadData8(p_packet);
        if (p_packet->data_count > 8) {
            masterConfig.rxConfig.rx_min_usec = pifMsp_ReadData16(p_packet);
            masterConfig.rxConfig.rx_max_usec = pifMsp_ReadData16(p_packet);
        }
        break;

    case MSP_SET_FAILSAFE_CONFIG:
        masterConfig.failsafeConfig.failsafe_delay = pifMsp_ReadData8(p_packet);
        masterConfig.failsafeConfig.failsafe_off_delay = pifMsp_ReadData8(p_packet);
        masterConfig.failsafeConfig.failsafe_throttle = pifMsp_ReadData16(p_packet);
        masterConfig.failsafeConfig.failsafe_kill_switch = pifMsp_ReadData8(p_packet);
        masterConfig.failsafeConfig.failsafe_throttle_low_delay = pifMsp_ReadData16(p_packet);
        masterConfig.failsafeConfig.failsafe_procedure = pifMsp_ReadData8(p_packet);
        break;

    case MSP_SET_RXFAIL_CONFIG:
        i = pifMsp_ReadData8(p_packet);
        if (i < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            masterConfig.rxConfig.failsafe_channel_configurations[i].mode = pifMsp_ReadData8(p_packet);
            masterConfig.rxConfig.failsafe_channel_configurations[i].step = CHANNEL_VALUE_TO_RXFAIL_STEP(pifMsp_ReadData16(p_packet));
        } else {
            return false;
        }
        break;

    case MSP_SET_RSSI_CONFIG:
        masterConfig.rxConfig.rssi_channel = pifMsp_ReadData8(p_packet);
        break;

    case MSP_SET_RX_MAP:
        for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
            masterConfig.rxConfig.rcmap[i] = pifMsp_ReadData8(p_packet);
        }
        break;

    case MSP_SET_BF_CONFIG:

#ifdef USE_QUAD_MIXER_ONLY
        pifMsp_ReadData8(p_packet); // mixerMode ignored
#else
        masterConfig.mixerMode = pifMsp_ReadData8(p_packet); // mixerMode
#endif

        featureClearAll();
        featureSet(pifMsp_ReadData32(p_packet)); // features bitmap

        masterConfig.rxConfig.serialrx_provider = pifMsp_ReadData8(p_packet); // serialrx_type

        masterConfig.boardAlignment.rollDegrees = pifMsp_ReadData16(p_packet); // board_align_roll
        masterConfig.boardAlignment.pitchDegrees = pifMsp_ReadData16(p_packet); // board_align_pitch
        masterConfig.boardAlignment.yawDegrees = pifMsp_ReadData16(p_packet); // board_align_yaw

        masterConfig.batteryConfig.currentMeterScale = pifMsp_ReadData16(p_packet);
        masterConfig.batteryConfig.currentMeterOffset = pifMsp_ReadData16(p_packet);
        break;

    case MSP_SET_CF_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (p_packet->data_count % portConfigSize != 0) {
                return false;
            }

            uint8_t remainingPortsInPacket = p_packet->data_count / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = pifMsp_ReadData8(p_packet);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig) {
                    return false;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = pifMsp_ReadData16(p_packet);
                portConfig->msp_baudrateIndex = pifMsp_ReadData8(p_packet);
                portConfig->gps_baudrateIndex = pifMsp_ReadData8(p_packet);
                portConfig->telemetry_baudrateIndex = pifMsp_ReadData8(p_packet);
                portConfig->blackbox_baudrateIndex = pifMsp_ReadData8(p_packet);
            }
        }
        break;

#ifdef LED_STRIP
    case MSP_SET_LED_COLORS:
        for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &masterConfig.colors[i];
            color->h = pifMsp_ReadData16(p_packet);
            color->s = pifMsp_ReadData8(p_packet);
            color->v = pifMsp_ReadData8(p_packet);
        }
        break;

    case MSP_SET_LED_STRIP_CONFIG:
        {
            i = pifMsp_ReadData8(p_packet);
            if (i >= MAX_LED_STRIP_LENGTH || currentPort->dataSize != (1 + 7)) {
                return false;
            }
            ledConfig_t *ledConfig = &masterConfig.ledConfigs[i];
            uint16_t mask;
            // currently we're storing directions and functions in a uint16 (flags)
            // the msp uses 2 x uint16_t to cater for future expansion
            mask = pifMsp_ReadData16(p_packet);
            ledConfig->flags = (mask << LED_DIRECTION_BIT_OFFSET) & LED_DIRECTION_MASK;

            mask = pifMsp_ReadData16(p_packet);
            ledConfig->flags |= (mask << LED_FUNCTION_BIT_OFFSET) & LED_FUNCTION_MASK;

            mask = pifMsp_ReadData8(p_packet);
            ledConfig->xy = CALCULATE_LED_X(mask);

            mask = pifMsp_ReadData8(p_packet);
            ledConfig->xy |= CALCULATE_LED_Y(mask);

            ledConfig->color = pifMsp_ReadData8(p_packet);

            reevalulateLedConfig();
        }
        break;
#endif
    case MSP_REBOOT:
        isRebootScheduled = true;
        break;

    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return false;
    }
    return true;
}

static void evtMspReceive(PifMsp* p_owner, PifMspPacket* p_packet, PifIssuerP p_issuer)
{
    pifMsp_MakeAnswer(p_owner, p_packet);
    if (!(processOutCommand(p_owner, p_packet) || processInCommand(p_packet))) {
        pifMsp_MakeError(p_owner, p_packet);
    }
    pifMsp_SendAnswer(p_owner);

    if (isRebootScheduled) {
        waitForSerialPortToFinishTransmitting((serialPort_t*)p_issuer);
        stopMotors();
        handleOneshotFeatureChangeOnRestart();
        systemReset();
    }
}
