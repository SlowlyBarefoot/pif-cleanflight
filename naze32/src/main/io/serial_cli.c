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
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include <platform.h>
#include "scheduler.h"
#include "version.h"

#include "build_config.h"

#include "core/pif_i2c.h"
#include "core/pif_log.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/color.h"
#include "common/typeconversion.h"

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "drivers/buf_writer.h"

#include "io/escservo.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/rc_controls.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/beeper.h"
#include "io/asyncfatfs/asyncfatfs.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "common/printf.h"

#include "mw.h"
#include "serial_cli.h"

#define CLI_ERROR_PARSE			    PIF_LOG_CMD_USER_ERROR
#define CLI_ERROR_OUT_OF_RANGE      (PIF_LOG_CMD_USER_ERROR - 1)
#define CLI_ERROR_INVALID		    (PIF_LOG_CMD_USER_ERROR - 2)

// FIXME remove this for targets that don't need a CLI.  Perhaps use a no-op macro when USE_CLI is not enabled
// signal that we're in cli mode
uint8_t cliMode = 0;

#ifdef USE_CLI

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort);

static serialPort_t *cliPort;

static int cliAux(int argc, char *argv[]);
static int cliRxFail(int argc, char *argv[]);
static int cliAdjustmentRange(int argc, char *argv[]);
static int cliMotorMix(int argc, char *argv[]);
static int cliDefaults(int argc, char *argv[]);
static int cliDump(int argc, char *argv[]);
static int cliExit(int argc, char *argv[]);
static int cliFeature(int argc, char *argv[]);
static int cliMotor(int argc, char *argv[]);
static int cliPlaySound(int argc, char *argv[]);
static int cliProfile(int argc, char *argv[]);
static int cliRateProfile(int argc, char *argv[]);
static void cliReboot(void);
static int cliSave(int argc, char *argv[]);
static int cliSerial(int argc, char *argv[]);

#ifdef USE_SERVOS
static int cliServo(int argc, char *argv[]);
static int cliServoMix(int argc, char *argv[]);
#endif

static int cliSet(int argc, char *argv[]);
static int cliGet(int argc, char *argv[]);
static int cliStatus(int argc, char *argv[]);
#ifndef SKIP_TASK_STATISTICS
static int cliTasks(int argc, char *argv[]);
#endif
static int cliVersion(int argc, char *argv[]);
static int cliRxRange(int argc, char *argv[]);

#ifdef GPS
static int cliGpsPassthrough(int argc, char *argv[]);
#endif

static int cliMap(int argc, char *argv[]);

#ifdef LED_STRIP
static int cliLed(int argc, char *argv[]);
static int cliColor(int argc, char *argv[]);
#endif

#ifndef USE_QUAD_MIXER_ONLY
static int cliMixer(int argc, char *argv[]);
#endif

#ifdef USE_FLASHFS
static int cliFlashInfo(int argc, char *argv[]);
static int cliFlashErase(int argc, char *argv[]);
#ifdef USE_FLASH_TOOLS
static int cliFlashWrite(int argc, char *argv[]);
static int cliFlashRead(int argc, char *argv[]);
#endif
#endif

#ifdef USE_SDCARD
static int cliSdInfo(int argc, char *argv[]);
#endif

#ifndef USE_QUAD_MIXER_ONLY
//  this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", NULL
};
#endif

// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "ONESHOT125",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", NULL
};

// sync this with rxFailsafeChannelMode_e
static const char rxFailsafeModeCharacters[] = "ahs";

static const rxFailsafeChannelMode_e rxFailsafeModesTable[RX_FAILSAFE_TYPE_COUNT][RX_FAILSAFE_MODE_COUNT] = {
    { RX_FAILSAFE_MODE_AUTO, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_INVALID },
    { RX_FAILSAFE_MODE_INVALID, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET }
};

// sync this with sensors_e
static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "SONAR", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

typedef struct {
    const char *name;
#ifndef SKIP_CLI_COMMAND_HELP
    const char *description;
    const char *args;
#endif
    void (*func)(int argc, char *argv[]);
} clicmd_t;

#ifndef SKIP_CLI_COMMAND_HELP
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name , \
    method , \
    description , \
    args \
}
#else
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name, \
    method \
}
#endif

// should be sorted a..z for bsearch()
const PifLogCmdEntry cmdTable[] = {
	{ "help", pifLog_CmdHelp, "This command", NULL },
	{ "pversion", pifLog_CmdPrintVersion, "Print version", NULL },
	{ "task", pifLog_CmdPrintTask, "Print task", NULL },
	{ "pstatus", pifLog_CmdSetStatus, "Set and print status", NULL },
    CLI_COMMAND_DEF("adjrange", "configure adjustment ranges", NULL, cliAdjustmentRange),
    CLI_COMMAND_DEF("aux", "configure modes", NULL, cliAux),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
#endif
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cliDefaults),
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|profile]", cliDump),
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("feature", "configure features",
        "list\r\n"
        "\t<+|->[name]", cliFeature),
#ifdef USE_FLASHFS
    CLI_COMMAND_DEF("flash_erase", "erase flash chip", NULL, cliFlashErase),
    CLI_COMMAND_DEF("flash_info", "show flash chip info", NULL, cliFlashInfo),
#ifdef USE_FLASH_TOOLS
    CLI_COMMAND_DEF("flash_read", NULL, "<length> <address>", cliFlashRead),
    CLI_COMMAND_DEF("flash_write", NULL, "<address> <message>", cliFlashWrite),
#endif
#endif
    CLI_COMMAND_DEF("get", "get variable value",
            "[name]", cliGet),
#ifdef GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
#ifdef LED_STRIP
    CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order",
        "[<map>]", cliMap),
#ifndef USE_QUAD_MIXER_ONLY
    CLI_COMMAND_DEF("mixer", "configure mixer",
        "list\r\n"
        "\t<name>", cliMixer),
#endif
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
    CLI_COMMAND_DEF("motor",  "get/set motor",
       "<index> [<value>]", cliMotor),
    CLI_COMMAND_DEF("play_sound", NULL,
        "[<index>]\r\n", cliPlaySound),
    CLI_COMMAND_DEF("profile", "change profile",
        "[<index>]", cliProfile),
    CLI_COMMAND_DEF("rateprofile", "change rate profile",
        "[<index>]", cliRateProfile),
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
    CLI_COMMAND_DEF("rxfail", "show/set rx failsafe settings", NULL, cliRxFail),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#endif
    CLI_COMMAND_DEF("set", "change setting",
        "[<name>=<value>]", cliSet),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("smix", "servo mixer",
        "<rule> <servo> <source> <rate> <speed> <min> <max> <box>\r\n"
        "\treset\r\n"
        "\tload <mixer>\r\n"
        "\treverse <servo> <source> r|n", cliServoMix),
#endif
#ifdef USE_SDCARD
    CLI_COMMAND_DEF("sd_info", "sdcard info", NULL, cliSdInfo),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
#ifndef SKIP_TASK_STATISTICS
    CLI_COMMAND_DEF("tasks", "show task stats", NULL, cliTasks),
#endif
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),

    CLI_COMMAND_DEF(NULL, NULL, NULL, NULL)
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(clicmd_t))

static const char * const lookupTableOffOn[] = {
    "OFF", "ON"
};

static const char * const lookupTableUnit[] = {
    "IMPERIAL", "METRIC"
};

static const char * const lookupTableAlignment[] = {
    "DEFAULT",
    "CW0",
    "CW90",
    "CW180",
    "CW270",
    "CW0FLIP",
    "CW90FLIP",
    "CW180FLIP",
    "CW270FLIP"
};

#ifdef GPS
static const char * const lookupTableGPSProvider[] = {
    "NMEA", "UBLOX"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN"
};
#endif

static const char * const lookupTableCurrentSensor[] = {
    "NONE", "ADC", "VIRTUAL"
};

static const char * const lookupTableGimbalMode[] = {
    "NORMAL", "MIXTILT"
};

static const char * const lookupTablePidController[] = {
    "MW23", "MWREWRITE", "LUX"
};

static const char * const lookupTableBlackboxDevice[] = {
    "SERIAL", "SPIFLASH", "SDCARD"
};

static const char * const lookupTableSerialRX[] = {
    "SPEK1024",
    "SPEK2048",
    "SBUS",
    "SUMD",
    "SUMH",
    "XB-B",
    "XB-B-RJ01",
    "IBUS"
};

static const char * const lookupTableGyroFilter[] = {
    "OFF", "LOW", "MEDIUM", "HIGH"
};

static const char * const lookupTableGyroLpf[] = {
    "OFF",
    "188HZ",
    "98HZ",
    "42HZ",
    "20HZ",
    "10HZ"
};

static const char * const lookupDeltaMethod[] = {
    "ERROR", "MEASUREMENT"
};

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;

typedef enum {
    TABLE_OFF_ON = 0,
    TABLE_UNIT,
    TABLE_ALIGNMENT,
#ifdef GPS
    TABLE_GPS_PROVIDER,
    TABLE_GPS_SBAS_MODE,
#endif
#ifdef BLACKBOX
    TABLE_BLACKBOX_DEVICE,
#endif
    TABLE_CURRENT_SENSOR,
    TABLE_GIMBAL_MODE,
    TABLE_PID_CONTROLLER,
    TABLE_SERIAL_RX,
    TABLE_GYRO_FILTER,
    TABLE_GYRO_LPF,
    TABLE_DELTA_METHOD,
} lookupTableIndex_e;

static const lookupTableEntry_t lookupTables[] = {
    { lookupTableOffOn, sizeof(lookupTableOffOn) / sizeof(char *) },
    { lookupTableUnit, sizeof(lookupTableUnit) / sizeof(char *) },
    { lookupTableAlignment, sizeof(lookupTableAlignment) / sizeof(char *) },
#ifdef GPS
    { lookupTableGPSProvider, sizeof(lookupTableGPSProvider) / sizeof(char *) },
    { lookupTableGPSSBASMode, sizeof(lookupTableGPSSBASMode) / sizeof(char *) },
#endif
#ifdef BLACKBOX
    { lookupTableBlackboxDevice, sizeof(lookupTableBlackboxDevice) / sizeof(char *) },
#endif
    { lookupTableCurrentSensor, sizeof(lookupTableCurrentSensor) / sizeof(char *) },
    { lookupTableGimbalMode, sizeof(lookupTableGimbalMode) / sizeof(char *) },
    { lookupTablePidController, sizeof(lookupTablePidController) / sizeof(char *) },
    { lookupTableSerialRX, sizeof(lookupTableSerialRX) / sizeof(char *) },
    { lookupTableGyroFilter, sizeof(lookupTableGyroFilter) / sizeof(char *) },
    { lookupTableGyroLpf, sizeof(lookupTableGyroLpf) / sizeof(char *) },
    { lookupDeltaMethod, sizeof(lookupDeltaMethod) / sizeof(char *) }
};

#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 4
#define VALUE_MODE_OFFSET 6

typedef enum {
    // value type
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),
    VAR_UINT32 = (4 << VALUE_TYPE_OFFSET),
    VAR_FLOAT = (5 << VALUE_TYPE_OFFSET),

    // value section
    MASTER_VALUE = (0 << VALUE_SECTION_OFFSET),
    PROFILE_VALUE = (1 << VALUE_SECTION_OFFSET),
    CONTROL_RATE_VALUE = (2 << VALUE_SECTION_OFFSET),

    // value mode
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET)
} cliValueFlag_e;

#define VALUE_TYPE_MASK (0x0F)
#define VALUE_SECTION_MASK (0x30)
#define VALUE_MODE_MASK (0xC0)

typedef struct cliMinMaxConfig_s {
    const int32_t min;
    const int32_t max;
} cliMinMaxConfig_t;

typedef struct cliLookupTableConfig_s {
    const lookupTableIndex_e tableIndex;
} cliLookupTableConfig_t;

typedef union {
    cliLookupTableConfig_t lookup;
    cliMinMaxConfig_t minmax;

} cliValueConfig_t;

typedef struct {
    const char *name;
    const uint8_t type; // see cliValueFlag_e
    void *ptr;
    const cliValueConfig_t config;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "looptime",                   VAR_UINT16 | MASTER_VALUE,  &masterConfig.looptime, .config.minmax = {0, 9000} },
    { "emf_avoidance",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.emf_avoidance, .config.lookup = { TABLE_OFF_ON } },
    { "i2c_highspeed",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.i2c_highspeed, .config.lookup = { TABLE_OFF_ON } },
    { "gyro_sync",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.gyroSync, .config.lookup = { TABLE_OFF_ON } },
    { "gyro_sync_denom",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.gyroSyncDenominator, .config.minmax = { 1,  32 } },

    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.midrc, .config.minmax = { 1200,  1700 } },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.mincheck, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.maxcheck, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE,  &masterConfig.rxConfig.rssi_channel, .config.minmax = { 0,  MAX_SUPPORTED_RC_CHANNEL_COUNT } },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.rssi_scale, .config.minmax = { RSSI_SCALE_MIN,  RSSI_SCALE_MAX } },
    { "rssi_ppm_invert",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rxConfig.rssi_ppm_invert, .config.lookup = { TABLE_OFF_ON } },
    { "rc_smoothing",               VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rxConfig.rcSmoothing, .config.lookup = { TABLE_OFF_ON } },
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.inputFilteringMode, .config.lookup = { TABLE_OFF_ON } },

    { "min_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.minthrottle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.maxthrottle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "min_command",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.mincommand, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.servoCenterPulse, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },

    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_low, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } }, // FIXME upper limit should match code in the mixer, 1500 currently
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_high, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } }, // FIXME lower limit should match code in the mixer, 1500 currently,
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.neutral3d, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "3d_deadband_throttle",       VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_throttle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },

    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.motor_pwm_rate, .config.minmax = { 50,  32000 } },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.servo_pwm_rate, .config.minmax = { 50,  498 } },

    { "retarded_arm",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.retarded_arm, .config.lookup = { TABLE_OFF_ON } },
    { "disarm_kill_switch",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.disarm_kill_switch, .config.lookup = { TABLE_OFF_ON } },
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.auto_disarm_delay, .config.minmax = { 0,  60 } },
    { "small_angle",                VAR_UINT8  | MASTER_VALUE,  &masterConfig.small_angle, .config.minmax = { 0,  180 } },

    { "fixedwing_althold_dir",      VAR_INT8   | MASTER_VALUE,  &masterConfig.airplaneConfig.fixedwing_althold_dir, .config.minmax = { -1,  1 } },

    { "reboot_character",           VAR_UINT8  | MASTER_VALUE,  &masterConfig.serialConfig.reboot_character, .config.minmax = { 48,  126 } },

#ifdef GPS
    { "gps_provider",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.gpsConfig.provider, .config.lookup = { TABLE_GPS_PROVIDER } },
    { "gps_sbas_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.gpsConfig.sbasMode, .config.lookup = { TABLE_GPS_SBAS_MODE } },
    { "gps_auto_config",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.gpsConfig.autoConfig, .config.lookup = { TABLE_OFF_ON } },
    { "gps_auto_baud",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.gpsConfig.autoBaud, .config.lookup = { TABLE_OFF_ON } },

    { "gps_pos_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOS], .config.minmax = { 0,  200 } },
    { "gps_pos_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOS], .config.minmax = { 0,  200 } },
    { "gps_pos_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOS], .config.minmax = { 0,  200 } },
    { "gps_posr_p",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOSR], .config.minmax = { 0,  200 } },
    { "gps_posr_i",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOSR], .config.minmax = { 0,  200 } },
    { "gps_posr_d",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOSR], .config.minmax = { 0,  200 } },
    { "gps_nav_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDNAVR], .config.minmax = { 0,  200 } },
    { "gps_nav_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDNAVR], .config.minmax = { 0,  200 } },
    { "gps_nav_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDNAVR], .config.minmax = { 0,  200 } },
    { "gps_wp_radius",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.gps_wp_radius, .config.minmax = { 0,  2000 } },
    { "nav_controls_heading",       VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].gpsProfile.nav_controls_heading, .config.lookup = { TABLE_OFF_ON } },
    { "nav_speed_min",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_speed_min, .config.minmax = { 10,  2000 } },
    { "nav_speed_max",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_speed_max, .config.minmax = { 10,  2000 } },
    { "nav_slew_rate",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_slew_rate, .config.minmax = { 0,  100 } },
#endif

    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rxConfig.serialrx_provider, .config.lookup = { TABLE_SERIAL_RX } },
    { "sbus_inversion",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rxConfig.sbus_inversion, .config.lookup = { TABLE_OFF_ON } },
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.spektrum_sat_bind, .config.minmax = { SPEKTRUM_SAT_BIND_DISABLED,  SPEKTRUM_SAT_BIND_MAX} },

#ifdef TELEMETRY
    { "telemetry_switch",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.telemetryConfig.telemetry_switch, .config.lookup = { TABLE_OFF_ON } },
    { "telemetry_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.telemetryConfig.telemetry_inversion, .config.lookup = { TABLE_OFF_ON } },
    { "frsky_default_lattitude",    VAR_FLOAT  | MASTER_VALUE,  &masterConfig.telemetryConfig.gpsNoFixLatitude, .config.minmax = { -90.0,  90.0 } },
    { "frsky_default_longitude",    VAR_FLOAT  | MASTER_VALUE,  &masterConfig.telemetryConfig.gpsNoFixLongitude, .config.minmax = { -180.0,  180.0 } },
    { "frsky_coordinates_format",   VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_coordinate_format, .config.minmax = { 0,  FRSKY_FORMAT_NMEA } },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.telemetryConfig.frsky_unit, .config.lookup = { TABLE_UNIT } },
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_vfas_precision, .config.minmax = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH } },
    { "hott_alarm_sound_interval",  VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.hottAlarmSoundInterval, .config.minmax = { 0,  120 } },
#endif

    { "battery_capacity",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.batteryCapacity, .config.minmax = { 0,  20000 } },
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatscale, .config.minmax = { VBAT_SCALE_MIN,  VBAT_SCALE_MAX } },
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatmaxcellvoltage, .config.minmax = { 10,  50 } },
    { "vbat_min_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatmincellvoltage, .config.minmax = { 10,  50 } },
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatwarningcellvoltage, .config.minmax = { 10,  50 } },
    { "current_meter_scale",        VAR_INT16  | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterScale, .config.minmax = { -10000,  10000 } },
    { "current_meter_offset",       VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterOffset, .config.minmax = { 0,  3300 } },
    { "multiwii_current_meter_output", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.batteryConfig.multiwiiCurrentMeterOutput, .config.lookup = { TABLE_OFF_ON } },
    { "current_meter_type",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.batteryConfig.currentMeterType, .config.lookup = { TABLE_CURRENT_SENSOR } },

    { "align_gyro",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.sensorAlignmentConfig.gyro_align, .config.lookup = { TABLE_ALIGNMENT } },
    { "align_acc",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.sensorAlignmentConfig.acc_align, .config.lookup = { TABLE_ALIGNMENT } },
    { "align_mag",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.sensorAlignmentConfig.mag_align, .config.lookup = { TABLE_ALIGNMENT } },

    { "align_board_roll",           VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.rollDegrees, .config.minmax = { -180,  360 } },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.pitchDegrees, .config.minmax = { -180,  360 } },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.yawDegrees, .config.minmax = { -180,  360 } },

    { "max_angle_inclination",      VAR_UINT16 | MASTER_VALUE,  &masterConfig.max_angle_inclination, .config.minmax = { 100,  900 } },

    { "gyro_lpf",                   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.gyro_lpf, .config.lookup = { TABLE_GYRO_LPF } },
    { "gyro_soft_lpf",              VAR_FLOAT  | MASTER_VALUE,  &masterConfig.soft_gyro_lpf_hz, .config.minmax = { 0,  500 } },
    { "moron_threshold",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.gyroConfig.gyroMovementCalibrationThreshold, .config.minmax = { 0,  128 } },
    { "imu_dcm_kp",                 VAR_UINT16 | MASTER_VALUE,  &masterConfig.dcm_kp, .config.minmax = { 0,  20000 } },
    { "imu_dcm_ki",                 VAR_UINT16 | MASTER_VALUE,  &masterConfig.dcm_ki, .config.minmax = { 0,  20000 } },

    { "alt_hold_deadband",          VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.alt_hold_deadband, .config.minmax = { 1,  250 } },
    { "alt_hold_fast_change",       VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].rcControlsConfig.alt_hold_fast_change, .config.lookup = { TABLE_OFF_ON } },
    { "deadband",                   VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.deadband, .config.minmax = { 0,  32 } },
    { "yaw_deadband",               VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.yaw_deadband, .config.minmax = { 0,  100 } },

    { "throttle_correction_value",  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].throttle_correction_value, .config.minmax = { 0,  150 } },
    { "throttle_correction_angle",  VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].throttle_correction_angle, .config.minmax = { 1,  900 } },

    { "yaw_control_direction",      VAR_INT8   | MASTER_VALUE,  &masterConfig.yaw_control_direction, .config.minmax = { -1,  1 } },

    { "pid_at_min_throttle",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &masterConfig.mixerConfig.pid_at_min_throttle, .config.lookup = { TABLE_OFF_ON } },
    { "airmode_saturation_limit",   VAR_UINT8  | MASTER_VALUE, &masterConfig.mixerConfig.airmode_saturation_limit, .config.minmax = { 0,  100 } },
    { "yaw_motor_direction",        VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.yaw_motor_direction, .config.minmax = { -1,  1 } },
    { "yaw_jump_prevention_limit",  VAR_UINT16 | MASTER_VALUE, &masterConfig.mixerConfig.yaw_jump_prevention_limit, .config.minmax = { YAW_JUMP_PREVENTION_LIMIT_LOW,  YAW_JUMP_PREVENTION_LIMIT_HIGH } },

#ifdef USE_SERVOS
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, &masterConfig.mixerConfig.tri_unarmed_servo, .config.lookup = { TABLE_OFF_ON } },
    { "servo_lowpass_freq",         VAR_FLOAT  | MASTER_VALUE, &masterConfig.mixerConfig.servo_lowpass_freq, .config.minmax = { 10,  400} },
    { "servo_lowpass_enable",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, &masterConfig.mixerConfig.servo_lowpass_enable, .config.lookup = { TABLE_OFF_ON } },
#endif

    { "default_rate_profile",       VAR_UINT8  | PROFILE_VALUE , &masterConfig.profile[0].defaultRateProfileIndex, .config.minmax = { 0,  MAX_CONTROL_RATE_PROFILE_COUNT - 1 } },
    { "rc_rate",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcRate8, .config.minmax = { 0,  250 } },
    { "rc_expo",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcExpo8, .config.minmax = { 0,  100 } },
    { "rc_yaw_expo",                VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcYawExpo8, .config.minmax = { 0,  100 } },
    { "thr_mid",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrMid8, .config.minmax = { 0,  100 } },
    { "thr_expo",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrExpo8, .config.minmax = { 0,  100 } },
    { "roll_rate",                  VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_ROLL], .config.minmax = { 0,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX } },
    { "pitch_rate",                 VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_PITCH], .config.minmax = { 0,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX } },
    { "yaw_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_YAW], .config.minmax = { 0,  CONTROL_RATE_CONFIG_YAW_RATE_MAX } },
    { "tpa_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].dynThrPID, .config.minmax = { 0,  CONTROL_RATE_CONFIG_TPA_MAX} },
    { "tpa_breakpoint",             VAR_UINT16 | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].tpa_breakpoint, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX} },

    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_delay, .config.minmax = { 0,  200 } },
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_off_delay, .config.minmax = { 0,  200 } },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_throttle, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX } },
    { "failsafe_kill_switch",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.failsafeConfig.failsafe_kill_switch, .config.lookup = { TABLE_OFF_ON } },
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_throttle_low_delay, .config.minmax = { 0,  300 } },
    { "failsafe_procedure",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_procedure, .config.minmax = { 0,  1 } },

    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.rx_min_usec, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX } },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.rx_max_usec, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX } },

#ifdef USE_SERVOS
    { "gimbal_mode",                VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].gimbalConfig.mode, .config.lookup = { TABLE_GIMBAL_MODE } },
#endif

    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.acc_hardware, .config.minmax = { 0,  ACC_MAX } },
    { "acc_cut_hz",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].acc_cut_hz, .config.minmax = { 0,  200 } },
    { "accxy_deadband",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].accDeadband.xy, .config.minmax = { 0,  100 } },
    { "accz_deadband",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].accDeadband.z, .config.minmax = { 0,  100 } },
    { "accz_lpf_cutoff",            VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].accz_lpf_cutoff, .config.minmax = { 1,  20 } },
    { "acc_unarmedcal",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].acc_unarmedcal, .config.lookup = { TABLE_OFF_ON } },
    { "acc_trim_pitch",             VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].accelerometerTrims.values.pitch, .config.minmax = { -300,  300 } },
    { "acc_trim_roll",              VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].accelerometerTrims.values.roll, .config.minmax = { -300,  300 } },

#ifdef BARO
    { "baro_tab_size",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_sample_count, .config.minmax = { 0,  BARO_SAMPLE_COUNT_MAX } },
    { "baro_noise_lpf",             VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_noise_lpf, .config.minmax = { 0 , 1 } },
    { "baro_cf_vel",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_cf_vel, .config.minmax = { 0 , 1 } },
    { "baro_cf_alt",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_cf_alt, .config.minmax = { 0 , 1 } },
    { "baro_hardware",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.baro_hardware, .config.minmax = { 0,  BARO_MAX } },
#endif

#ifdef MAG
    { "mag_hardware",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.mag_hardware, .config.minmax = { 0,  MAG_MAX } },
    { "mag_declination",            VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].mag_declination, .config.minmax = { -18000,  18000 } },
#endif

    { "pid_delta_method",           VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].pidProfile.deltaMethod, .config.lookup = { TABLE_DELTA_METHOD } },

    { "pid_controller",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].pidProfile.pidController, .config.lookup = { TABLE_PID_CONTROLLER } },

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PITCH], .config.minmax = { PID_MIN,  PID_MAX } },
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PITCH], .config.minmax = { PID_MIN,  PID_MAX } },
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PITCH], .config.minmax = { PID_MIN,  PID_MAX } },
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[ROLL], .config.minmax = { PID_MIN,  PID_MAX } },
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[ROLL], .config.minmax = { PID_MIN,  PID_MAX } },
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[ROLL], .config.minmax = { PID_MIN,  PID_MAX } },
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[YAW], .config.minmax = { PID_MIN,  PID_MAX } },
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[YAW], .config.minmax = { PID_MIN,  PID_MAX } },
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[YAW], .config.minmax = { PID_MIN,  PID_MAX } },

    { "p_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[PITCH], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "i_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[PITCH], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "d_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[PITCH], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "p_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[ROLL], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "i_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[ROLL], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "d_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[ROLL], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "p_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[YAW], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "i_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[YAW], .config.minmax = { PID_F_MIN,  PID_F_MAX } },
    { "d_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[YAW], .config.minmax = { PID_F_MIN,  PID_F_MAX } },

    { "level_horizon",              VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.H_level, .config.minmax = { 0,  10 } },
    { "level_angle",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.A_level, .config.minmax = { 0,  10 } },
    { "sensitivity_horizon",        VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.H_sensitivity, .config.minmax = { 0,  250 } },

    { "p_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDALT], .config.minmax = { PID_MIN,  PID_MAX } },
    { "i_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDALT], .config.minmax = { PID_MIN,  PID_MAX } },
    { "d_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDALT], .config.minmax = { PID_MIN,  PID_MAX } },

    { "p_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDLEVEL], .config.minmax = { PID_MIN,  PID_MAX } },
    { "i_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDLEVEL], .config.minmax = { PID_MIN,  PID_MAX } },
    { "d_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDLEVEL], .config.minmax = { PID_MIN,  PID_MAX } },

    { "p_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDVEL], .config.minmax = { PID_MIN,  PID_MAX } },
    { "i_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDVEL], .config.minmax = { PID_MIN,  PID_MAX } },
    { "d_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDVEL], .config.minmax = { PID_MIN,  PID_MAX } },

    { "yaw_p_limit",                VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yaw_p_limit, .config.minmax = { YAW_P_LIMIT_MIN, YAW_P_LIMIT_MAX } },
    { "dterm_cut_hz",               VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.dterm_cut_hz, .config.minmax = {0, 500 } },

#ifdef GTUNE
    { "gtune_loP_rll",              VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_lolimP[FD_ROLL], .config.minmax = { 10,  200 } },
    { "gtune_loP_ptch",             VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_lolimP[FD_PITCH], .config.minmax = { 10,  200 } },
    { "gtune_loP_yw",               VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_lolimP[FD_YAW], .config.minmax = { 10,  200 } },
    { "gtune_hiP_rll",              VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_hilimP[FD_ROLL], .config.minmax = { 0,  200 } },
    { "gtune_hiP_ptch",             VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_hilimP[FD_PITCH], .config.minmax = { 0,  200 } },
    { "gtune_hiP_yw",               VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_hilimP[FD_YAW], .config.minmax = { 0,  200 } },
    { "gtune_pwr",                  VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_pwr, .config.minmax = { 0,  10 } },
    { "gtune_settle_time",          VAR_UINT16 | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_settle_time, .config.minmax = { 200,  1000 } },
    { "gtune_average_cycles",       VAR_UINT8  | PROFILE_VALUE,  &masterConfig.profile[0].pidProfile.gtune_average_cycles, .config.minmax = { 8,  128 } },
#endif

#ifdef BLACKBOX
    { "blackbox_rate_num",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_rate_num, .config.minmax = { 1,  32 } },
    { "blackbox_rate_denom",        VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_rate_denom, .config.minmax = { 1,  32 } },
    { "blackbox_device",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.blackbox_device, .config.lookup = { TABLE_BLACKBOX_DEVICE } },
#endif

    { "magzero_x",                  VAR_INT16  | MASTER_VALUE, &masterConfig.magZero.raw[X], .config.minmax = { -32768,  32767 } },
    { "magzero_y",                  VAR_INT16  | MASTER_VALUE, &masterConfig.magZero.raw[Y], .config.minmax = { -32768,  32767 } },
    { "magzero_z",                  VAR_INT16  | MASTER_VALUE, &masterConfig.magZero.raw[Z], .config.minmax = { -32768,  32767 } },
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(clivalue_t))


typedef union {
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetVar(const clivalue_t *var, const int_float_value_t value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);

static void cliShowParseError(void)
{
    pifLog_Print(LT_NONE, "Parse error\r\n");
}

static void cliShowArgumentRangeError(char *name, int min, int max)
{
    pifLog_Printf(LT_NONE, "%s must be between %d and %d\r\n", name, min, max);
}

static void processChannelRangeArgs(int argc, char *argv[], channelRange_t *range)
{
    int val;

    for (int argIndex = 0; argIndex < 2; argIndex++) {
        if (argc > argIndex) {
            val = atoi(argv[argIndex]);
            val = CHANNEL_VALUE_TO_STEP(val);
            if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
                if (argIndex == 0) {
                    range->startStep = val;
                } else {
                    range->endStep = val;
                }
            }
        }
    }
}

static int cliRxFail(int argc, char *argv[])
{
    uint8_t channel;
    char buf[3];
    char* argn[1];

    if (argc == 0) {
        // print out rxConfig failsafe settings
        for (channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
            itoa(channel, buf, 3);
            argn[0] = buf;
            cliRxFail(1, argn);
        }
    } else {
        channel = atoi(argv[0]);
        if ((channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)) {

            rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &masterConfig.rxConfig.failsafe_channel_configurations[channel];

            uint16_t value;
            rxFailsafeChannelType_e type = (channel < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_TYPE_FLIGHT : RX_FAILSAFE_TYPE_AUX;
            rxFailsafeChannelMode_e mode = channelFailsafeConfiguration->mode;
            bool requireValue = channelFailsafeConfiguration->mode == RX_FAILSAFE_MODE_SET;

            if (argc > 1) {
                char *p = strchr(rxFailsafeModeCharacters, *argv[1]);
                if (p) {
                    uint8_t requestedMode = p - rxFailsafeModeCharacters;
                    mode = rxFailsafeModesTable[type][requestedMode];
                } else {
                    mode = RX_FAILSAFE_MODE_INVALID;
                }
                if (mode == RX_FAILSAFE_MODE_INVALID) {
                    cliShowParseError();
                    return CLI_ERROR_PARSE;
                }

                requireValue = mode == RX_FAILSAFE_MODE_SET;

                if (argc > 2) {
                    if (!requireValue) {
                        cliShowParseError();
                        return CLI_ERROR_PARSE;
                    }
                    value = atoi(argv[2]);
                    value = CHANNEL_VALUE_TO_RXFAIL_STEP(value);
                    if (value > MAX_RXFAIL_RANGE_STEP) {
                        pifLog_Print(LT_NONE, "Value out of range\r\n");
                        return CLI_ERROR_OUT_OF_RANGE;
                    }

                    channelFailsafeConfiguration->step = value;
                } else if (requireValue) {
                    cliShowParseError();
                    return CLI_ERROR_PARSE;
                }
                channelFailsafeConfiguration->mode = mode;

            }

            char modeCharacter = rxFailsafeModeCharacters[channelFailsafeConfiguration->mode];

            // triple use of cliPrintf below
            // 1. acknowledge interpretation on command,
            // 2. query current setting on single item,
            // 3. recursive use for full list.

            if (requireValue) {
                pifLog_Printf(LT_NONE, "rxfail %u %c %d\r\n",
                    channel,
                    modeCharacter,
                    RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfiguration->step)
                );
            } else {
                pifLog_Printf(LT_NONE, "rxfail %u %c\r\n",
                    channel,
                    modeCharacter
                );
            }
        } else {
            cliShowArgumentRangeError("channel", 0, MAX_SUPPORTED_RC_CHANNEL_COUNT - 1);
            return CLI_ERROR_OUT_OF_RANGE;
        }
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliAux(int argc, char *argv[])
{
    int i, val = 0;

    if (argc == 0) {
        // print out aux channel settings
        for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            pifLog_Printf(LT_NONE, "aux %u %u %u %u %u\r\n",
                i,
                mac->modeId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep)
            );
        }
    } else {
        i = atoi(argv[0]);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            if (argc > 1) {
                val = atoi(argv[1]);
                if (val >= 0 && val < CHECKBOX_ITEM_COUNT) {
                    mac->modeId = val;
                }
            }
            if (argc > 2) {
                val = atoi(argv[2]);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    mac->auxChannelIndex = val;
                }
            }
            processChannelRangeArgs(argc + 3, &argv[3], &mac->range);

            if (argc - 1 != 4) {
                memset(mac, 0, sizeof(modeActivationCondition_t));
                return PIF_LOG_CMD_TOO_FEW_ARGS;
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_MODE_ACTIVATION_CONDITION_COUNT - 1);
            return CLI_ERROR_OUT_OF_RANGE;
        }
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliSerial(int argc, char *argv[])
{
    int i, val;

    if (argc == 0) {
        for (i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(masterConfig.serialConfig.portConfigs[i].identifier)) {
                continue;
            };
            pifLog_Printf(LT_NONE, "serial %d %d %ld %ld %ld %ld\r\n" ,
                masterConfig.serialConfig.portConfigs[i].identifier,
                masterConfig.serialConfig.portConfigs[i].functionMask,
                baudRates[masterConfig.serialConfig.portConfigs[i].msp_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].gps_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].telemetry_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].blackbox_baudrateIndex]
            );
        }
        return PIF_LOG_CMD_NO_ERROR;
    }

    serialPortConfig_t portConfig;
    memset(&portConfig, 0 , sizeof(portConfig));

    serialPortConfig_t *currentConfig;

    val = atoi(argv[0]);
    currentConfig = serialFindPortConfiguration(val);
    if (currentConfig) {
        portConfig.identifier = val;
    }

    if (argc > 1) {
        val = atoi(argv[1]);
        portConfig.functionMask = val & 0xFFFF;
    }

    for (i = 0; i < 4; i ++) {
        if (argc > 2 + i) {
            break;
        }

        val = atoi(argv[2 + i]);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch(i) {
            case 0:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.msp_baudrateIndex = baudRateIndex;
                break;
            case 1:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.gps_baudrateIndex = baudRateIndex;
                break;
            case 2:
                if (baudRateIndex != BAUD_AUTO && baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.telemetry_baudrateIndex = baudRateIndex;
                break;
            case 3:
                if (baudRateIndex < BAUD_19200 || baudRateIndex > BAUD_250000) {
                    continue;
                }
                portConfig.blackbox_baudrateIndex = baudRateIndex;
                break;
        }
    }

    if (argc < 6) {
        cliShowParseError();
        return CLI_ERROR_PARSE;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));

   	return PIF_LOG_CMD_NO_ERROR;
}

static int cliAdjustmentRange(int argc, char *argv[])
{
    int i, val = 0;

    if (argc == 0) {
        // print out adjustment ranges channel settings
        for (i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            adjustmentRange_t *ar = &currentProfile->adjustmentRanges[i];
            pifLog_Printf(LT_NONE, "adjrange %u %u %u %u %u %u %u\r\n",
                i,
                ar->adjustmentIndex,
                ar->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
                ar->adjustmentFunction,
                ar->auxSwitchChannelIndex
            );
        }
    } else {
        i = atoi(argv[0]);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = &currentProfile->adjustmentRanges[i];

            if (argc > 1) {
                val = atoi(argv[1]);
                if (val >= 0 && val < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    ar->adjustmentIndex = val;
                }
            }
            if (argc > 2) {
                val = atoi(argv[2]);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                }
            }

            processChannelRangeArgs(argc + 3, &argv[3], &ar->range);

            if (argc > 5) {
                val = atoi(argv[5]);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentFunction = val;
                }
            }
            if (argc > 6) {
                val = atoi(argv[6]);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                }
            }

            if (argc - 1 != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
                cliShowParseError();
                return CLI_ERROR_PARSE;
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_ADJUSTMENT_RANGE_COUNT - 1);
            return CLI_ERROR_OUT_OF_RANGE;
        }
    }
   	return PIF_LOG_CMD_NO_ERROR;
}

static int cliMotorMix(int argc, char *argv[])
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(cmdline);
#else
    int i;
    int num_motors = 0;
    uint8_t len;
    char buf[16];

    if (argc == 0) {
        pifLog_Print(LT_NONE, "Motor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (masterConfig.customMotorMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            pifLog_Printf(LT_NONE, "#%d:\t", i);
            pifLog_Printf(LT_NONE, "%s\t", ftoa(masterConfig.customMotorMixer[i].throttle, buf));
            pifLog_Printf(LT_NONE, "%s\t", ftoa(masterConfig.customMotorMixer[i].roll, buf));
            pifLog_Printf(LT_NONE, "%s\t", ftoa(masterConfig.customMotorMixer[i].pitch, buf));
            pifLog_Printf(LT_NONE, "%s\r\n", ftoa(masterConfig.customMotorMixer[i].yaw, buf));
        }
    } else if (strncasecmp(argv[0], "reset", 5) == 0) {
        // erase custom mixer
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
            masterConfig.customMotorMixer[i].throttle = 0.0f;
    } else if (strncasecmp(argv[0], "load", 4) == 0) {
        if (argc > 1) {
            len = strlen(argv[1]);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    pifLog_Print(LT_NONE, "Invalid name\r\n");
                    return CLI_ERROR_INVALID;
                }
                if (strncasecmp(argv[1], mixerNames[i], len) == 0) {
                    mixerLoadMix(i, masterConfig.customMotorMixer);
                    pifLog_Printf(LT_NONE, "Loaded %s\r\n", mixerNames[i]);
                    cliMotorMix(0, NULL);
                    break;
                }
            }
        }
    } else {
        i = atoi(argv[0]); // get motor number
        if (i < MAX_SUPPORTED_MOTORS) {
            if (argc > 1) {
                masterConfig.customMotorMixer[i].throttle = fastA2F(argv[1]);
            }
            if (argc > 2) {
                masterConfig.customMotorMixer[i].roll = fastA2F(argv[2]);
            }
            if (argc > 3) {
                masterConfig.customMotorMixer[i].pitch = fastA2F(argv[3]);
            }
            if (argc > 4) {
                masterConfig.customMotorMixer[i].yaw = fastA2F(argv[4]);
            }
            if (argc - 1 != 4) {
                cliShowParseError();
                return CLI_ERROR_PARSE;
            } else {
                cliMotorMix(0, NULL);
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS - 1);
            return CLI_ERROR_OUT_OF_RANGE;
        }
    }
#endif
   	return PIF_LOG_CMD_NO_ERROR;
}

static int cliRxRange(int argc, char *argv[])
{
    int i;

    if (argc == 0) {
        for (i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
            rxChannelRangeConfiguration_t *channelRangeConfiguration = &masterConfig.rxConfig.channelRanges[i];
            pifLog_Printf(LT_NONE, "rxrange %u %u %u\r\n", i, channelRangeConfiguration->min, channelRangeConfiguration->max);
        }
    } else if (strcasecmp(argv[0], "reset") == 0) {
        resetAllRxChannelRangeConfigurations(masterConfig.rxConfig.channelRanges);
    } else {
        i = atoi(argv[0]);
        if (i >= 0 && i < NON_AUX_CHANNEL_COUNT) {
            int rangeMin, rangeMax;

            if (argc > 1) {
                rangeMin = atoi(argv[1]);
            }

            if (argc > 2) {
                rangeMax = atoi(argv[2]);
            }

            if (argc - 1 != 2) {
                cliShowParseError();
                return CLI_ERROR_PARSE;
            } else if (rangeMin < PWM_PULSE_MIN || rangeMin > PWM_PULSE_MAX || rangeMax < PWM_PULSE_MIN || rangeMax > PWM_PULSE_MAX) {
                cliShowParseError();
                return CLI_ERROR_PARSE;
            } else {
                rxChannelRangeConfiguration_t *channelRangeConfiguration = &masterConfig.rxConfig.channelRanges[i];
                channelRangeConfiguration->min = rangeMin;
                channelRangeConfiguration->max = rangeMax;
            }
        } else {
            cliShowArgumentRangeError("channel", 0, NON_AUX_CHANNEL_COUNT - 1);
            return CLI_ERROR_OUT_OF_RANGE;
        }
    }
   	return PIF_LOG_CMD_NO_ERROR;
}

#ifdef LED_STRIP
static int cliLed(int argc, char *argv[])
{
    int i;
    char ledConfigBuffer[20];

    if (argc == 0) {
        for (i = 0; i < MAX_LED_STRIP_LENGTH; i++) {
            generateLedConfig(i, ledConfigBuffer, sizeof(ledConfigBuffer));
            pifLog_Printf(LT_NONE, "led %u %s\r\n", i, ledConfigBuffer);
        }
    } else {
        i = atoi(argv[0]);
        if (i < MAX_LED_STRIP_LENGTH) {
            if (!parseLedStripConfig(i, argc > 1 ? argv[1] : NULL)) {
                cliShowParseError();
                return CLI_ERROR_PARSE;
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_LED_STRIP_LENGTH - 1);
            return CLI_ERROR_OUT_OF_RANGE;
        }
    }
   	return PIF_LOG_CMD_NO_ERROR;
}

static int cliColor(int argc, char *argv[])
{
    int i;

    if (argc == 0) {
        for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
            pifLog_Printf(LT_NONE, "color %u %d,%u,%u\r\n",
                i,
                masterConfig.colors[i].h,
                masterConfig.colors[i].s,
                masterConfig.colors[i].v
            );
        }
    } else {
        i = atoi(argv[0]);
        if (i < CONFIGURABLE_COLOR_COUNT) {
            if (!parseColor(i, argc > 1 ? argv[1] : NULL)) {
                cliShowParseError();
                return CLI_ERROR_PARSE;
            }
        } else {
            cliShowArgumentRangeError("index", 0, CONFIGURABLE_COLOR_COUNT - 1);
            return CLI_ERROR_OUT_OF_RANGE;
        }
    }
   	return PIF_LOG_CMD_NO_ERROR;
}
#endif

#ifdef USE_SERVOS
static int cliServo(int argc, char *argv[])
{
    enum { SERVO_ARGUMENT_COUNT = 8 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;

    if (argc == 0) {
        // print out servo settings
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo = &currentProfile->servoConf[i];

            pifLog_Printf(LT_NONE, "servo %u %d %d %d %d %d %d %d\r\n",
                i,
                servo->min,
                servo->max,
                servo->middle,
                servo->angleAtMin,
                servo->angleAtMax,
                servo->rate,
                servo->forwardFromChannel
            );
        }
    } else {
        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.

        // If command line doesn't fit the format, don't modify the config
        if (argc < SERVO_ARGUMENT_COUNT) {
            cliShowParseError();
            return CLI_ERROR_PARSE;
        }
        for (i = 0; i < SERVO_ARGUMENT_COUNT; i++) {
            arguments[i] = atoi(argv[i]);
        }

        enum {INDEX = 0, MIN, MAX, MIDDLE, ANGLE_AT_MIN, ANGLE_AT_MAX, RATE, FORWARD};

        i = arguments[INDEX];

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowParseError();
            return CLI_ERROR_PARSE;
        }

        servo = &currentProfile->servoConf[i];

        if (
            arguments[MIN] < PWM_PULSE_MIN || arguments[MIN] > PWM_PULSE_MAX ||
            arguments[MAX] < PWM_PULSE_MIN || arguments[MAX] > PWM_PULSE_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -100 || arguments[RATE] > 100 ||
            arguments[FORWARD] >= MAX_SUPPORTED_RC_CHANNEL_COUNT ||
            arguments[ANGLE_AT_MIN] < 0 || arguments[ANGLE_AT_MIN] > 180 ||
            arguments[ANGLE_AT_MAX] < 0 || arguments[ANGLE_AT_MAX] > 180
        ) {
            cliShowParseError();
            return CLI_ERROR_PARSE;
        }

        servo->min = arguments[1];
        servo->max = arguments[2];
        servo->middle = arguments[3];
        servo->angleAtMin = arguments[4];
        servo->angleAtMax = arguments[5];
        servo->rate = arguments[6];
        servo->forwardFromChannel = arguments[7];
    }
   	return PIF_LOG_CMD_NO_ERROR;
}
#endif

#ifdef USE_SERVOS
static int cliServoMix(int argc, char *argv[])
{
    int i, rtn = PIF_LOG_CMD_NO_ERROR;
    uint8_t len;
    char* argn[1];
    int args[8];

    if (argc == 0) {

        pifLog_Print(LT_NONE, "Rule\tServo\tSource\tRate\tSpeed\tMin\tMax\tBox\r\n");

        for (i = 0; i < MAX_SERVO_RULES; i++) {
            if (masterConfig.customServoMixer[i].rate == 0)
                break;

            pifLog_Printf(LT_NONE, "#%d:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                i,
                masterConfig.customServoMixer[i].targetChannel,
                masterConfig.customServoMixer[i].inputSource,
                masterConfig.customServoMixer[i].rate,
                masterConfig.customServoMixer[i].speed,
                masterConfig.customServoMixer[i].min,
                masterConfig.customServoMixer[i].max,
                masterConfig.customServoMixer[i].box
            );
        }
        pifLog_Print(LT_NONE, "\r\n");
    } else if (strncasecmp(argv[0], "reset", 5) == 0) {
        // erase custom mixer
        memset(masterConfig.customServoMixer, 0, sizeof(masterConfig.customServoMixer));
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            currentProfile->servoConf[i].reversedSources = 0;
        }
    } else if (strncasecmp(argv[0], "load", 4) == 0) {
        if (argc > 1) {
            len = strlen(argv[1]);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    pifLog_Print(LT_NONE, "Invalid name\r\n");
                    return CLI_ERROR_INVALID;
                }
                if (strncasecmp(argv[1], mixerNames[i], len) == 0) {
                    servoMixerLoadMix(i, masterConfig.customServoMixer);
                    pifLog_Printf(LT_NONE, "Loaded %s\r\n", mixerNames[i]);
                    cliServoMix(0, NULL);
                    break;
                }
            }
        }
    } else if (strncasecmp(argv[0], "reverse", 7) == 0) {
        enum {SERVO = 0, INPUT, REVERSE, ARGS_COUNT};
        int servoIndex, inputSource;

        if (argc == 1) {
            pifLog_Print(LT_NONE, "s");
            for (inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                pifLog_Printf(LT_NONE, "\ti%d", inputSource);
            pifLog_Print(LT_NONE, "\r\n");

            for (servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
                pifLog_Printf(LT_NONE, "%d", servoIndex);
                for (inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                    pifLog_Printf(LT_NONE, "\t%s  ", (currentProfile->servoConf[servoIndex].reversedSources & (1 << inputSource)) ? "r" : "n");
                pifLog_Print(LT_NONE, "\r\n");
            }
            return PIF_LOG_CMD_NO_ERROR;
        }

        if (argc != 4) {
            cliShowParseError();
            return CLI_ERROR_PARSE;
        }
        for (i = 0; i < ARGS_COUNT - 1; i++) {
            args[i] = atoi(argv[1 + i]);
        }

        if (args[SERVO] >= 0 && args[SERVO] < MAX_SUPPORTED_SERVOS
                && args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT
                && (*argv[3] == 'r' || *argv[3] == 'n')) {
            if (*argv[3] == 'r')
                currentProfile->servoConf[args[SERVO]].reversedSources |= 1 << args[INPUT];
            else
                currentProfile->servoConf[args[SERVO]].reversedSources &= ~(1 << args[INPUT]);
        } else {
            cliShowParseError();
            rtn = CLI_ERROR_PARSE;
        }

        argn[0] = "reverse";
        cliServoMix(1, argn);
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, MIN, MAX, BOX, ARGS_COUNT};
        if (argc != ARGS_COUNT) {
            cliShowParseError();
            return CLI_ERROR_PARSE;
        }
        for (i = 0; i < ARGS_COUNT; i++) {
            args[i] = atoi(argv[i]);
        }

        i = args[RULE];
        if (i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] >= 0 && args[TARGET] < MAX_SUPPORTED_SERVOS &&
            args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT &&
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[MIN] >= 0 && args[MIN] <= 100 &&
            args[MAX] >= 0 && args[MAX] <= 100 && args[MIN] < args[MAX] &&
            args[BOX] >= 0 && args[BOX] <= MAX_SERVO_BOXES) {
            masterConfig.customServoMixer[i].targetChannel = args[TARGET];
            masterConfig.customServoMixer[i].inputSource = args[INPUT];
            masterConfig.customServoMixer[i].rate = args[RATE];
            masterConfig.customServoMixer[i].speed = args[SPEED];
            masterConfig.customServoMixer[i].min = args[MIN];
            masterConfig.customServoMixer[i].max = args[MAX];
            masterConfig.customServoMixer[i].box = args[BOX];
            cliServoMix(1, NULL);
        } else {
            cliShowParseError();
            rtn = CLI_ERROR_PARSE;
        }
    }
    return rtn;
}
#endif

#ifdef USE_SDCARD

static void cliWriteBytes(const uint8_t *buffer, int count)
{
    while (count > 0) {
        pifLog_Print(LT_NONE, "%c", *buffer);
        buffer++;
        count--;
    }
}

static int cliSdInfo(int argc, char *argv[]) {
    UNUSED(argc);
    UNUSED(argv);

    pifLog_Print(LT_NONE, "SD card: ");

    if (!sdcard_isInserted()) {
        pifLog_Print(LT_NONE, "None inserted\r\n");
        return;
    }

    if (!sdcard_isInitialized()) {
        pifLog_Print(LT_NONE, "Startup failed\r\n");
        return;
    }

    const sdcardMetadata_t *metadata = sdcard_getMetadata();

    pifLog_Printf(LT_NONE, "Manufacturer 0x%x, %ukB, %02d/%04d, v%d.%d, '",
        metadata->manufacturerID,
        metadata->numBlocks / 2, /* One block is half a kB */
        metadata->productionMonth,
        metadata->productionYear,
        metadata->productRevisionMajor,
        metadata->productRevisionMinor
    );

    cliWriteBytes((uint8_t*)metadata->productName, sizeof(metadata->productName));

    pifLog_Print(LT_NONE, "'\r\n" "Filesystem: ");

    switch (afatfs_getFilesystemState()) {
        case AFATFS_FILESYSTEM_STATE_READY:
            pifLog_Print(LT_NONE, "Ready");
        break;
        case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
            pifLog_Print(LT_NONE, "Initializing");
        break;
        case AFATFS_FILESYSTEM_STATE_UNKNOWN:
        case AFATFS_FILESYSTEM_STATE_FATAL:
            pifLog_Print(LT_NONE, "Fatal");

            switch (afatfs_getLastError()) {
                case AFATFS_ERROR_BAD_MBR:
                    pifLog_Print(LT_NONE, " - no FAT MBR partitions");
                break;
                case AFATFS_ERROR_BAD_FILESYSTEM_HEADER:
                    pifLog_Print(LT_NONE, " - bad FAT header");
                break;
                case AFATFS_ERROR_GENERIC:
                case AFATFS_ERROR_NONE:
                    ; // Nothing more detailed to print
                break;
            }

            pifLog_Print(LT_NONE, "\r\n");
        break;
    }
    return PIF_LOG_CMD_NO_ERROR;
}

#endif

#ifdef USE_FLASHFS

static int cliFlashInfo(int argc, char *argv[])
{
    const flashGeometry_t *layout = flashfsGetGeometry();

    UNUSED(argc);
    UNUSED(argv);

    pifLog_Printf(LT_NONE, "Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u, usedSize=%u\r\n",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize, flashfsGetOffset());
    return PIF_LOG_CMD_NO_ERROR;
}

static int cliFlashErase(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    pifLog_Print(LT_NONE, "Erasing...\r\n");
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
        delay(100);
    }

    pifLog_Print(LT_NONE, "Done.\r\n");
    return PIF_LOG_CMD_NO_ERROR;
}

#ifdef USE_FLASH_TOOLS

static int cliFlashWrite(int argc, char *argv[])
{
    uint32_t address = atoi(argv[0]);

    if (argc > 1) {
        cliShowParseError();
        return CLI_ERROR_PARSE;
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(argv[1]), true);
        flashfsFlushSync();

        pifLog_Printf(LT_NONE, "Wrote %u bytes at %u.\r\n", strlen(argv[1]), address);
    }
    return PIF_LOG_CMD_NO_ERROR;
}

static int cliFlashRead(int argc, char *argv[])
{
    uint32_t address = atoi(argv[0]);
    uint32_t length;
    int i;

    uint8_t buffer[32];

    if (argc > 1) {
        cliShowParseError();
        return CLI_ERROR_PARSE;
    } else {
        length = atoi(argv[1]);

        pifLog_Printf(LT_NONE, "Reading %u bytes at %u:\r\n", length, address);

        while (length > 0) {
            int bytesRead;

            bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (i = 0; i < bytesRead; i++) {
                pifLog_Print(LT_NONE, "%c", buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                //Assume we reached the end of the volume or something fatal happened
                break;
            }
        }
        pifLog_Print(LT_NONE, "\r\n");
    }
    return PIF_LOG_CMD_NO_ERROR;
}

#endif
#endif

static void dumpValues(uint16_t valueSection)
{
    uint32_t i;
    const clivalue_t *value;
    for (i = 0; i < VALUE_COUNT; i++) {
        value = &valueTable[i];

        if ((value->type & VALUE_SECTION_MASK) != valueSection) {
            continue;
        }

        pifLog_Printf(LT_NONE, "set %s = ", valueTable[i].name);
        cliPrintVar(value, 0);
        pifLog_Print(LT_NONE, "\r\n");
    }
}

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_CONTROL_RATE_PROFILE = (1 << 2)
} dumpFlags_e;

#define DUMP_ALL (DUMP_MASTER | DUMP_PROFILE | DUMP_CONTROL_RATE_PROFILE)


static const char* const sectionBreak = "\r\n";

#define printSectionBreak() pifLog_Printf(LT_NONE, (char *)sectionBreak)

static int cliDump(int argc, char *argv[])
{
    unsigned int i;
    char buf[16];
    uint32_t mask;

#ifndef USE_QUAD_MIXER_ONLY
    float thr, roll, pitch, yaw;
#endif

    uint8_t dumpMask = DUMP_ALL;

    UNUSED(argc);

    if (strcasecmp(argv[0], "master") == 0) {
        dumpMask = DUMP_MASTER; // only
    }
    if (strcasecmp(argv[0], "profile") == 0) {
        dumpMask = DUMP_PROFILE; // only
    }
    if (strcasecmp(argv[0], "rates") == 0) {
        dumpMask = DUMP_CONTROL_RATE_PROFILE; // only
    }

    if (dumpMask & DUMP_MASTER) {

        pifLog_Print(LT_NONE, "\r\n# version\r\n");
        cliVersion(0, NULL);

        pifLog_Print(LT_NONE, "\r\n# dump master\r\n");
        pifLog_Print(LT_NONE, "\r\n# mixer\r\n");

#ifndef USE_QUAD_MIXER_ONLY
        pifLog_Printf(LT_NONE, "mixer %s\r\n", mixerNames[masterConfig.mixerMode - 1]);

        pifLog_Print(LT_NONE, "mmix reset\r\n");

        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (masterConfig.customMotorMixer[i].throttle == 0.0f)
                break;
            thr = masterConfig.customMotorMixer[i].throttle;
            roll = masterConfig.customMotorMixer[i].roll;
            pitch = masterConfig.customMotorMixer[i].pitch;
            yaw = masterConfig.customMotorMixer[i].yaw;
            pifLog_Printf(LT_NONE, "mmix %d", i);
            if (thr < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Print(LT_NONE, ftoa(thr, buf));
            if (roll < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Print(LT_NONE, ftoa(roll, buf));
            if (pitch < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Print(LT_NONE, ftoa(pitch, buf));
            if (yaw < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Printf(LT_NONE, "%s\r\n", ftoa(yaw, buf));
        }

#ifdef USE_SERVOS
        // print custom servo mixer if exists
        pifLog_Print(LT_NONE, "smix reset\r\n");

        for (i = 0; i < MAX_SERVO_RULES; i++) {

            if (masterConfig.customServoMixer[i].rate == 0)
                break;

            pifLog_Printf(LT_NONE, "smix %d %d %d %d %d %d %d %d\r\n",
                i,
                masterConfig.customServoMixer[i].targetChannel,
                masterConfig.customServoMixer[i].inputSource,
                masterConfig.customServoMixer[i].rate,
                masterConfig.customServoMixer[i].speed,
                masterConfig.customServoMixer[i].min,
                masterConfig.customServoMixer[i].max,
                masterConfig.customServoMixer[i].box
            );
        }

#endif
#endif

        pifLog_Print(LT_NONE, "\r\n\r\n# feature\r\n");

        mask = featureMask();
        for (i = 0; ; i++) { // disable all feature first
            if (featureNames[i] == NULL)
                break;
            pifLog_Printf(LT_NONE, "feature -%s\r\n", featureNames[i]);
        }
        for (i = 0; ; i++) {  // reenable what we want.
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                pifLog_Printf(LT_NONE, "feature %s\r\n", featureNames[i]);
        }

        pifLog_Print(LT_NONE, "\r\n\r\n# map\r\n");

        for (i = 0; i < 8; i++)
            buf[masterConfig.rxConfig.rcmap[i]] = rcChannelLetters[i];
        buf[i] = '\0';
        pifLog_Printf(LT_NONE, "map %s\r\n", buf);

        pifLog_Print(LT_NONE, "\r\n\r\n# serial\r\n");
        cliSerial(0, NULL);

#ifdef LED_STRIP
        pifLog_Print(LT_NONE, "\r\n\r\n# led\r\n");
        cliLed(0, NULL);

        pifLog_Print(LT_NONE, "\r\n\r\n# color\r\n");
        cliColor(0, NULL);
#endif
        printSectionBreak();
        dumpValues(MASTER_VALUE);

        pifLog_Print(LT_NONE, "\r\n# rxfail\r\n");
        cliRxFail(0, NULL);
    }

    if (dumpMask & DUMP_PROFILE) {
        pifLog_Print(LT_NONE, "\r\n# dump profile\r\n");

        pifLog_Print(LT_NONE, "\r\n# profile\r\n");
        cliProfile(0, NULL);

        pifLog_Print(LT_NONE, "\r\n# aux\r\n");

        cliAux(0, NULL);

        pifLog_Print(LT_NONE, "\r\n# adjrange\r\n");

        cliAdjustmentRange(0, NULL);

        pifLog_Print(LT_NONE, "\r\n# rxrange\r\n");

        cliRxRange(0, NULL);

#ifdef USE_SERVOS
        pifLog_Print(LT_NONE, "\r\n# servo\r\n");

        cliServo(0, NULL);

        // print servo directions
        unsigned int channel;

        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            for (channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                if (servoDirection(i, channel) < 0) {
                    pifLog_Printf(LT_NONE, "smix reverse %d %d r\r\n", i , channel);
                }
            }
        }
#endif

        printSectionBreak();

        dumpValues(PROFILE_VALUE);
    }

    if (dumpMask & DUMP_CONTROL_RATE_PROFILE) {
        pifLog_Print(LT_NONE, "\r\n# dump rates\r\n");

        pifLog_Print(LT_NONE, "\r\n# rateprofile\r\n");
        cliRateProfile(0, NULL);

        printSectionBreak();

        dumpValues(CONTROL_RATE_VALUE);
    }
    return PIF_LOG_CMD_NO_ERROR;
}

void cliEnter(serialPort_t *serialPort)
{
    cliMode = 1;
    cliPort = serialPort;

    pifLog_Init();

    if (!pifLog_AttachComm(&serialPort->comm)) return;
    if (!pifLog_UseCommand(cmdTable, "\r\n# ")) return;
    pifLog_Print(LT_NONE, "\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
    ENABLE_ARMING_FLAG(PREVENT_ARMING);
}

static int cliExit(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    pifLog_Print(LT_NONE, "\r\nLeaving CLI mode, unsaved changes lost.\r\n");
    
    cliMode = 0;
    // incase a motor was left running during motortest, clear it here
    mixerResetDisarmedMotors();
    cliReboot();

    return PIF_LOG_CMD_NO_ERROR;
}

static int cliFeature(int argc, char *argv[])
{
    uint32_t i;
    uint32_t mask;

    mask = featureMask();

    if (argc == 0) {
        pifLog_Print(LT_NONE, "Enabled: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                pifLog_Printf(LT_NONE, "%s ", featureNames[i]);
        }
        pifLog_Print(LT_NONE, "\r\n");
    } else if (strcasecmp(argv[0], "list") == 0) {
        pifLog_Print(LT_NONE, "Available: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            pifLog_Printf(LT_NONE, "%s ", featureNames[i]);
        }
        pifLog_Print(LT_NONE, "\r\n");
    } else {
        bool remove = false;
        if (argv[0][0] == '-') {
            // remove feature
            remove = true;
            argv[0]++; // skip over -
        }

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                pifLog_Print(LT_NONE, "Invalid name\r\n");
                break;
            }

            if (strcasecmp(argv[0], featureNames[i]) == 0) {

                mask = 1 << i;
#ifndef GPS
                if (mask & FEATURE_GPS) {
                    pifLog_Print(LT_NONE, "unavailable\r\n");
                    break;
                }
#endif
#ifndef SONAR
                if (mask & FEATURE_SONAR) {
                    pifLog_Print(LT_NONE, "unavailable\r\n");
                    break;
                }
#endif
                if (remove) {
                    featureClear(mask);
                    pifLog_Print(LT_NONE, "Disabled");
                } else {
                    featureSet(mask);
                    pifLog_Print(LT_NONE, "Enabled");
                }
                pifLog_Printf(LT_NONE, " %s\r\n", featureNames[i]);
                break;
            }
        }
    }
    return PIF_LOG_CMD_NO_ERROR;
}

#ifdef GPS
static int cliGpsPassthrough(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    gpsEnablePassthrough(cliPort);
    return PIF_LOG_CMD_NO_ERROR;
}
#endif

static int cliMap(int argc, char *argv[])
{
    uint32_t len;
    uint32_t i;
    char out[9];

    UNUSED(argc);

    len = strlen(argv[0]);

    if (len == 8) {
        // uppercase it
        for (i = 0; i < 8; i++)
            argv[0][i] = toupper((unsigned char)argv[0][i]);
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, argv[0][i]) && !strchr(argv[0] + i + 1, argv[0][i]))
                continue;
            cliShowParseError();
            return CLI_ERROR_PARSE;
        }
        parseRcChannels(argv[0], &masterConfig.rxConfig);
    }
    pifLog_Print(LT_NONE, "Map: ");
    for (i = 0; i < 8; i++)
        out[masterConfig.rxConfig.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    pifLog_Printf(LT_NONE, "%s\r\n", out);
    return PIF_LOG_CMD_NO_ERROR;
}

#ifndef USE_QUAD_MIXER_ONLY
static int cliMixer(int argc, char *argv[])
{
    int i;

    if (argc == 0) {
        pifLog_Printf(LT_NONE, "Mixer: %s\r\n", mixerNames[masterConfig.mixerMode - 1]);
        return PIF_LOG_CMD_NO_ERROR;
    } else if (strcasecmp(argv[0], "list") == 0) {
        pifLog_Print(LT_NONE, "Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            pifLog_Printf(LT_NONE, "%s ", mixerNames[i]);
        }
        pifLog_Print(LT_NONE, "\r\n");
        return PIF_LOG_CMD_NO_ERROR;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            pifLog_Print(LT_NONE, "Invalid name\r\n");
            return CLI_ERROR_INVALID;
        }
        if (strcasecmp(argv[0], mixerNames[i]) == 0) {
            masterConfig.mixerMode = i + 1;
            break;
        }
    }

    cliMixer(0, NULL);
    return PIF_LOG_CMD_NO_ERROR;
}
#endif

static int cliMotor(int argc, char *argv[])
{
    int motor_index = 0;
    int motor_value = 0;

    if (argc == 0) {
        cliShowParseError();
        return CLI_ERROR_PARSE;
    }

    motor_index = atoi(argv[0]);

    if (motor_index < 0 || motor_index >= MAX_SUPPORTED_MOTORS) {
        cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS - 1);
        return CLI_ERROR_OUT_OF_RANGE;
    }

    if (argc == 2) {
        motor_value = atoi(argv[1]);

        if (motor_value < PWM_RANGE_MIN || motor_value > PWM_RANGE_MAX) {
            cliShowArgumentRangeError("value", 1000, 2000);
            return CLI_ERROR_OUT_OF_RANGE;
        } else {
            motor_disarmed[motor_index] = motor_value;
        }
    }

    pifLog_Printf(LT_NONE, "motor %d: %d\r\n", motor_index, motor_disarmed[motor_index]);
    return PIF_LOG_CMD_NO_ERROR;
}

static int cliPlaySound(int argc, char *argv[])
{
#if FLASH_SIZE <= 64
    UNUSED(argc);
    UNUSED(argv);
#else
    int i;
    const char *name;
    static int lastSoundIdx = -1;

    if (argc == 0) {
        i = lastSoundIdx + 1;     //next sound index
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            while (true) {   //no name for index; try next one
                if (++i >= beeperTableEntryCount())
                    i = 0;   //if end then wrap around to first entry
                if ((name=beeperNameForTableIndex(i)) != NULL)
                    break;   //if name OK then play sound below
                if (i == lastSoundIdx + 1) {     //prevent infinite loop
                    pifLog_Print(LT_NONE, "Error playing sound\r\n");
                    return CLI_ERROR_INVALID;
                }
            }
        }
    } else {       //index value was given
        i = atoi(argv[0]);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            pifLog_Printf(LT_NONE, "No sound for index %d\r\n", i);
            return CLI_ERROR_INVALID;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    pifLog_Printf(LT_NONE, "Playing sound %d: %s\r\n", i, name);
    beeper(beeperModeForTableIndex(i));
#endif
    return PIF_LOG_CMD_NO_ERROR;
}

static int cliProfile(int argc, char *argv[])
{
    int i;

    if (argc == 0) {
        pifLog_Printf(LT_NONE, "profile %d\r\n", getCurrentProfile());
    } else {
        i = atoi(argv[0]);
        if (i >= 0 && i < MAX_PROFILE_COUNT) {
            masterConfig.current_profile_index = i;
            writeEEPROM();
            readEEPROM();
            cliProfile(0, NULL);
        }
    }
    return PIF_LOG_CMD_NO_ERROR;
}

static int cliRateProfile(int argc, char *argv[])
{
    int i;

    if (argc == 0) {
        pifLog_Printf(LT_NONE, "rateprofile %d\r\n", getCurrentControlRateProfile());
    } else {
        i = atoi(argv[0]);
        if (i >= 0 && i < MAX_CONTROL_RATE_PROFILE_COUNT) {
            changeControlRateProfile(i);
            cliRateProfile(0, NULL);
        }
    }
    return PIF_LOG_CMD_NO_ERROR;
}

static void cliReboot(void) {
    pifLog_Print(LT_NONE, "\r\nRebooting");
    pifLog_SendAndExit();
    stopMotors();
    handleOneshotFeatureChangeOnRestart();
    systemReset();
}

static int cliSave(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    pifLog_Print(LT_NONE, "Saving");
    //copyCurrentProfileToProfileSlot(masterConfig.current_profile_index);
    writeEEPROM();
    cliReboot();
    return PIF_LOG_CMD_NO_ERROR;
}

static int cliDefaults(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    pifLog_Print(LT_NONE, "Resetting to defaults");
    resetEEPROM();
    cliReboot();
    return PIF_LOG_CMD_NO_ERROR;
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[10];

    void *ptr = var->ptr;
    if ((var->type & VALUE_SECTION_MASK) == PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }
    if ((var->type & VALUE_SECTION_MASK) == CONTROL_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

    switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            value = *(uint8_t *)ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)ptr;
            break;

        case VAR_FLOAT:
            pifLog_Print(LT_NONE, ftoa(*(float *)ptr, buf));
            if (full && (var->type & VALUE_MODE_MASK) == MODE_DIRECT) {
                pifLog_Printf(LT_NONE, " %s", ftoa((float)var->config.minmax.min, buf));
                pifLog_Printf(LT_NONE, " %s", ftoa((float)var->config.minmax.max, buf));
            }
            return; // return from case for float only
    }

    switch(var->type & VALUE_MODE_MASK) {
        case MODE_DIRECT:
            pifLog_Printf(LT_NONE, "%d", value);
            if (full) {
                pifLog_Printf(LT_NONE, " %d %d", var->config.minmax.min, var->config.minmax.max);
            }
            break;
        case MODE_LOOKUP:
            pifLog_Print(LT_NONE, lookupTables[var->config.lookup.tableIndex].values[value]);
            break;
    }
}

static void cliSetVar(const clivalue_t *var, const int_float_value_t value)
{
    void *ptr = var->ptr;
    if ((var->type & VALUE_SECTION_MASK) == PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }
    if ((var->type & VALUE_SECTION_MASK) == CONTROL_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

    switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
        case VAR_INT8:
            *(int8_t *)ptr = value.int_value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(int16_t *)ptr = value.int_value;
            break;

        case VAR_UINT32:
            *(uint32_t *)ptr = value.int_value;
            break;

        case VAR_FLOAT:
            *(float *)ptr = (float)value.float_value;
            break;
    }
}

static int cliSet(int argc, char *argv[])
{
    uint32_t i;
    const clivalue_t *val;
    int rtn = PIF_LOG_CMD_NO_ERROR;

    if (argc == 0 || (argc == 1 && argv[0][0] == '*')) {
        pifLog_Print(LT_NONE, "Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            pifLog_Printf(LT_NONE, "%s = ", valueTable[i].name);
            cliPrintVar(val, argc); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            pifLog_Print(LT_NONE, "\r\n");
        }
    } else if (argc > 2 && argv[1][0] == '=') {
        // has equals

        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            // ensure exact match when setting to prevent setting variables with shorter names
            if (strncasecmp(argv[0], valueTable[i].name, strlen(valueTable[i].name)) == 0) {

                bool changeValue = false;
                int_float_value_t tmp;
                switch (valueTable[i].type & VALUE_MODE_MASK) {
                    case MODE_DIRECT: {
                            int32_t value = 0;
                            float valuef = 0;

                            value = atoi(argv[2]);
                            valuef = fastA2F(argv[2]);

                            if (valuef >= valueTable[i].config.minmax.min && valuef <= valueTable[i].config.minmax.max) { // note: compare float value

                                if ((valueTable[i].type & VALUE_TYPE_MASK) == VAR_FLOAT)
                                    tmp.float_value = valuef;
                                else
                                    tmp.int_value = value;

                                changeValue = true;
                            }
                        }
                        break;
                    case MODE_LOOKUP: {
                            const lookupTableEntry_t *tableEntry = &lookupTables[valueTable[i].config.lookup.tableIndex];
                            bool matched = false;
                            for (uint8_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                                matched = strcasecmp(tableEntry->values[tableValueIndex], argv[2]) == 0;

                                if (matched) {
                                    tmp.int_value = tableValueIndex;
                                    changeValue = true;
                                }
                            }
                        }
                        break;
                }

                if (changeValue) {
                    cliSetVar(val, tmp);

                    pifLog_Printf(LT_NONE, "%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    pifLog_Print(LT_NONE, "Invalid value\r\n");
                }

                return PIF_LOG_CMD_NO_ERROR;
            }
        }
        pifLog_Print(LT_NONE, "Invalid name\r\n");
        rtn = CLI_ERROR_INVALID;
    } else {
        // no equals, check for matching variables.
        cliGet(argc, argv);
    }
    return rtn;
}

static int cliGet(int argc, char *argv[])
{
    uint32_t i;
    const clivalue_t *val;
    int matchedCommands = 0;

    for (i = 0; i < VALUE_COUNT; i++) {
        if (argc == 0 || strstr(valueTable[i].name, argv[0])) {
            val = &valueTable[i];
            pifLog_Printf(LT_NONE, "%s = ", valueTable[i].name);
            cliPrintVar(val, 0);
            pifLog_Print(LT_NONE, "\r\n");

            matchedCommands++;
        }
    }


    if (matchedCommands) {
    	return PIF_LOG_CMD_NO_ERROR;
    }

    pifLog_Print(LT_NONE, "Invalid name\r\n");
    return CLI_ERROR_INVALID;
}

static int cliStatus(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    pifLog_Printf(LT_NONE, "System Uptime: %d seconds, Voltage: %d * 0.1V (%dS battery - %s), System load: %d.%02d\r\n",
        millis() / 1000,
        vbat,
        batteryCellCount,
        getBatteryStateString(),
        pif_performance._use_rate / 100,
        pif_performance._use_rate % 100
    );

    pifLog_Printf(LT_NONE, "CPU Clock=%dMHz", (SystemCoreClock / 1000000));

    uint8_t i;
    uint32_t mask;
    uint32_t detectedSensorsMask = sensorsMask();

    for (i = 0; ; i++) {

        if (sensorTypeNames[i] == NULL)
            break;

        mask = (1 << i);
        if ((detectedSensorsMask & mask)) {
            if (mask & SENSOR_NAMES_MASK) {
                const char *sensorHardware;
                switch (mask) {
                case SENSOR_GYRO:
                    sensorHardware = sensor_link.gyro.hw_name;
                    break;

                case SENSOR_ACC:
                    sensorHardware = sensor_link.acc.hw_name;
                    break;

                case SENSOR_MAG:
                    sensorHardware = sensor_link.mag.hw_name;
                    break;

                case SENSOR_BARO:
                    sensorHardware = sensor_link.baro.hw_name;
                    break;
                }

                pifLog_Printf(LT_NONE, ", %s=%s", sensorTypeNames[i], sensorHardware);

                if (mask == SENSOR_ACC && sensor_link.acc.revisionCode) {
                    pifLog_Printf(LT_NONE, ".%c", sensor_link.acc.revisionCode);
                }
            }
            else {
                pifLog_Printf(LT_NONE, ", %s", sensorTypeNames[i]);
            }
        }
    }
    pifLog_Print(LT_NONE, "\r\n");

#ifdef USE_I2C
    uint16_t i2cErrorCounter = i2cGetErrorCounter();
#else
    uint16_t i2cErrorCounter = 0;
#endif

    pifLog_Printf(LT_NONE, "Cycle Time: %d, I2C Errors: %d, config size: %d\r\n", cycleTime, i2cErrorCounter, sizeof(master_t));
    pifLog_Printf(LT_NONE, "Task Count=%d, Timer Count=%d\r\n", pifTaskManager_Count(), pifTimerManager_Count(&g_timer_1ms));
    pifLog_Printf(LT_NONE, "gyro_sync=%d temp=%d.%2d DegC\r\n", sensor_link.gyro.can_sync, (int)sensor_link.baro.temperature, (int)((sensor_link.baro.temperature - (int)sensor_link.baro.temperature) * 100));
   	return PIF_LOG_CMD_NO_ERROR;
}

#ifndef SKIP_TASK_STATISTICS
static int cliTasks(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    cfTaskId_e taskId;
    PifTask* p_task;

    pifLog_Print(LT_NONE, "Task list:\r\n");
    for (taskId = 0; taskId < TASK_COUNT; taskId++) {
        p_task = cfTasks[taskId].p_task;
        if (cfTasks[taskId].isCreate && p_task->_total_execution_time) {
            pifLog_Printf(LT_NONE, "%d - %s, mode = %d, max = %d us, avg = %d us total = %d ms", taskId, cfTasks[taskId].taskName, p_task->_mode,
                    p_task->_max_execution_time, p_task->_total_execution_time / p_task->_execution_count, p_task->_total_execution_time / 1000);
            if (p_task->_total_period_time) {
                pifLog_Printf(LT_NONE, ", period = %d us", p_task->_total_period_time / p_task->_period_count);
            }
            if (p_task->_total_trigger_delay) {
                pifLog_Printf(LT_NONE, ", delay: max = %d acg = %d us", p_task->_max_trigger_delay, p_task->_total_trigger_delay / p_task->_execution_count);
            }
            pifLog_Print(LT_NONE, "\r\n");
        }
        else {
            pifLog_Printf(LT_NONE, "%d - %s, mode = %d, create = %d\r\n", taskId, cfTasks[taskId].taskName, p_task->_mode, cfTasks[taskId].isCreate);
        }
    }
   	return PIF_LOG_CMD_NO_ERROR;
}
#endif

static int cliVersion(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    pifLog_Printf(LT_NONE, "# Cleanflight/%s %s %s / %s (%s)",
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
   	return PIF_LOG_CMD_NO_ERROR;
}

void cliInit(serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
}
#endif
