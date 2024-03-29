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

#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "sensor/pif_hmc5883.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "nvic.h"
#include "gpio.h"
#include "bus_i2c.h"
#include "light_led.h"

#include "sensors/sensors.h"

#include "compass_hmc5883l.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

//#define DEBUG_MAG_DATA_READY_INTERRUPT

// HMC5883L, default address 0x1E
// NAZE Target connections
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Normal
 *              0  |  1   |  Positive Bias
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  �0.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  �1.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  �1.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  �2.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  �4.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  �4.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  �5.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  �8.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Continuous-Conversion Mode.
 *              0  |  1   |  Single-Conversion Mode
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Sleep Mode
 */

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.

const char* hmc5883_name = "HMC5883";

static PifHmc5883 hmc5883;

static float magGain[3] = { 1.0f, 1.0f, 1.0f };

#ifdef NAZE
static const hmc5883Config_t nazeHmc5883Config_v1_v4 = {
        .gpioAPB2Peripherals = RCC_APB2Periph_GPIOB,
        .gpioPin = Pin_12,
        .gpioPort = GPIOB,

        /* Disabled for v4 needs more work.
        .exti_port_source = GPIO_PortSourceGPIOB,
        .exti_pin_source = GPIO_PinSource12,
        .exti_line = EXTI_Line12,
        .exti_irqn = EXTI15_10_IRQn
        */
};
static const hmc5883Config_t nazeHmc5883Config_v5 = {
        .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
        .gpioPin = Pin_14,
        .gpioPort = GPIOC,
        .exti_port_source = GPIO_PortSourceGPIOC,
        .exti_line = EXTI_Line14,
        .exti_pin_source = GPIO_PinSource14,
        .exti_irqn = EXTI15_10_IRQn
};
#endif

static const hmc5883Config_t *hmc5883Config = NULL;

void hmc5883lInit(void* p_param);

static bool hmc5883lRead(float* p_mag)
{
    return pifImuSensor_ReadRawMag(&sensor_link.imu_sensor, p_mag);
}

void MAG_DATA_READY_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(hmc5883Config->exti_line) == RESET) {
        return;
    }

    EXTI_ClearITPendingBit(hmc5883Config->exti_line);

#ifdef DEBUG_MAG_DATA_READY_INTERRUPT
    // Measure the delta between calls to the interrupt handler
    // currently should be around 65/66 milli seconds / 15hz output rate
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;

    uint32_t now = millis();
    callDelta = now - lastCalledAt;

    //UNUSED(callDelta);
    debug[0] = callDelta;

    lastCalledAt = now;
#endif
}

static void hmc5883lConfigureDataReadyInterruptHandling(void)
{
#ifdef USE_MAG_DATA_READY_SIGNAL

    if (!(hmc5883Config->exti_port_source && hmc5883Config->exti_pin_source)) {
        return;
    }
#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(hmc5883Config->exti_port_source, hmc5883Config->exti_pin_source);
#endif

#ifdef ENSURE_MAG_DATA_READY_IS_HIGH
    uint8_t status = GPIO_ReadInputDataBit(hmc5883Config->gpioPort, hmc5883Config->gpioPin);
    if (!status) {
        return;
    }
#endif

    registerExtiCallbackHandler(hmc5883Config->exti_irqn, MAG_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(hmc5883Config->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = hmc5883Config->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = hmc5883Config->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MAG_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_MAG_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

bool hmc5883lDetect(void* p_param)
{
    UNUSED(p_param);

#ifdef NAZE
    if (hardwareRevision < NAZE32_REV5) {
        hmc5883Config = &nazeHmc5883Config_v1_v4;
    } else {
        hmc5883Config = &nazeHmc5883Config_v5;
    }
#endif    

    if (!pifHmc5883_Detect(&g_i2c_port)) return false;

    sensor_link.mag.hw_name = hmc5883_name;
    sensor_link.mag.init = hmc5883lInit;
    sensor_link.mag.read = hmc5883lRead;

    return true;
}

void hmc5883lInit(void* p_param)
{
    int16_t magADC[3];
    int i;
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    bool bret = true;           // Error indicator
    PifHmc5883ConfigA config_a;

    (void)p_param;

    gpio_config_t gpio;

    if (hmc5883Config) {
#ifdef STM32F10X
        if (hmc5883Config->gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(hmc5883Config->gpioAPB2Peripherals, ENABLE);
        }
#endif
        gpio.pin = hmc5883Config->gpioPin;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(hmc5883Config->gpioPort, &gpio);
    }

    if (!pifHmc5883_Init(&hmc5883, PIF_ID_AUTO, &g_i2c_port, &sensor_link.imu_sensor)) return;

    delay(50);
    config_a.byte = 0;
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_POS_BIAS;
    config_a.bit.data_rate = HMC5883_DATARATE_15HZ;
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte);   // Reg A DOR = 0x010 + MS1, MS0 set to pos bias
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.
    pifHmc5883_SetGain(&hmc5883, HMC5883_GAIN_2_5GA); // Set the Gain to 2.5Ga (7:5->011)
    delay(100);
    pifHmc5883_ReadMag(&hmc5883, magADC);

    for (i = 0; i < 10; i++) {  // Collect 10 samples
        pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_SINGLE);
        delay(50);
        pifHmc5883_ReadMag(&hmc5883, magADC);       // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged rather than taking the max.
        xyz_total[X] += magADC[X];
        xyz_total[Y] += magADC[Y];
        xyz_total[Z] += magADC[Z];

        // Detect saturation.
        if (-4096 >= MIN(magADC[X], MIN(magADC[Y], magADC[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        LED1_TOGGLE;
    }

    // Apply the negative bias. (Same gain)
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_NEG_BIAS;
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte);   // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
    for (i = 0; i < 10; i++) {
        pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_SINGLE);
        delay(50);
        pifHmc5883_ReadMag(&hmc5883, magADC);               // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged.
        xyz_total[X] -= magADC[X];
        xyz_total[Y] -= magADC[Y];
        xyz_total[Z] -= magADC[Z];

        // Detect saturation.
        if (-4096 >= MIN(magADC[X], MIN(magADC[Y], magADC[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        LED1_TOGGLE;
    }

    magGain[X] = fabsf(660.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[X]);
    magGain[Y] = fabsf(660.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Y]);
    magGain[Z] = fabsf(660.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Z]);

    // leave test mode
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_NORMAL;
    config_a.bit.samples = HMC5883_SAMPLES_8;
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte);         // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    pifHmc5883_SetGain(&hmc5883, HMC5883_GAIN_1_3GA);                                       // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_CONTINOUS);    // Mode register             -- 000000 00    continuous Conversion Mode
    delay(100);

    if (!bret) {                // Something went wrong so get a best guess
        magGain[X] = 1.0f;
        magGain[Y] = 1.0f;
        magGain[Z] = 1.0f;
    }

    hmc5883lConfigureDataReadyInterruptHandling();
}
