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

#include "sensor/pif_qmc5883.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "nvic.h"
#include "gpio.h"
#include "bus_i2c.h"
#include "light_led.h"

#include "sensors/sensors.h"

#include "compass_qmc5883l.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

//#define DEBUG_MAG_DATA_READY_INTERRUPT

// QMC5883L, default address 0x1E
// NAZE Target connections
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

const char* qmc5883_name = "QMC5883";

static PifQmc5883 qmc5883;

#ifdef NAZE
static const qmc5883Config_t nazeQmc5883Config_v1_v4 = {
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
static const qmc5883Config_t nazeQmc5883Config_v5 = {
        .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
        .gpioPin = Pin_14,
        .gpioPort = GPIOC,
        .exti_port_source = GPIO_PortSourceGPIOC,
        .exti_line = EXTI_Line14,
        .exti_pin_source = GPIO_PinSource14,
        .exti_irqn = EXTI15_10_IRQn
};
#endif

static const qmc5883Config_t *qmc5883Config = NULL;

void qmc5883lInit(void* p_param);

static bool qmc5883lRead(int32_t *mag)
{
    float raw[3];

    if (!pifImuSensor_ReadRawMag(&sensor_link.imu_sensor, raw)) return false;
    mag[0] = raw[0];
    mag[1] = raw[1];
    mag[2] = raw[2];
    return true;
}

static void MAG_DATA_READY_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(qmc5883Config->exti_line) == RESET) {
        return;
    }

    EXTI_ClearITPendingBit(qmc5883Config->exti_line);

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

static void qmc5883lConfigureDataReadyInterruptHandling(void)
{
#ifdef USE_MAG_DATA_READY_SIGNAL

    if (!(qmc5883Config->exti_port_source && qmc5883Config->exti_pin_source)) {
        return;
    }
#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(qmc5883Config->exti_port_source, qmc5883Config->exti_pin_source);
#endif

#ifdef ENSURE_MAG_DATA_READY_IS_HIGH
    uint8_t status = GPIO_ReadInputDataBit(qmc5883Config->gpioPort, qmc5883Config->gpioPin);
    if (!status) {
        return;
    }
#endif

    registerExtiCallbackHandler(qmc5883Config->exti_irqn, MAG_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(qmc5883Config->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = qmc5883Config->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = qmc5883Config->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MAG_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_MAG_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

bool qmc5883lDetect(void* p_param)
{
    UNUSED(p_param);

#ifdef NAZE
    if (hardwareRevision < NAZE32_REV5) {
        qmc5883Config = &nazeQmc5883Config_v1_v4;
    } else {
        qmc5883Config = &nazeQmc5883Config_v5;
    }
#endif    

    if (!pifQmc5883_Detect(&g_i2c_port)) return false;

    sensor_link.mag.hw_name = qmc5883_name;
    sensor_link.mag.init = qmc5883lInit;
    sensor_link.mag.read = qmc5883lRead;

    return true;
}

void qmc5883lInit(void* p_param)
{
    PifQmc5883Control1 control_1;

    (void)p_param;

    gpio_config_t gpio;

    if (qmc5883Config) {
#ifdef STM32F10X
        if (qmc5883Config->gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(qmc5883Config->gpioAPB2Peripherals, ENABLE);
        }
#endif
        gpio.pin = qmc5883Config->gpioPin;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(qmc5883Config->gpioPort, &gpio);
    }

    pifQmc5883_Init(&qmc5883, PIF_ID_AUTO, &g_i2c_port, &sensor_link.imu_sensor);

    control_1.bit.mode = QMC5883_MODE_CONTIMUOUS;
    control_1.bit.odr = QMC5883_ODR_10HZ;
    control_1.bit.rng = QMC5883_RNG_2G;
    control_1.bit.osr = QMC5883_OSR_64;
    pifQmc5883_SetControl1(&qmc5883, control_1);

    qmc5883lConfigureDataReadyInterruptHandling();
}
