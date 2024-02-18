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

#include <platform.h>
#include "build_config.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"

#include "sonar_hcsr04.h"

#include "sensor/pif_hc_sr04.h"

#include "sensors/sensors.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

#if defined(SONAR)
const char* hcsr04_name = "HC_SR04";

static sonarHardware_t const *sonarHardware;
static PifHcSr04 s_hcsr04;

#if !defined(UNIT_TEST)
static void ECHO_EXTI_IRQHandler(void)
{
    if (digitalIn(sonarHardware->echo_gpio, sonarHardware->echo_pin) != 0) {
    	pifHcSr04_sigReceiveEcho(&s_hcsr04, 1);
    } else {
    	pifHcSr04_sigReceiveEcho(&s_hcsr04, 0);
    }

    EXTI_ClearITPendingBit(sonarHardware->exti_line);
}

void EXTI0_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif

static void _actHcSr04Trigger(SWITCH state)
{
    if (state) {
        digitalHi(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
    }
    else {
        digitalLo(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
    }
}

static void _evtHcSr04Distance(int32_t distance)
{
    sensor_link.sonar.distance = distance;

    float temp = sensor_link.baro.temperature;
    static float pretemp = 0;

    if ((int)temp != (int)pretemp) {
        pifHcSr04_SetTemperature(&s_hcsr04, temp);
#ifdef __PIF_DEBUG__
        pifLog_Printf(LT_INFO, "Temp=%f", temp);
#endif
        pretemp = temp;
    }

    if (sensor_link.p_task_altitude) {
        if (!sensor_link.p_task_altitude->_running) pifTask_SetTrigger(sensor_link.p_task_altitude);
    }
}

void hcsr04_init(void* p_param)
{
    const sonarHardware_t *initialSonarHardware = (const sonarHardware_t *)p_param;

	if (!pifHcSr04_Init(&s_hcsr04, PIF_ID_AUTO)) return;
	s_hcsr04.act_trigger = _actHcSr04Trigger;
	s_hcsr04.evt_read = _evtHcSr04Distance;
	if (!pifHcSr04_StartTrigger(&s_hcsr04, sensor_link.sonar.period)) return;
    sensor_link.sonar.hw_name = hcsr04_name;
    sensor_link.sonar.p_task = s_hcsr04._p_task;

    sonarHardware = initialSonarHardware;
    sensor_link.sonar.maxRangeCm = HCSR04_MAX_RANGE_CM;
    sensor_link.sonar.detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    sensor_link.sonar.detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;

#if !defined(UNIT_TEST)
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

    // trigger pin
    gpio.pin = sonarHardware->trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(sonarHardware->trigger_gpio, &gpio);

    // echo pin
    gpio.pin = sonarHardware->echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(sonarHardware->echo_gpio, &gpio);

#ifdef STM32F10X
    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, sonarHardware->exti_pin_source);
#endif

    EXTI_ClearITPendingBit(sonarHardware->exti_line);

    EXTIInit.EXTI_Line = sonarHardware->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = sonarHardware->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

#endif
