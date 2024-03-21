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
#include "build/build_config.h"

#include "system.h"
#include "gpio.h"

#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"

#include "sonar_hcsr04.h"

#ifdef SONAR_PIF
	#include "sensor/pif_hc_sr04.h"
#endif

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

#if defined(SONAR)
STATIC_UNIT_TESTED volatile int32_t measurement = -1;
static uint32_t lastMeasurementAt;
static sonarHardware_t const *sonarHardware;

extiCallbackRec_t hcsr04_extiCallbackRec;
static IO_t echoIO;
//static IO_t triggerIO;

#ifdef SONAR_PIF
	static PifHcSr04 s_hcsr04;
#endif

void hcsr04_extiHandler(extiCallbackRec_t* cb)
{
#ifndef SONAR_PIF
    static uint32_t timing_start;
    uint32_t timing_stop;
    UNUSED(cb);

    if (digitalIn(sonarHardware->echo_gpio, sonarHardware->echo_pin) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            measurement = timing_stop - timing_start;
        }
    }
#else
    pifHcSr04_sigReceiveEcho(&s_hcsr04, digitalIn(sonarHardware->echo_gpio, sonarHardware->echo_pin));
#endif
}

#ifdef SONAR_PIF

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
    measurement = distance; 
}

#endif

bool hcsr04_init(const sonarHardware_t *initialSonarHardware, sonarRange_t *sonarRange)
{
    sonarHardware = initialSonarHardware;
    sonarRange->maxRangeCm = HCSR04_MAX_RANGE_CM;
    sonarRange->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    sonarRange->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;

#if !defined(UNIT_TEST)
    gpio_config_t gpio;

#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#endif

#ifdef SONAR_PIF
	if (!pifHcSr04_Init(&s_hcsr04, PIF_ID_AUTO)) return false;
	s_hcsr04.act_trigger = _actHcSr04Trigger;
	s_hcsr04.evt_read = _evtHcSr04Distance;
	if (!pifHcSr04_StartTrigger(&s_hcsr04, 50)) return false;       // 50ms
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

    echoIO = IOGetByTag(sonarHardware->echoIO);
    EXTIHandlerInit(&hcsr04_extiCallbackRec, hcsr04_extiHandler);
    EXTIConfig(echoIO, &hcsr04_extiCallbackRec, NVIC_PRIO_SONAR_EXTI, EXTI_Trigger_Rising_Falling); // TODO - priority!
    EXTIEnable(echoIO, true);

    lastMeasurementAt = millis() - 60; // force 1st measurement in hcsr04_get_distance()
#else
    UNUSED(lastMeasurementAt); // to avoid "unused" compiler warning
    UNUSED(echoIO);
#endif
    return true;
}

#ifndef SONAR_PIF

// measurement reading is done asynchronously, using interrupt
void hcsr04_start_reading(void)
{
#if !defined(UNIT_TEST)
    uint32_t now = millis();

    if (now < (lastMeasurementAt + 60)) {
        // the repeat interval of trig signal should be greater than 60ms
        // to avoid interference between connective measurements.
        return;
    }

    lastMeasurementAt = now;

    digitalHi(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
    //  The width of trig signal must be greater than 10us
    delayMicroseconds(11);
    digitalLo(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
#endif
}

#endif

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04_get_distance(void)
{
    // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance traveled.
    //
    // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
#ifndef SONAR_PIF
    int32_t distance = measurement / 59;
#else
    int32_t distance = measurement;
#endif

    return distance;
}
#endif
