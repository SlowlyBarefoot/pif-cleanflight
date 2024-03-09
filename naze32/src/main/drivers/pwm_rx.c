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
#include "debug.h"

#include "common/utils.h"

#include "system.h"
#include "scheduler.h"

#include "nvic.h"
#include "gpio.h"
#include "timer.h"

#include "pwm_mapping.h"

#include "pwm_rx.h"

#include "rc/pif_rc_ppm.h"

#define PPM_CAPTURE_COUNT 12
#define PWM_INPUT_PORT_COUNT 8

#if PPM_CAPTURE_COUNT > PWM_INPUT_PORT_COUNT
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PPM_CAPTURE_COUNT
#else
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PWM_INPUT_PORT_COUNT
#endif

// TODO - change to timer clocks ticks
#define INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX 0x03

static inputFilteringMode_e inputFilteringMode;

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

typedef enum {
    INPUT_MODE_PPM,
    INPUT_MODE_PWM,
} pwmInputMode_t;

typedef struct {
    pwmInputMode_t mode;
    uint8_t channel; // only used for pwm, ignored by ppm

    uint8_t state;
    captureCompare_t rise;
    captureCompare_t fall;
    captureCompare_t capture;

    uint8_t missedEvents;

    const timerHardware_t *timerHardware;
    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;
} pwmInputPort_t;

static pwmInputPort_t pwmInputPorts[PWM_INPUT_PORT_COUNT];

static uint16_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];

#define PPM_TIMER_PERIOD 0x10000
#define PWM_TIMER_PERIOD 0x10000

static bool ppmComplete = false;
static uint8_t ppmCountShift = 0;

static PifRcPpm s_rc_ppm;


#define PPM_IN_MIN_SYNC_PULSE_US    2700    // microseconds
#define PPM_IN_MIN_CHANNEL_PULSE_US 750     // microseconds
#define PPM_IN_MAX_CHANNEL_PULSE_US 2250    // microseconds
#define PPM_STABLE_FRAMES_REQUIRED_COUNT    25
#define PPM_IN_MIN_NUM_CHANNELS     4
#define PPM_IN_MAX_NUM_CHANNELS     PWM_PORTS_OR_PPM_CAPTURE_COUNT


bool isPPMDataBeingReceived(void)
{
    return ppmComplete;
}

void resetPPMDataReceivedState(void)
{
    ppmComplete = false;
}

#define MIN_CHANNELS_BEFORE_PPM_FRAME_CONSIDERED_VALID 4

void pwmRxInit(inputFilteringMode_e initialInputFilteringMode)
{
    inputFilteringMode = initialInputFilteringMode;
}

static void _evtRcPpmReceive(PifRc* p_owner, uint16_t* p_channel, PifIssuerP p_issuer)
{
	int i;
//	PifTask* p_task = (PifTask*)p_issuer;

    (void)p_issuer;

	for (i = 0; i < p_owner->_channel_count; i++) {
		captures[i] = p_channel[i];
	}
    ppmComplete = true;
//	pifTask_SetTrigger(p_task);
}

static bool ppmInit(void)
{
    if (!pifRcPpm_Init(&s_rc_ppm, PIF_ID_AUTO, PPM_IN_MAX_NUM_CHANNELS, PPM_IN_MIN_SYNC_PULSE_US)) return false;
    pifRcPpm_SetValidRange(&s_rc_ppm, PPM_IN_MIN_CHANNEL_PULSE_US, PPM_IN_MAX_CHANNEL_PULSE_US);
    pifRc_AttachEvtReceive(&s_rc_ppm.parent, _evtRcPpmReceive, cfTasks[TASK_RX].p_task);
    return true;
}

static void ppmOverflowCallback(timerOvrHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    UNUSED(capture);
}

static void ppmEdgeCallback(timerCCHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    UNUSED(capture);

    pifRcPpm_sigTick(&s_rc_ppm, (*pif_act_timer1us)());
}

#define MAX_MISSED_PWM_EVENTS 10

bool isPWMDataBeingReceived(void)
{
    int channel;
    for (channel = 0; channel < PWM_PORTS_OR_PPM_CAPTURE_COUNT; channel++) {
        if (captures[channel] != PPM_RCVR_TIMEOUT) {
            return true;
        }
    }
    return false;
}

static void pwmOverflowCallback(timerOvrHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, overflowCb);

    if (++pwmInputPort->missedEvents > MAX_MISSED_PWM_EVENTS) {
        captures[pwmInputPort->channel] = PPM_RCVR_TIMEOUT;
        pwmInputPort->missedEvents = 0;
    }
}

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, edgeCb);
    const timerHardware_t *timerHardwarePtr = pwmInputPort->timerHardware;

    if (pwmInputPort->state == 0) {
        pwmInputPort->rise = capture;
        pwmInputPort->state = 1;
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Falling);
    } else {
        pwmInputPort->fall = capture;

        // compute and store capture
        pwmInputPort->capture = pwmInputPort->fall - pwmInputPort->rise;
        captures[pwmInputPort->channel] = pwmInputPort->capture;

        // switch state
        pwmInputPort->state = 0;
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);
        pwmInputPort->missedEvents = 0;
    }
}

static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;

    if (inputFilteringMode == INPUT_FILTERING_ENABLED) {
        TIM_ICInitStructure.TIM_ICFilter = INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX;
    } else {
        TIM_ICInitStructure.TIM_ICFilter = 0x00;
    }

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

void pwmInConfig(const timerHardware_t *timerHardwarePtr, uint8_t channel)
{
    pwmInputPort_t *self = &pwmInputPorts[channel];

    self->state = 0;
    self->missedEvents = 0;
    self->channel = channel;
    self->mode = INPUT_MODE_PWM;
    self->timerHardware = timerHardwarePtr;

    pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->gpioInputMode);
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);

    timerConfigure(timerHardwarePtr, (uint16_t)PWM_TIMER_PERIOD, PWM_TIMER_MHZ);

    timerChCCHandlerInit(&self->edgeCb, pwmEdgeCallback);
    timerChOvrHandlerInit(&self->overflowCb, pwmOverflowCallback);
    timerChConfigCallbacks(timerHardwarePtr, &self->edgeCb, &self->overflowCb);
}

#define UNUSED_PPM_TIMER_REFERENCE 0
#define FIRST_PWM_PORT 0

void ppmAvoidPWMTimerClash(const timerHardware_t *timerHardwarePtr, TIM_TypeDef *sharedPwmTimer)
{
    if (timerHardwarePtr->tim == sharedPwmTimer) {
        ppmCountShift = 3;  // Divide by 8 if the timer is running at 8 MHz
    }
}

void ppmInConfig(const timerHardware_t *timerHardwarePtr)
{
    ppmInit();

    pwmInputPort_t *self = &pwmInputPorts[FIRST_PWM_PORT];

    self->mode = INPUT_MODE_PPM;
    self->timerHardware = timerHardwarePtr;

    pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->gpioInputMode);
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);

    timerConfigure(timerHardwarePtr, (uint16_t)PPM_TIMER_PERIOD, PWM_TIMER_MHZ);

    timerChCCHandlerInit(&self->edgeCb, ppmEdgeCallback);
    timerChOvrHandlerInit(&self->overflowCb, ppmOverflowCallback);
    timerChConfigCallbacks(timerHardwarePtr, &self->edgeCb, &self->overflowCb);
}

uint16_t ppmRead(uint8_t channel)
{
    return captures[channel];
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
