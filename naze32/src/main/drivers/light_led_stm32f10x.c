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

#include "core/pif.h"

#include "common/utils.h"

#include "system.h"
#include "gpio.h"

#include "light_led.h"


void ledInit(void)
{
#if defined(LED0) || defined(LED1) || defined(LED2)
    uint32_t i;

    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] = {
#ifdef LED0
        {
            .gpio = LED0_GPIO,
            .cfg = { LED0_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
#ifdef LED1
        {
            .gpio = LED1_GPIO,
            .cfg = { LED1_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
#ifdef LED2
        {
            .gpio = LED2_GPIO,
            .cfg = { LED2_PIN, Mode_Out_PP, Speed_2MHz }
        }
#endif
    };

    uint8_t gpio_count = ARRAYLEN(gpio_setup);

#ifdef LED0
    RCC_APB2PeriphClockCmd(LED0_PERIPHERAL, ENABLE);
#endif
#ifdef LED1
    RCC_APB2PeriphClockCmd(LED1_PERIPHERAL, ENABLE);
#endif
#ifdef LED2
    RCC_APB2PeriphClockCmd(LED2_PERIPHERAL, ENABLE);
#endif

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;

    for (i = 0; i < gpio_count; i++) {
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }

#endif
}

void pif_ChangeStatusLed(int num, BOOL state)
{
    switch (num) {
#ifdef LED0
    case 0:
        if (state) LED0_ON; else LED0_OFF;
        break;
#endif
#ifdef LED1
    case 1:
        if (state) LED1_ON; else LED1_OFF;
        break;
#endif
#ifdef LED2
    case 2: 
        if (state) LED2_ON; else LED2_OFF; 
        break;
#endif
    default:
        break;
    }
}

void pif_ToggleStatusLed(int num)
{
    switch (num) {
#ifdef LED0
    case 0:
        LED0_TOGGLE;
        break;
#endif
#ifdef LED1
    case 1:
        LED1_TOGGLE;
        break;
#endif
#ifdef LED2
    case 2:
        LED2_TOGGLE;
        break;
#endif
    default:
        break;
    }
}
