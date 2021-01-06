/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <am_bsp.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "application.h"

#define APPLICATION_LED_PIN          AM_BSP_GPIO_LED1
#define APPLICATION_LED_TIMER        2
#define APPLICATION_LED_TIMER_SOURCE AM_HAL_CTIMER_LFRC_32HZ
#define APPLICATION_LED_TIMER_SEG    AM_HAL_CTIMER_TIMERA
#define APPLICATION_LED_TIMER_INT    AM_HAL_CTIMER_INT_TIMERA2C0
#define APPLICATION_LED_PERIOD       32 * 3
#define APPLICATION_LED_ON_TIME      2

TaskHandle_t application_task_handle;

void application_task(void *pvParameters)
{
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);

    am_hal_ctimer_output_config(
        APPLICATION_LED_TIMER, APPLICATION_LED_TIMER_SEG, APPLICATION_LED_PIN,
        AM_HAL_CTIMER_OUTPUT_NORMAL, AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);

    am_hal_ctimer_config_single(
        APPLICATION_LED_TIMER, APPLICATION_LED_TIMER_SEG,
        (AM_HAL_CTIMER_FN_PWM_REPEAT | APPLICATION_LED_TIMER_SOURCE |
         AM_HAL_CTIMER_INT_ENABLE));

    am_hal_ctimer_period_set(APPLICATION_LED_TIMER, APPLICATION_LED_TIMER_SEG,
                             APPLICATION_LED_PERIOD,
                             APPLICATION_LED_PERIOD - APPLICATION_LED_ON_TIME);

    am_hal_ctimer_int_enable(APPLICATION_LED_TIMER_INT);
    NVIC_EnableIRQ(CTIMER_IRQn);

    am_hal_ctimer_start(APPLICATION_LED_TIMER, APPLICATION_LED_TIMER_SEG);
    while (1) {
        taskYIELD();
    }
}
