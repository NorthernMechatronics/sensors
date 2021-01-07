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

#include "console_task.h"
#include "gas.h"
#include "application.h"
#include "task_message.h"

#define APPLICATION_ADC_BATTERY_SLOT     5
#define APPLICATION_ADC_TEMPERATURE_SLOT 7

#define APPLICATION_LED_PIN          AM_BSP_GPIO_LED1
#define APPLICATION_LED_TIMER        2
#define APPLICATION_LED_TIMER_SOURCE AM_HAL_CTIMER_LFRC_32HZ
#define APPLICATION_LED_TIMER_SEG    AM_HAL_CTIMER_TIMERA
#define APPLICATION_LED_TIMER_INT    AM_HAL_CTIMER_INT_TIMERA2C0
#define APPLICATION_LED_PERIOD       32 * 5
#define APPLICATION_LED_ON_TIME      2

TaskHandle_t  application_task_handle;
QueueHandle_t application_task_queue;

static void *   adc_handle;
static uint16_t adc_battery_code;
static uint16_t adc_temperature_code;

static float battery_voltage;
static float temperature;

const static float reference_voltage = 1.5f;

const static am_hal_adc_config_t adc_cfg = {
    .eClock     = AM_HAL_ADC_CLKSEL_HFRC_DIV2,
    .ePolarity  = AM_HAL_ADC_TRIGPOL_RISING,
    .eTrigger   = AM_HAL_ADC_TRIGSEL_SOFTWARE,
    .eReference = AM_HAL_ADC_REFSEL_INT_1P5,
    .eClockMode = AM_HAL_ADC_CLKMODE_LOW_POWER,
    .ePowerMode = AM_HAL_ADC_LPMODE1,
    .eRepeat    = AM_HAL_ADC_SINGLE_SCAN};

static void application_button_handler(void);
static void application_timer_isr(void);

static void application_adc_setup()
{
    am_hal_adc_slot_config_t slot_cfg;

    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_initialize(0, &adc_handle)) {
        am_util_stdio_printf(
            "Error - reservation of the ADC instance failed.\n");
    }

    if (AM_HAL_STATUS_SUCCESS !=
        am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_WAKE, false)) {
        am_util_stdio_printf("Error - ADC power on failed.\n");
    }

    if (am_hal_adc_configure(adc_handle, (am_hal_adc_config_t *)&adc_cfg) !=
        AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("Error - configuring ADC failed.\n");
    }

    slot_cfg.bEnabled       = false;
    slot_cfg.bWindowCompare = false;
    slot_cfg.eChannel       = AM_HAL_ADC_SLOT_CHSEL_SE0; // 0
    slot_cfg.eMeasToAvg     = AM_HAL_ADC_SLOT_AVG_1;     // 0
    slot_cfg.ePrecisionMode = AM_HAL_ADC_SLOT_14BIT;     // 0

    am_hal_adc_configure_slot(adc_handle, 0, &slot_cfg); // Unused slot
    am_hal_adc_configure_slot(adc_handle, 1, &slot_cfg); // Unused slot
    am_hal_adc_configure_slot(adc_handle, 2, &slot_cfg); // Unused slot
    am_hal_adc_configure_slot(adc_handle, 3, &slot_cfg); // Unused slot
    am_hal_adc_configure_slot(adc_handle, 4, &slot_cfg); // Unused slot
    am_hal_adc_configure_slot(adc_handle, 6, &slot_cfg); // Unused slot

    // Battery voltage
    slot_cfg.bEnabled       = true;
    slot_cfg.bWindowCompare = true;
    slot_cfg.eChannel       = AM_HAL_ADC_SLOT_CHSEL_BATT;
    slot_cfg.eMeasToAvg     = AM_HAL_ADC_SLOT_AVG_1;
    slot_cfg.ePrecisionMode = AM_HAL_ADC_SLOT_14BIT;
    am_hal_adc_configure_slot(adc_handle, APPLICATION_ADC_BATTERY_SLOT,
                              &slot_cfg);

    // Temperature
    slot_cfg.bEnabled       = true;
    slot_cfg.bWindowCompare = true;
    slot_cfg.eChannel       = AM_HAL_ADC_SLOT_CHSEL_TEMP;
    slot_cfg.eMeasToAvg     = AM_HAL_ADC_SLOT_AVG_1;
    slot_cfg.ePrecisionMode = AM_HAL_ADC_SLOT_10BIT;
    am_hal_adc_configure_slot(adc_handle, APPLICATION_ADC_TEMPERATURE_SLOT,
                              &slot_cfg);

    am_hal_adc_interrupt_enable(adc_handle, AM_HAL_ADC_INT_CNVCMP);
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);

    am_hal_adc_enable(adc_handle);
}

static void application_button_setup()
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_AM_BSP_GPIO_BUTTON0);
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON0, application_button_handler);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    NVIC_EnableIRQ(GPIO_IRQn);
    NVIC_SetPriority(GPIO_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
}

static void application_timer_setup()
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

    am_hal_ctimer_int_register(APPLICATION_LED_TIMER_INT,
                               application_timer_isr);
    am_hal_ctimer_int_clear(APPLICATION_LED_TIMER_INT);
    am_hal_ctimer_int_enable(APPLICATION_LED_TIMER_INT);
    NVIC_EnableIRQ(CTIMER_IRQn);

    am_hal_ctimer_start(APPLICATION_LED_TIMER, APPLICATION_LED_TIMER_SEG);
}

static void application_adc_convert_sample()
{
    float temperature_voltage;

    battery_voltage =
        ((float)adc_battery_code) * 3.0f * reference_voltage / (16384.0f);
    temperature_voltage =
        ((float)adc_temperature_code) * reference_voltage / (1024.0f * 64.0f);

    float vt[] = {temperature_voltage, 0.0f, -123.456};

    if (AM_HAL_STATUS_SUCCESS ==
        am_hal_adc_control(adc_handle, AM_HAL_ADC_REQ_TEMP_CELSIUS_GET, vt)) {
        temperature = vt[1];
    }
}

void application_display_measurement()
{
    am_util_stdio_printf("Battery Voltage:     %.2f V (0x%04X)\n\r",
                         battery_voltage, adc_battery_code);
    am_util_stdio_printf("Device Temperature: %.2f C (0x%04X)\n\r", temperature,
                         adc_temperature_code);
}

void application_report()
{
    am_util_stdio_printf("\n\r\n\r");
    application_display_measurement();
    am_util_stdio_printf("\n\r");
    gas_display_measurement();
    am_util_stdio_printf("\n\r");
    nm_console_print_prompt();
}

void application_task(void *pvParameters)
{
    task_message_t task_message;

    application_task_queue = xQueueCreate(10, sizeof(task_message_t));

    application_adc_setup();
    application_button_setup();
    application_timer_setup();

    while (1) {
        if (xQueueReceive(application_task_queue, &task_message,
                          portMAX_DELAY) == pdPASS) {
            switch (task_message.ui32Event) {
            case APPLICATION_EVENT_ADC_CNVCMP:
                application_adc_convert_sample();
                break;
            case APPLICATION_EVENT_REPORT:
                application_report();
                break;
            }
        }
    }
}

void application_button_handler()
{
    uint64_t ui64Status;

    portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
    task_message_t task_message;

    task_message.ui32Event = APPLICATION_EVENT_REPORT;
    xQueueSendFromISR(application_task_queue, &task_message,
                      &xHigherPriorityTaskWoken);
}

void application_timer_isr() { am_hal_adc_sw_trigger(adc_handle); }

void am_adc_isr()
{
    uint32_t ui32IntStatus;
    am_hal_adc_interrupt_status(adc_handle, &ui32IntStatus, true);
    am_hal_adc_interrupt_clear(adc_handle, ui32IntStatus);

    uint32_t            ui32NumSamples = 1;
    am_hal_adc_sample_t sample;

    //
    // Emtpy the FIFO, we'll just look at the last one read.
    //
    while (AM_HAL_ADC_FIFO_COUNT(ADC->FIFO)) {
        ui32NumSamples = 1;
        am_hal_adc_samples_read(adc_handle, true, NULL, &ui32NumSamples,
                                &sample);

        //
        // Determine which slot it came from?
        //
        if (sample.ui32Slot == APPLICATION_ADC_BATTERY_SLOT) {
            //
            // The returned ADC sample is for the battery voltage divider.
            //
            adc_battery_code = AM_HAL_ADC_FIFO_SAMPLE(sample.ui32Sample);

        } else if (sample.ui32Slot == APPLICATION_ADC_TEMPERATURE_SLOT) {
            //
            // The returned ADC sample is for the temperature sensor.
            // We need the integer part in the low 16-bits.
            //
            adc_temperature_code = sample.ui32Sample & 0xFFC0;
        }
    }

    portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
    task_message_t task_message;

    task_message.ui32Event = APPLICATION_EVENT_ADC_CNVCMP;
    xQueueSendFromISR(application_task_queue, &task_message,
                      &xHigherPriorityTaskWoken);
}
