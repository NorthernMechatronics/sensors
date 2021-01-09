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

#include <console_task.h>
#include <task_message.h>

#include "bme68x.h"
#include "nm_devices_bme68x.h"

#include "gas.h"

#define GAS_CLOCK_SOURCE   AM_HAL_CTIMER_LFRC_32HZ
#define GAS_TIMER_PERIOD   32
#define GAS_MEASURE_PERIOD GAS_TIMER_PERIOD * 6
#define GAS_TIMER_SEGMENT  AM_HAL_CTIMER_TIMERA
#define GAS_TIMER_NUMBER   0
#define GAS_TIMER_INT      AM_HAL_CTIMER_INT_TIMERA0

TaskHandle_t  gas_task_handle;
QueueHandle_t gas_task_queue;

static struct bme68x_dev        bme;
static struct bme68x_conf       conf;
static struct bme68x_heatr_conf heatr_conf;

static float temperature;
static float pressure;
static float humidity;
static float gas_resistance;

static void gas_timer_isr()
{
    portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
    task_message_t task_message;

    am_hal_ctimer_int_clear(GAS_TIMER_INT);

    task_message.ui32Event = GAS_EVENT_MEASURE;
    xQueueSendFromISR(gas_task_queue, &task_message, &xHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void gas_timer_setup()
{
    am_hal_ctimer_config_t GASTimer = {
        0,
        (AM_HAL_CTIMER_FN_REPEAT | AM_HAL_CTIMER_INT_ENABLE | GAS_CLOCK_SOURCE),
        0,
    };

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
    am_hal_ctimer_clear(GAS_TIMER_NUMBER, GAS_TIMER_SEGMENT);
    am_hal_ctimer_config(GAS_TIMER_NUMBER, &GASTimer);
    am_hal_ctimer_period_set(GAS_TIMER_NUMBER, GAS_TIMER_SEGMENT,
                             GAS_MEASURE_PERIOD, (GAS_MEASURE_PERIOD >> 1));

    am_hal_ctimer_int_register(GAS_TIMER_INT, gas_timer_isr);
    am_hal_ctimer_int_clear(GAS_TIMER_INT);
    am_hal_ctimer_int_enable(GAS_TIMER_INT);

    NVIC_EnableIRQ(CTIMER_IRQn);
    NVIC_SetPriority(CTIMER_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);

    am_hal_ctimer_start(GAS_TIMER_NUMBER, GAS_TIMER_SEGMENT);
}

static void gas_sensor_setup()
{
    int8_t rslt;

    /* Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     * For SPI : BME68X_SPI_INTF
     */
    rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
    bme68x_check_rslt("bme68x_interface_init", rslt);

    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter  = BME68X_FILTER_OFF;
    conf.odr     = BME68X_ODR_NONE;
    conf.os_hum  = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt         = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable     = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur  = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
}

static void gas_measure()
{
    int8_t             rslt;
    struct bme68x_data data;
    uint8_t            n_fields;

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    uint16_t del_period =
        bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf) + heatr_conf.heatr_dur;

    // Yield to other tasks while measurement is being made
    //bme.delay_us(del_period * 1000, bme.intf_ptr);
    vTaskDelay(del_period);

    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    bme68x_check_rslt("bme68x_get_data", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    if (n_fields && data.status != 0xa0) {
        temperature    = data.temperature;
        pressure       = data.pressure;
        humidity       = data.humidity;
        gas_resistance = data.gas_resistance;

        if (gas_resistance < 16000) {
            am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_CLEAR);
        } else {
            am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_SET);
        }
    }
}

void gas_display_measurement()
{
    am_util_stdio_printf("Temperature:    %.2f C\n\r"
                         "Pressure:       %.2f kPa\n\r"
                         "Humidity:       %.2f %%\n\r"
                         "Gas Resistance: %.2f kOhm\n\r",
                         temperature, pressure / 1000, humidity,
                         gas_resistance / 1000);
}

void gas_task(void *pvParameters)
{
    task_message_t task_message;

    am_util_stdio_printf("\n\rBME680 sensor task started\n\r\n\r");
    nm_console_print_prompt();

    gas_task_queue = xQueueCreate(10, sizeof(task_message_t));

    gas_sensor_setup();
    gas_timer_setup();

    while (1) {
        if (xQueueReceive(gas_task_queue, &task_message, portMAX_DELAY) ==
            pdPASS) {
            switch (task_message.ui32Event) {
            case GAS_EVENT_MEASURE:
                gas_measure();
                break;
            }
        }
    }
}
