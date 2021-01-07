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
#include "bmi270.h"

#include "imu.h"

TaskHandle_t  imu_task_handle;
QueueHandle_t imu_task_queue;

static int8_t set_feature_config(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Configure the type of sensor. */
    config.type = BMI2_STEP_COUNTER;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(&config, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Setting water-mark level to 1 for step counter to get interrupt after 20 step counts. Every 20 steps once
         * output triggers. */
        config.cfg.step_counter.watermark_level = 1;

        /* Set new configuration. */
        rslt = bmi270_set_sensor_config(&config, 1, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}


void imu_task(void *pvParameters)
{
    am_util_stdio_printf("\n\rBMI270 sensor task started\n\r");
    nm_console_print_prompt();

    imu_task_queue = xQueueCreate(10, sizeof(task_message_t));

    struct bmi2_dev bmi2_dev;
    struct bmi2_sensor_data sensor_data;
    int8_t rslt;
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_STEP_COUNTER };
    uint16_t int_status = 0;
    struct bmi2_sens_int_config sens_int = { .type = BMI2_STEP_COUNTER, .hw_int_pin = BMI2_INT2 };

    sensor_data.type = BMI2_STEP_COUNTER;

    rslt = bmi2_interface_init(&bmi2_dev, BMI2_SPI_INTF);
    bmi2_error_codes_print_result(rslt);

    rslt = bmi270_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    rslt = bmi270_sensor_enable(sensor_sel, 2, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Set the feature configuration for step counter. */
    rslt = set_feature_config(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Map the step counter feature interrupt. */
    rslt = bmi270_map_feat_int(&sens_int, 1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    uint32_t total_steps = 0;
    while (1) {
        rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* To check the interrupt status of the step counter. */
        if (int_status & BMI270_STEP_CNT_STATUS_MASK)
        {
            /* Get step counter output. */
            rslt = bmi270_get_sensor_data(&sensor_data, 1, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);
            total_steps = sensor_data.sens_data.step_counter_output;
        }
        vTaskDelay(5000);
    }
}
