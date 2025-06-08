/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef _IMU_BMI270_H_
#define _IMU_BMI270_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "bmi270.h"
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
} bmi270_value_t;
typedef struct {
    float pitch;
    float yaw;
    float roll;
} bmi270_axis_t;
void app_imu_init(i2c_bus_handle_t i2c_bus_handle,gpio_num_t  imu_int_pin);
uint64_t imu_interrupt_wake_Init();
#ifdef __cplusplus
}
#endif

#endif
