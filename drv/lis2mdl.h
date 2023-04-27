#ifndef __LIS2MDL_H
#define __LIS2MDL_H

/**
 * @file lis2mdl.h
 * @addtogroup sailbot
 *
 * @brief  Module for controlling the IMU on the SailBot.
 *
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <nrf.h>

typedef struct {
    int16_t x;  ///< X axis
    int16_t y;  ///< Y axis
    int16_t z;  ///< Z axis
} lis2mdl_compass_data_t;

typedef struct {
    int16_t x;  ///< X axis
    int16_t y;  ///< Y axis
    int16_t z;  ///< Z axis
} lsm6ds_acc_data_t;

typedef void (*lis2mdl_data_ready_cb_t)(void);  ///< Callback function prototype, it is called on each available sample
typedef void (*lsm6ds_data_ready_cb_t)(void);   ///< Callback function prototype, it is called on each available sample

void  imu9d_init(lsm6ds_data_ready_cb_t accelerometer_callback, lis2mdl_data_ready_cb_t magnetometer_callback);
void  lis2mdl_init(lis2mdl_data_ready_cb_t callback);
bool  lis2mdl_data_ready(void);
void  lis2mdl_read_heading(void);
float lis2mdl_last_heading(void);
void  lis2mdl_magnetometer_calibrate(lis2mdl_compass_data_t *offset);

void   lsm6ds_init(lsm6ds_data_ready_cb_t callback);
bool   lsm6ds_data_ready(void);
void   lsm6ds_read_accelerometer(void);
int8_t lsm6ds_last_roll(void);

#endif
