/*
 * LSM6DS3_Driver.h
 *
 *  Created on: 2016/12/02
 *      Author: strv
 */

#ifndef LSM6DS3_DRIVER_H_
#define LSM6DS3_DRIVER_H_

#include "spi.h"
#include "stm32f3xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>

typedef enum{
	IMU_ACC_2g,
	IMU_ACC_4g,
	IMU_ACC_8g,
	IMU_ACC_16g
}IMU_ACC_RANGE;

typedef	enum{
	IMU_GYRO_125dps,
	IMU_GYRO_245dps,
	IMU_GYRO_500dps,
	IMU_GYRO_1000dps,
	IMU_GYRO_2000dps
}IMU_GYRO_RANGE;

typedef enum{
	IMU_ACC_12_5,
	IMU_ACC_26,
	IMU_ACC_52,
	IMU_ACC_104,
	IMU_ACC_208,
	IMU_ACC_416,
	IMU_ACC_833,
	IMU_ACC_1666,
	IMU_ACC_3332,
	IMU_ACC_6664,
}IMU_ACC_RATE;

typedef enum{
	IMU_GYRO_12_5,
	IMU_GYRO_26,
	IMU_GYRO_52,
	IMU_GYRO_104,
	IMU_GYRO_208,
	IMU_GYRO_416,
	IMU_GYRO_833,
	IMU_GYRO_1666,
}IMU_GYRO_RATE;

void IMU_init(void);
void IMU_set_acc_range(IMU_ACC_RANGE range);
void IMU_set_gyro_range(IMU_GYRO_RANGE range);
void IMU_set_acc_rate(IMU_ACC_RATE rate);
void IMU_set_gyro_rate(IMU_GYRO_RATE rate);
int32_t IMU_get_temp(void);
void IMU_get_acc(int32_t* px, int32_t* py, int32_t* pz);
void IMU_get_gyro(int32_t* pr, int32_t* pp, int32_t* py);
void IMU_reflesh(void);
void IMU_spi_tx_cb(SPI_HandleTypeDef *hspi);
void IMU_spi_txrx_cb(SPI_HandleTypeDef *hspi);

#endif /* LSM6DS3_DRIVER_H_ */
