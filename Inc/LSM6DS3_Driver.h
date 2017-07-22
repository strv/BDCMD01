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
	IMU_ACC_2g	= 0x0,
	IMU_ACC_4g	= 0x1,
	IMU_ACC_8g	= 0x2,
	IMU_ACC_16g	= 0x3
}IMU_ACC_RANGE;

typedef	enum{
	IMU_GYRO_125dps		= 0x1,
	IMU_GYRO_245dps		= 0x0,
	IMU_GYRO_500dps		= 0x2,
	IMU_GYRO_1000dps	= 0x4,
	IMU_GYRO_2000dps	= 0x6
}IMU_GYRO_RANGE;

typedef enum{
	IMU_ACC_12_5	= 0x1,
	IMU_ACC_26		= 0x2,
	IMU_ACC_52		= 0x3,
	IMU_ACC_104		= 0x4,
	IMU_ACC_208		= 0x5,
	IMU_ACC_416		= 0x6,
	IMU_ACC_833		= 0x7,
	IMU_ACC_1666	= 0x8,
	IMU_ACC_3332	= 0x9,
	IMU_ACC_6664	= 0xA
}IMU_ACC_RATE;

typedef enum{
	IMU_GYRO_12_5	= 0x1,
	IMU_GYRO_26		= 0x2,
	IMU_GYRO_52		= 0x3,
	IMU_GYRO_104	= 0x4,
	IMU_GYRO_208	= 0x5,
	IMU_GYRO_416	= 0x6,
	IMU_GYRO_833	= 0x7,
	IMU_GYRO_1666	= 0x8
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
void IMU_pole(void);
void IMU_spi_tx_cb(SPI_HandleTypeDef *hspi);
void IMU_spi_txrx_cb(SPI_HandleTypeDef *hspi);

#endif /* LSM6DS3_DRIVER_H_ */
