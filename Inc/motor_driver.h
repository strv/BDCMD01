/*
 * motor_driver.h
 *
 *  Created on: 2016/12/12
 *      Author: strv
 */

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

typedef enum{
	MD_CH1 = 1 << 0,
	MD_CH2 = 1 << 1,
	MD_CH12 = MD_CH1 | MD_CH2,
	MD_CH_MAX
}MD_CH;

typedef enum{
	DIR_FWD,
	DIR_REV
}DIR;

#endif /* MOTOR_DRIVER_H_ */
