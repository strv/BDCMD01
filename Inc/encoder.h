/*
 * encoder.h
 *
 *  Created on: 2016/12/04
 *      Author: strv
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>
#include "motor_driver.h"

#define ENC1_TIM TIM4
#define	ENC2_TIM TIM3

typedef enum{
	DIR_FWD,
	DIR_REV
}DIR;

void encoder_init(DIR enc1_dir, DIR enc2_dir);
uint16_t encoder_get(MD_CH ch);

#endif /* ENCODER_H_ */
