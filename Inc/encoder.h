/*
 * encoder.h
 *
 *  Created on: 2016/12/04
 *      Author: strv
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

#define ENC1_TIM TIM4
#define	ENC2_TIM TIM3

typedef enum{
	DIR_FWD,
	DIR_REV
}DIR;

typedef enum{
	MD_CH1 = 1 << 0,
	MD_CH2 = 1 << 1
}MD_CH;

void encoder_init(DIR enc1_dir, DIR enc2_dir);
uint16_t encoder_get(MD_CH ch);

#endif /* ENCODER_H_ */
