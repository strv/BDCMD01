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
#define	ENC1_IRQHandler	TIM4_IRQHandler
#define	ENC2_IRQHandler	TIM3_IRQHandler

void encoder_init(DIR enc1_dir, DIR enc2_dir);
int64_t encoder_get(MD_CH ch);
void encoder_proc(void);
void ENC1_IRQHandler(void);
void ENC2_IRQHandler(void);
#endif /* ENCODER_H_ */
