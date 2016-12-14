/*
 * encoder.c
 *
 *  Created on: 2016/12/04
 *      Author: strv
 */

#include "encoder.h"
#include "tim.h"
#include <stdint.h>

#define getCNT_1() ENC1_TIM->CNT
#define getCNT_2() ENC2_TIM->CNT

static const int64_t enc_irq_tick = 0xFFFF + 1;
static int32_t dir[2] = {1 ,1};
static int64_t enc_total[2] = {};

void encoder_init(DIR enc1_dir, DIR enc2_dir){
	ENC1_TIM->SR &= ~TIM_SR_UIF;
	ENC2_TIM->SR &= ~TIM_SR_UIF;
	enc_start();
	dir[0] = (enc1_dir == DIR_FWD) ? 1: -1;
	dir[1] = (enc2_dir == DIR_FWD) ? 1: -1;
}

int64_t encoder_get(MD_CH ch){
	switch(ch){
	case MD_CH1:
		return ((int64_t)getCNT_1() + enc_total[0]) * dir[0];

	case MD_CH2:
		return ((int64_t)getCNT_2() + enc_total[1]) * dir[1];

	default:
		return 0;
		break;
	}
}

void ENC1_IRQHandler(void){
	if(ENC1_TIM->SR & TIM_SR_UIF){
		if(ENC1_TIM->CR1 & TIM_CR1_DIR){
			enc_total[0] -= enc_irq_tick;
		}else{
			enc_total[0] += enc_irq_tick;
		}
		ENC1_TIM->SR &= ~TIM_SR_UIF;
	}
}

void ENC2_IRQHandler(void){
	if(ENC2_TIM->SR & TIM_SR_UIF){
		if(ENC2_TIM->CR1 & TIM_CR1_DIR){
			enc_total[1] -= enc_irq_tick;
		}else{
			enc_total[1] += enc_irq_tick;
		}
		ENC2_TIM->SR &= ~TIM_SR_UIF;
	}
}
