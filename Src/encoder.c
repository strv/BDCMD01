/*
 * encoder.c
 *
 *  Created on: 2016/12/04
 *      Author: strv
 */

#include "encoder.h"
#include "tim.h"

#define getCCR_1() ENC1_TIM->CNT
#define getCCR_2() ENC2_TIM->CNT

static DIR dir1, dir2;

void encoder_init(DIR enc1_dir, DIR enc2_dir){
	enc_start();
	dir1 = enc1_dir;
	dir2 = enc2_dir;
}

uint16_t encoder_get(MD_CH ch){
	uint16_t res = 0;
	switch(ch){
	case MD_CH1:
		res = getCCR_1();
		break;

	case MD_CH2:
		res = getCCR_2();
		break;

	default:
		res = 0;
		break;
	}
	return res;
}
