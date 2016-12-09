/*
 * pwm.c
 *
 *  Created on: 2016/12/02
 *      Author: strv
 */

#include "pwm.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"

#define setCCR_1A(val) PWM1_TIM->CCR2 = val
#define setCCR_1B(val) PWM1_TIM->CCR3 = val
#define setCCR_2A(val) PWM2_TIM->CCR2 = val
#define setCCR_2B(val) PWM2_TIM->CCR3 = val

static const int32_t Pwm1_ccr_max = PWM1_Period * PWM_DUTY_MAX / 100;
static const int32_t Pwm2_ccr_max = PWM2_Period * PWM_DUTY_MAX / 100;
static const int32_t Pwm1_ccr_center = PWM1_Period * PWM_DUTY_MAX / 100 / 2;
static const int32_t Pwm2_ccr_center = PWM2_Period * PWM_DUTY_MAX / 100 / 2;
static const int32_t Vcmd_interval = 10;
static int32_t vcmd_interval_cnt[2] = {};
static int32_t next_ccr[2] = {};
static int32_t pres_vb;
static int32_t vcmd_target[2] = {};
static int32_t vcmd_tick[2] = {0, 0};

void pwm_enable(void){
	pwm_set_duty(PWM1, 0);
	pwm_set_duty(PWM2, 0);
	pres_vb = adc_vbatt();
	tim_start();
	HAL_Delay(10);
	HAL_GPIO_WritePin(MD_EN1_GPIO_Port, MD_EN1_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_EN2_GPIO_Port, MD_EN2_Pin , GPIO_PIN_SET);
}

void pwm_disable(void){
	HAL_GPIO_WritePin(MD_EN1_GPIO_Port, MD_EN1_Pin , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD_EN2_GPIO_Port, MD_EN2_Pin , GPIO_PIN_RESET);
}

void pwm_set_duty(PWM_CH ch, int32_t percent){
	if(percent > PWM_DUTY_MAX){
		percent = PWM_DUTY_MAX;
	}else if(percent < -PWM_DUTY_MAX){
		percent = -PWM_DUTY_MAX;
	}

	if(ch & PWM1){
		next_ccr[0] = percent * PWM1_Period / 100 / 2;
		setCCR_1A( next_ccr[0] + PWM1_Period / 2);
		setCCR_1B(-next_ccr[0] + PWM1_Period / 2);
	}
	if(ch & PWM2){
		next_ccr[1] = percent * PWM2_Period / 100 / 2;
		setCCR_2A(-next_ccr[1] + PWM2_Period / 2);
		setCCR_2B( next_ccr[1] + PWM2_Period / 2);
	}
}

void pwm_set_mv(PWM_CH ch, int32_t mv){
	if(ch & PWM1){
		vcmd_target[0] = mv;
	}
	if(ch & PWM2){
		vcmd_target[1] = mv;
	}
}

void pwm_set_mode(PWM_CH ch, PWM_MODE mode){
	if(ch & PWM1){
		if(mode == PWM_DUTY){
			vcmd_tick[0] = 0;
		}else if(mode == PWM_VCMD){
			vcmd_tick[0] = 1;
		}
	}
	if(ch & PWM2){
		if(mode == PWM_DUTY){
			vcmd_tick[1] = 0;
		}else if(mode == PWM_VCMD){
			vcmd_tick[1] = 1;
		}
	}
}

void PWM1_IRQ_Handler(void){
	if(PWM1_TIM->SR & TIM_SR_UIF_Msk){
		vcmd_interval_cnt[0] += vcmd_tick[0];
		if(vcmd_interval_cnt[0] >= Vcmd_interval){
			vcmd_interval_cnt[0] = 0;
			pres_vb = adc_vbatt();
			setCCR_1A( vcmd_target[0] * Pwm1_ccr_center / pres_vb + Pwm1_ccr_center);
			setCCR_1B(-vcmd_target[0] * Pwm1_ccr_center / pres_vb + Pwm1_ccr_center);
		}
		PWM1_TIM->SR &= ~(TIM_SR_UIF_Msk);
	}
}

void PWM2_IRQ_Handler(void){
	if(PWM2_TIM->SR & TIM_SR_UIF_Msk){
		vcmd_interval_cnt[1] += vcmd_tick[1];
		if(vcmd_interval_cnt[1] >= Vcmd_interval){
			vcmd_interval_cnt[1] = 0;
			pres_vb = adc_vbatt();
			setCCR_2A(-vcmd_target[1] * Pwm1_ccr_center / pres_vb + Pwm1_ccr_center);
			setCCR_2B( vcmd_target[1] * Pwm1_ccr_center / pres_vb + Pwm1_ccr_center);
		}
		PWM2_TIM->SR &= ~(TIM_SR_UIF_Msk);
	}
}
