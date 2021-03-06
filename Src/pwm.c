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
static const int32_t Vcmd_interval = 2;
static int32_t vcmd_interval_cnt[2] = {};
static int32_t next_ccr[2] = {};
static int32_t pres_vb;
static int32_t vcmd_target[2] = {};
static int32_t vcmd_tick[2] = {0, 0};
static bool inited = false;

void pwm_enable(void){
	pwm_set_duty(MD_CH12, 0);
	pres_vb = adc_vbatt();
	tim_start();
	HAL_Delay(10);
	HAL_GPIO_WritePin(MD_EN1_GPIO_Port, MD_EN1_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_EN2_GPIO_Port, MD_EN2_Pin , GPIO_PIN_SET);

	inited = true;
}

void pwm_disable(void){
	inited = false;

	HAL_GPIO_WritePin(MD_EN1_GPIO_Port, MD_EN1_Pin , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD_EN2_GPIO_Port, MD_EN2_Pin , GPIO_PIN_RESET);
}

void pwm_set_duty(MD_CH ch, int32_t percent){
	if(percent > PWM_DUTY_MAX){
		percent = PWM_DUTY_MAX;
	}else if(percent < -PWM_DUTY_MAX){
		percent = -PWM_DUTY_MAX;
	}

	if(ch & MD_CH1){
		next_ccr[0] = percent * PWM1_Period / 100;
		if(next_ccr[0] > 0){
			setCCR_1A(next_ccr[0]);
			setCCR_1B(0);
		}else{
			setCCR_1A(0);
			setCCR_1B(-next_ccr[0]);
		}
	}
	if(ch & MD_CH2){
		next_ccr[1] = percent * PWM2_Period / 100;
		if(next_ccr[1] > 0){
			setCCR_2A(0);
			setCCR_2B(next_ccr[1]);
		}else{
			setCCR_2A(-next_ccr[1]);
			setCCR_2B(0);
		}
	}
}

void pwm_set_mv(MD_CH ch, int32_t mv){
	if(ch & MD_CH1){
		vcmd_target[0] = mv;
	}
	if(ch & MD_CH2){
		vcmd_target[1] = mv;
	}
}

void pwm_set_mode(MD_CH ch, PWM_MODE mode){
	if(ch & MD_CH1){
		if(mode == PWM_DUTY){
			vcmd_tick[0] = 0;
		}else if(mode == PWM_VCMD){
			vcmd_tick[0] = 1;
		}
	}
	if(ch & MD_CH2){
		if(mode == PWM_DUTY){
			vcmd_tick[1] = 0;
		}else if(mode == PWM_VCMD){
			vcmd_tick[1] = 1;
		}
	}
}

void pwm_update_vb(void){
	if(!inited){
		return;
	}
	pres_vb = adc_vbatt();
}

static int32_t ccr_tmp1 = 0;
void PWM1_IRQ_Handler(void){
	if(PWM1_TIM->SR & TIM_SR_UIF_Msk){
		vcmd_interval_cnt[0] += vcmd_tick[0];
		if(vcmd_interval_cnt[0] >= Vcmd_interval){
			vcmd_interval_cnt[0] = 0;
			ccr_tmp1 = vcmd_target[0] * (PWM1_Period - PWM1_DT_OFST) / pres_vb;
			if(ccr_tmp1 < 0){
				ccr_tmp1 = -ccr_tmp1;
			}
			ccr_tmp1 += PWM1_DT_OFST;
			if(ccr_tmp1 > Pwm1_ccr_max){
				ccr_tmp1 = Pwm1_ccr_max;
			}
			if(vcmd_target[0] > 0){
				setCCR_1A(ccr_tmp1);
				setCCR_1B(0);
			}else if(vcmd_target[0] < 0){
				setCCR_1A(0);
				setCCR_1B(ccr_tmp1);
			}else{
				setCCR_1A(0);
				setCCR_1B(0);
			}
		}
		PWM1_TIM->SR &= ~(TIM_SR_UIF_Msk);
	}
}

static int32_t ccr_tmp2 = 0;
void PWM2_IRQ_Handler(void){
	if(PWM2_TIM->SR & TIM_SR_UIF_Msk){
		vcmd_interval_cnt[1] += vcmd_tick[1];
		if(vcmd_interval_cnt[1] >= Vcmd_interval){
			vcmd_interval_cnt[1] = 0;
			ccr_tmp2 = vcmd_target[1] * (PWM2_Period - PWM2_DT_OFST) / pres_vb;
			if(ccr_tmp2 < 0){
				ccr_tmp2 = -ccr_tmp2;
			}
			ccr_tmp2 += PWM2_DT_OFST;
			if(ccr_tmp2 > Pwm2_ccr_max){
				ccr_tmp2 = Pwm2_ccr_max;
			}
			if(vcmd_target[1] > 0){
				setCCR_2A(0);
				setCCR_2B(ccr_tmp2);
			}else if(vcmd_target[1] < 0){
				setCCR_2A(ccr_tmp2);
				setCCR_2B(0);
			}else{
				setCCR_2A(0);
				setCCR_2B(0);
			}
		}
		PWM2_TIM->SR &= ~(TIM_SR_UIF_Msk);
	}
}
