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

#define setCCR_1A(val) TIM8->CCR2 = val
#define setCCR_1B(val) TIM8->CCR3 = val
#define setCCR_2A(val) TIM1->CCR2 = val
#define setCCR_2B(val) TIM1->CCR3 = val

void pwm_enable(void){
	tim_start();
	HAL_GPIO_WritePin(MD_EN1_GPIO_Port, MD_EN1_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_EN2_GPIO_Port, MD_EN2_Pin , GPIO_PIN_SET);
}

void pwm_disable(void){
	HAL_GPIO_WritePin(MD_EN1_GPIO_Port, MD_EN1_Pin , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD_EN2_GPIO_Port, MD_EN2_Pin , GPIO_PIN_RESET);
}

void pwm_set_duty(PWM_CH ch, int32_t percent){
	int32_t ccr = 0;
	if(percent > PWM_DUTY_MAX){
		percent = PWM_DUTY_MAX;
	}else if(percent < -PWM_DUTY_MAX){
		percent = -PWM_DUTY_MAX;
	}

	if(ch & PWM1){
		ccr = percent * PWM1_Period / 100 / 2;
		setCCR_1A( ccr + PWM1_Period / 2);
		setCCR_1B(-ccr + PWM1_Period / 2);
	}
	if(ch & PWM2){
		ccr = percent * PWM2_Period / 100 / 2;
		setCCR_2A(-ccr + PWM2_Period / 2);
		setCCR_2B( ccr + PWM2_Period / 2);
	}
}

void pwm_set_mv(PWM_CH ch, int32_t mv){

}
