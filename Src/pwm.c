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

int32_t next_ccr[2];

void pwm_enable(void){
	pwm_set_duty(PWM1, 0);
	pwm_set_duty(PWM2, 0);
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
	}
	if(ch & PWM2){
		next_ccr[1] = percent * PWM2_Period / 100 / 2;
	}
}

void pwm_set_mv(PWM_CH ch, int32_t mv){

}

void PWM1_IRQ_Handler(void){
	if(PWM1_TIM->SR & TIM_SR_UIF_Msk){
		PWM1_TIM->SR &= ~(TIM_SR_UIF_Msk);
		setCCR_1A( next_ccr[0] + PWM1_Period / 2);
		setCCR_1B(-next_ccr[0] + PWM1_Period / 2);
	}
}

void PWM2_IRQ_Handler(void){
	if(PWM2_TIM->SR & TIM_SR_UIF_Msk){
		PWM2_TIM->SR &= ~(TIM_SR_UIF_Msk);
		setCCR_2A(-next_ccr[1] + PWM2_Period / 2);
		setCCR_2B( next_ccr[1] + PWM2_Period / 2);

	}
}
