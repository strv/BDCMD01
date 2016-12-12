/*
 * pwm.h
 *
 *  Created on: 2016/12/02
 *      Author: strv
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "stm32f3xx_hal.h"
#include "motor_driver.h"

#define PWM_DUTY_MAX (95)
#define	PWM1_TIM TIM1
#define	PWM2_TIM TIM8
#define PWM1_IRQ_Handler TIM1_UP_TIM16_IRQHandler
#define PWM2_IRQ_Handler TIM8_UP_IRQHandler

typedef enum{
	PWM_DUTY,
	PWM_VCMD,
	PWM_MODE_MAX
}PWM_MODE;

void pwm_enable(void);
void pwm_disable(void);
void pwm_set_duty(MD_CH ch, int32_t percent);
void pwm_set_mv(MD_CH ch, int32_t mv);
void pwm_set_mode(MD_CH ch, PWM_MODE mode);
void PWM1_IRQ_Handler(void);
void PWM2_IRQ_Handler(void);
#endif /* PWM_H_ */
