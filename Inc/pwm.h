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

#define PWM_DUTY_MAX (95)
#define	PWM1_TIM TIM8
#define	PWM2_TIM TIM1
#define PWM1_IRQ_Handler TIM8_UP_IRQHandler
#define PWM2_IRQ_Handler TIM1_UP_TIM16_IRQHandler

typedef enum{
	PWM1 = 1 << 0,
	PWM2 = 1 << 1,
	PWM_MAX
}PWM_CH;

void pwm_enable(void);
void pwm_disable(void);
void pwm_set_duty(PWM_CH ch, int32_t percent);
void pwm_set_mv(PWM_CH ch, int32_t mv);

#endif /* PWM_H_ */
