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
