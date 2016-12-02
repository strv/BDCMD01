/*
 * led.h
 *
 *  Created on: 2016/12/02
 *      Author: strv
 */

#ifndef LED_H_
#define LED_H_

typedef enum{
	LED1 = 1 << 0,
	LED2 = 1 << 1,
	LED3 = 1 << 2,
	LED_MAX
}LED;

void led_init();
void led_on(LED led);
void led_off(LED led);

#endif /* LED_H_ */
