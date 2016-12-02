/*
 * led.c
 *
 *  Created on: 2016/12/02
 *      Author: strv
 */

#include "led.h"
#include "stm32f3xx_hal.h"
#include "gpio.h"

GPIO_InitTypeDef GPIO_InitStruct = {
		0,
		GPIO_MODE_OUTPUT_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_LOW,
		0
};

void led_init(){
	GPIO_InitStruct.Pin = LED1_Pin;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin , GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = LED2_Pin;
	HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin , GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = LED3_Pin;
	HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin , GPIO_PIN_RESET);
}

void led_on(LED led){
	if(led & LED1){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin , GPIO_PIN_SET);
	}
	if(led & LED2){
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin , GPIO_PIN_SET);
	}
	if(led & LED3){
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin , GPIO_PIN_SET);
	}
}

void led_off(LED led){
	if(led & LED1){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin , GPIO_PIN_RESET);
	}
	if(led & LED2){
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin , GPIO_PIN_RESET);
	}
	if(led & LED3){
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin , GPIO_PIN_RESET);
	}
}
