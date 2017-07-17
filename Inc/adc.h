/**
  ******************************************************************************
  * File Name          : ADC.h
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

/* USER CODE BEGIN Private defines */
#define ADC_BUF_LEN (48)
#define ADC1_CH_NUM	(2)
#define ADC2_CH_NUM	(1)
#define ADC3_CH_NUM	(1)
#define ADC4_CH_NUM	(1)
#define ADC_Q (8)
//#define	ADC_CUR_GAIN	(45)	// [mV]/[A]	for ACS711 31A
#define	ADC_CUR_GAIN	(90)	// [mV]/[A] for ACS711 15A


typedef enum{
	ADC_TEMP,
	ADC_CUR1,
	ADC_CUR2,
	ADC_VB,
	ADC_REF1,
	ADC_REF2,
	ADC_REF3,
	ADC_REF4,
	ADC_MAX
}ADC_CH;
/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);
void MX_ADC4_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef adc_start_cal(void);
void adc_start(void);
uint32_t adc_get(ADC_CH ch);
int32_t adc_vbatt(void);
int32_t adc_cur1(void);
int32_t adc_cur2(void);
void adc_cur_cal(void);
void adc_cur_cal_start(void);
void adc_cur_cal_stop(void);
int32_t adc_cur_offset_delta(int32_t ch, int32_t delta);
int32_t adc_cur_offset_get(int32_t ch);
void adc_cur_offset_set(int32_t ch, int32_t offset);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
