/**
  ******************************************************************************
  * File Name          : ADC.c
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

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "dma.h"

/* USER CODE BEGIN 0 */
#include "arm_math.h"
#include "dac.h"

static uint16_t adc1[ADC_BUF_LEN * ADC1_CH_NUM];
static uint16_t adc2[ADC_BUF_LEN * ADC2_CH_NUM];
static uint16_t adc3[ADC_BUF_LEN * ADC3_CH_NUM];
static uint16_t adc4[ADC_BUF_LEN * ADC4_CH_NUM];
static int32_t cur_offset[2] = {};
static bool do_cur_long_cal = false;
static int32_t cur_reverse[2] = {1, 1};
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC4 init function */
void MX_ADC4_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;
static uint32_t HAL_RCC_ADC34_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    /* Peripheral DMA init*/
  
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    /* Peripheral DMA init*/
  
    hdma_adc2.Instance = DMA2_Channel3;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_DMA_REMAP_CHANNEL_ENABLE(HAL_REMAPDMA_ADC24_DMA2_CH34);

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC34_CLK_ENABLED++;
    if(HAL_RCC_ADC34_CLK_ENABLED==1){
      __HAL_RCC_ADC34_CLK_ENABLE();
    }

    /* Peripheral DMA init*/
  
    hdma_adc3.Instance = DMA2_Channel5;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc3);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC4)
  {
  /* USER CODE BEGIN ADC4_MspInit 0 */

  /* USER CODE END ADC4_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC34_CLK_ENABLED++;
    if(HAL_RCC_ADC34_CLK_ENABLED==1){
      __HAL_RCC_ADC34_CLK_ENABLE();
    }

    /* Peripheral DMA init*/
  
    hdma_adc4.Instance = DMA2_Channel4;
    hdma_adc4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc4.Init.Mode = DMA_CIRCULAR;
    hdma_adc4.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_DMA_REMAP_CHANNEL_ENABLE(HAL_REMAPDMA_ADC24_DMA2_CH34);

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc4);

  /* USER CODE BEGIN ADC4_MspInit 1 */

  /* USER CODE END ADC4_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC34_CLK_ENABLED--;
    if(HAL_RCC_ADC34_CLK_ENABLED==0){
      __HAL_RCC_ADC34_CLK_DISABLE();
    }

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC4)
  {
  /* USER CODE BEGIN ADC4_MspDeInit 0 */

  /* USER CODE END ADC4_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC34_CLK_ENABLED--;
    if(HAL_RCC_ADC34_CLK_ENABLED==0){
      __HAL_RCC_ADC34_CLK_DISABLE();
    }

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC4_MspDeInit 1 */

  /* USER CODE END ADC4_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef adc_start_cal(void){
	if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)){
		return HAL_ERROR;
	}
	while( HAL_IS_BIT_SET( HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_BUSY_INTERNAL ) );

	if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED)){
		return HAL_ERROR;
	}
	while( HAL_IS_BIT_SET( HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_BUSY_INTERNAL ) );

	if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED)){
		return HAL_ERROR;
	}
	while( HAL_IS_BIT_SET( HAL_ADC_GetState(&hadc3), HAL_ADC_STATE_BUSY_INTERNAL ) );

	if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED)){
		return HAL_ERROR;
	}
	while( HAL_IS_BIT_SET( HAL_ADC_GetState(&hadc4), HAL_ADC_STATE_BUSY_INTERNAL ) );
	return HAL_OK;
}

void adc_start(void){
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1, sizeof(adc1)/sizeof(adc1[0]) );
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2, sizeof(adc2)/sizeof(adc2[0]) );
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3, sizeof(adc3)/sizeof(adc3[0]) );
    HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc4, sizeof(adc4)/sizeof(adc4[0]) );
}

int32_t adc_get(ADC_CH ch){
	uint32_t i;
	uint32_t i_end;
	int32_t ret_val = 0;
	uint16_t* p_buf;
	uint32_t offset = 0;
	uint32_t tick = 1;
	int32_t buf_num = 0;

	switch(ch){
	case ADC_TEMP:
		i_end = ADC_BUF_LEN * ADC1_CH_NUM;
		p_buf = adc1;
		offset = 0;
		tick = ADC1_CH_NUM;
		break;

	case ADC_CUR2:
		i_end = ADC_BUF_LEN * ADC3_CH_NUM;
		p_buf = adc3;
		offset = 0;
		tick = ADC3_CH_NUM;
		break;

	case ADC_CUR1:
		i_end = ADC_BUF_LEN * ADC2_CH_NUM;
		p_buf = adc2;
		offset = 0;
		tick = ADC2_CH_NUM;
		break;

	case ADC_VB:
		i_end = ADC_BUF_LEN * ADC4_CH_NUM;
		p_buf = adc4;
		offset = 0;
		tick = ADC4_CH_NUM * 4;
		break;

	case ADC_REF1:
		i_end = ADC_BUF_LEN * ADC1_CH_NUM;
		p_buf = adc1;
		offset = 1;
		tick = ADC1_CH_NUM;
		break;

	case ADC_REF2:
		i_end = ADC_BUF_LEN * ADC2_CH_NUM;
		p_buf = adc2;
		offset = 1;
		tick = ADC2_CH_NUM;
		break;

	case ADC_REF3:
		i_end = ADC_BUF_LEN * ADC3_CH_NUM;
		p_buf = adc3;
		offset = 1;
		tick = ADC3_CH_NUM;
		break;

	case ADC_REF4:
		i_end = ADC_BUF_LEN * ADC4_CH_NUM;
		p_buf = adc4;
		offset = 1;
		tick = ADC4_CH_NUM;
		break;

	default:
		return 0;
	}

	for(i = offset; i < i_end; i += tick){
		ret_val += *(p_buf + i);
		buf_num++;
	}
	ret_val = (ret_val << ADC_Q) / buf_num;
/*
	for(i = offset; i < i_end; i += tick){
		ret_val += *(p_buf + i) << ADC_Q;
		buf_num++;
	}
	dac_set_mv(0, 2000);
	ret_val = ret_val / buf_num;
*/
	return ret_val;
}

int32_t adc_vbatt(void){
	// (int64_t)adc_get(ADC_VB) * 3300LL * 19LL / 4095 / (1 << ADC_Q)
	return (adc_get(ADC_VB) * 4180 / 273) >> ADC_Q;
}

int32_t adc_cur1(void){
	int32_t i;
	//i = ((int64_t)adc_get(ADC_CUR1) - (int64_t)cur_offset[0]) * 3300LL * 1000LL / 4095 / (1 << ADC_Q) / ADC_CUR_GAIN;
	//i = ((((adc_get(ADC_CUR1) - cur_offset[0]) / ADC_CUR_GAIN) * 3300) / 4095) * 1000 / (1 << ADC_Q);
	i = (adc_get(ADC_CUR1) - cur_offset[0]) * 1375 / 39312;
	return -i;
}

int32_t adc_cur2(void){
	int32_t i;
	// 8.6usec
	//i = ((int64_t)adc_get(ADC_CUR2) - (int64_t)cur_offset[1]) * 3300LL * 1000LL / 4095 / (1 << ADC_Q) / ADC_CUR_GAIN;

	// 6.8usec
	//i = ((((adc_get(ADC_CUR2) - cur_offset[1]) / ADC_CUR_GAIN) * 3300) / 4095) * 1000 / (1 << ADC_Q);

	// 6.2usec
	i = (adc_get(ADC_CUR2) - cur_offset[1]) * 1375 / 39312;
	return -i;
}

void adc_cur_cal(void){
	if(do_cur_long_cal){
		cur_offset[0] = (cur_offset[0] + adc_get(ADC_CUR1)) / 2;
		cur_offset[1] = (cur_offset[1] + adc_get(ADC_CUR2)) / 2;
	}else{
		cur_offset[0] = adc_get(ADC_CUR1);
		cur_offset[1] = adc_get(ADC_CUR2);
	}
}

void adc_cur_cal_start(void){
	do_cur_long_cal = false;
	adc_cur_cal();
	do_cur_long_cal = true;
}

void adc_cur_cal_stop(void){
	do_cur_long_cal = false;
}

int32_t adc_cur_offset_delta(int32_t ch, int32_t delta){
	if(ch == 0 || ch == 1){
		cur_offset[ch] += delta * cur_reverse[0];
		return cur_offset[ch];
	}
	return 0;
}

int32_t adc_cur_offset_get(int32_t ch){
	if(ch == 0 || ch == 1){
		return cur_offset[ch];
	}
	return 0;
}

void adc_cur_offset_set(int32_t ch, int32_t offset){
	if(ch == 0 || ch == 1){
		cur_offset[ch] = offset;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC1){

	}else if(hadc->Instance == ADC2){

	}else if(hadc->Instance == ADC3){

	}else if(hadc->Instance == ADC4){

	}
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
