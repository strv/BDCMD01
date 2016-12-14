/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include "led.h"
#include "uart_util.h"
#include "pwm.h"
#include "xprintf.h"
#include "LSM6DS3_Driver.h"
#include "S24C02D_Driver.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
const uint32_t Interval = 50;
volatile uint32_t tick_last = 0;
volatile uint32_t tick_now = 0;
int32_t led_pos = 1;
uint8_t buf[] = {
		0x00,
		0x01,
		0x02,
		0x03,
		0x04,
		0x05,
		0x06,
		0x07
};
bool set_duty(int32_t argc,int32_t* argv);
UU_ConsoleCommand duty_cmd = {
	"DUTY",
	set_duty,
	"DUTY [ch] [percent]\r\n\
	Set duty ratio to motor driver [ch]. [percent] can set a number between -100 to 100."
};

bool print_cur(int32_t argc, int32_t* argv);
UU_ConsoleCommand cur_cmd = {
	"CUR",
	print_cur,
	"CUR [ch]\r\n\
	Get present current value of [ch] in [mA]. It will print both value if do not set [ch] argument."
};

bool print_adc(int32_t argc, int32_t* argv);
UU_ConsoleCommand print_adc_cmd = {
	"ADC",
	print_adc,
	"ADC [ch]\r\n\
	Get present ADC value of [ch] in raw value. It will print all value if do not set [ch] argument."
};

bool print_vb(int32_t argc, int32_t* argv);
UU_ConsoleCommand print_vb_cmd = {
	"VB",
	print_vb,
	"VB\r\n\
	Get present input voltage in [mV]."
};

bool set_pwm_mode(int32_t argc, int32_t* argv);
UU_ConsoleCommand set_pwm_mode_cmd = {
	"PWMMODE",
	set_pwm_mode,
	"PWMMODE [ch] [mode]\r\n\
	Set / Get PWM output mode. [ch] is 1, 2 or 3. 3 is both. [mode] is 0 (Duty) or 1 (Vcmd). "
};

bool vcmd(int32_t argc, int32_t* argv);
UU_ConsoleCommand vcmd_cmd = {
	"VCMD",
	vcmd,
	"VCMD [ch] [mV]\r\n\
	Set target output voltage in [mV] for [ch]. [ch] is 1 , 2 or 3. 3 is both."
};

bool imu_read(int32_t argc, int32_t* argv);
UU_ConsoleCommand imu_read_cmd = {
	"IMU",
	imu_read,
	"IMU\r\n\
	Get raw IMU value"
};

bool enc_read(int32_t argc, int32_t* argv);
UU_ConsoleCommand enc_read_cmd = {
	"ENC",
	enc_read,
	"ENC\r\n\
	Get raw encoder value"
};
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_OPAMP4_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  led_init();
  uu_init();
  xprintf("Build: %s %s\r\n",__DATE__,__TIME__);

  if(HAL_OK == opa_start_cal()){
	  xprintf("OP amp calibration success.\r\n");
  }else{
	  xprintf("OP amp calibration fail.\r\n");
  }
  opa_start();

  if(HAL_OK == adc_start_cal()){
	  xprintf("ADC calibration success.\r\n");
  }else{
	  xprintf("ADC calibration fail.\r\n");
  }
  adc_start();
  xputs("Start ADC\r\n");
  dac_start();
  xputs("Start DAC\r\n");

  pwm_enable();
  xputs("Enable PWM\r\n");

  IMU_init();
  eeprom_init(0xA0);
  if(HAL_OK != eeprom_write_page(0x00, buf)){
	  xputs("eep write error\r\n");
  }
  xputs("Dump :");
  for(uint32_t i = 0; i < 8; i++){
	  xprintf(" 0x%02X",buf[i]);
  }
  xputs("\r\n");
  for(uint32_t i = 0; i < 8; i++){
	  buf[i] = 0;
  }
  xputs("Dump :");
  for(uint32_t i = 0; i < 8; i++){
	  xprintf(" 0x%02X",buf[i]);
  }
  xputs("\r\n");
  HAL_Delay(10);
  if(HAL_OK != eeprom_read_page_start(0x01, buf)){
	  xputs("eep read error\r\n");
  }
  while(eeprom_is_busy());
  xputs("Dump :");
  for(uint32_t i = 0; i < 8; i++){
	  xprintf(" 0x%02X",buf[i]);
  }
  xputs("\r\n");

  encoder_init(DIR_REV, DIR_REV);
  uu_push_command(&duty_cmd);
  uu_push_command(&cur_cmd);
  uu_push_command(&print_adc_cmd);
  uu_push_command(&print_vb_cmd);
  uu_push_command(&set_pwm_mode_cmd);
  uu_push_command(&vcmd_cmd);
  uu_push_command(&imu_read_cmd);
  uu_push_command(&enc_read_cmd);
  adc_cur_cal_start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  tick_now = HAL_GetTick();
	  if(tick_now - tick_last >= Interval){
		  IMU_reflesh();
		  if(tick_now < 64 * Interval){
			  adc_cur_cal();
		  }else{
			  adc_cur_cal_stop();
		  }
		  led_on(led_pos);
		  led_off(~led_pos);
		  led_pos <<= 1;
		  if(led_pos > LED3){
			  led_pos = LED1;
		  }
		  /*
		  xputs("\r\n");

		  xprintf("Tick:%6d\r\n",tick_now);
		  xprintf("VB :%4d CUR1:%4d CUR2:%4d TEMP:%4d \r\n", adc_get(ADC_VB) * 3300 * 19 / 4095, adc_get(ADC_CUR1), adc_get(ADC_CUR2), adc_get(ADC_TEMP));
		  xprintf("REF:%4d REF :%4d REF :%4d REF :%4d \r\n", adc_get(ADC_REF4), adc_get(ADC_REF2), adc_get(ADC_REF1), adc_get(ADC_REF1));

		  IMU_get_acc(&ax, &ay, &az);
		  xprintf("ACC  X:%6d Y:%6d Z:%6d\r\n", ax, ay, az);

		  IMU_get_gyro(&ax, &ay, &az);
		  xprintf("GYRO X:%6d Y:%6d Z:%6d\r\n", ax, ay, az);

		  xprintf("IMU Temp: %6d\r\n", IMU_get_temp());

		  xprintf("ENC1:%6d ENC2:%6d\r\n", encoder_get(MD_CH1), encoder_get(MD_CH2));
		   */
		  dac_set(0, tick_now);
		  dac_set(1, tick_now * 2);

		  tick_last += Interval;
	  }
	  uu_proc_command();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

bool set_duty(int32_t argc,int32_t* argv){
	if(argc != 2){
		return false;
	}

	if(argv[0] == 1){
		pwm_set_duty(MD_CH1, argv[1]);
	}else if(argv[0] == 2){
		pwm_set_duty(MD_CH2, argv[1]);
	}

	return true;
}

bool print_cur(int32_t argc, int32_t* argv){
	if(argc > 1){
		return false;
	}

	if(argc == 0){
		xprintf("CUR : %6d %6d", adc_cur1(), adc_cur2());
	}else{
		xprintf("CUR%d = ", argv[0]);
		if(argv[0] == 1){
			xprintf("%6d", adc_cur1());
		}else if(argv[0] == 1){
			xprintf("%6d", adc_cur2());
		}
	}
	xputs("\r\n");
	return true;
}

bool print_adc(int32_t argc, int32_t* argv){
	if(argc > 1){
		return false;
	}

	xprintf("ADC : %4d %4d %4d %4d %4d\r\n", adc_get(ADC_VB), adc_get(ADC_CUR1), adc_get(ADC_CUR2), adc_get(ADC_TEMP), adc_get(ADC_REF1));
	return true;
}

bool print_vb(int32_t argc, int32_t* argv){
	if(argc > 0){
		return false;
	}

	xprintf("VB : %5d\r\n", adc_vbatt());
	return true;
}

bool set_pwm_mode(int32_t argc, int32_t* argv){
	if(argc != 2){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}
	if(argv[1] > PWM_MODE_MAX){
		return false;
	}

	pwm_set_mode(argv[0], argv[1]);
	xprintf("Set PWM mode to %s\r\n", argv[1] == PWM_DUTY ? "DUTY" : "VCMD");

	return true;
}

bool vcmd(int32_t argc, int32_t* argv){
	if(argc != 2){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	pwm_set_mv(argv[0], argv[1]);
	xprintf("Set target voltage to %d\r\n", argv[1]);

	return true;
}

bool imu_read(int32_t argc, int32_t* argv){
	int32_t ax,ay,az;
	IMU_get_acc(&ax, &ay, &az);
	xprintf("ACC  X:%6d Y:%6d Z:%6d\r\n", ax, ay, az);
	IMU_get_gyro(&ax, &ay, &az);
	xprintf("GYRO X:%6d Y:%6d Z:%6d\r\n", ax, ay, az);
	xprintf("IMU Temp: %6d\r\n", IMU_get_temp());
	return true;
}

bool enc_read(int32_t argc, int32_t* argv){
	xprintf("ENC1:%6d ENC2:%6d\r\n",(int32_t)encoder_get(MD_CH1), (int32_t)encoder_get(MD_CH2));
	return true;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
