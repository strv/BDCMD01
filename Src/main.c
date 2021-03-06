/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Device memo				*/
/* Flash 256 kB 			*/
/* SRAM  40 kB				*/
/* CCM RAM 8 kB				*/
#include <stdbool.h>
#include <stdint.h>
#include <arm_math.h>
#include "led.h"
#include "uart_util.h"
#include "pwm.h"
#include "xprintf.h"
#include "LSM6DS3_Driver.h"
#include "S24C02D_Driver.h"
#include "encoder.h"
#include "torque_cntl.h"
#include "speed_cntl.h"
#include "MadgwickAHRS.h"
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
int32_t rpm_log_num = 0;

MotorProfile mtp_PG45775 = {
	0,
	"PG45775 24V 6000rpm",
	4400,		// measured value
	1865,		// measured value
	6000,
	5233 / 24,
	385,
	12100,
	630,
	24000,
	30800,
	0,
	0,
	0
};

MotorProfile mtp_GT_Tuen = {
	0,
	"Tamiya GT Tune",
	200,		// measured value
	75,			// measured value
	16400,
	2277,
	0,
	36000,
	3100,
	7200,
	120000,
	0,
	0,
	0
};

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

bool tc_en(int32_t argc, int32_t* argv);
UU_ConsoleCommand tc_en_cmd = {
	"TCEN",
	tc_en,
	"TC_EN [ch]\r\n\
	Enable a torque controller"
};

bool ccmd(int32_t argc, int32_t* argv);
UU_ConsoleCommand ccmd_cmd = {
	"CCMD",
	ccmd,
	"CCMD [ch] [mA]\r\n\
	Set target output current in [mA] for [ch]. [ch] is 1 , 2 or 3. 3 is both."
};

bool tcmd(int32_t argc, int32_t* argv);
UU_ConsoleCommand tcmd_cmd = {
	"TCMD",
	tcmd,
	"TCMD [ch] [uNm]\r\n\
	Set target output torque in [uNm] for [ch]. [ch] is 1 , 2 or 3. 3 is both."
};

bool tc_gain(int32_t argc, int32_t* argv);
UU_ConsoleCommand tc_gain_cmd = {
	"TCGAIN",
	tc_gain,
	"TCGAIN [ch] [kp] [ki] [kd]\r\n\
	Set PID gain to [ch]. [ch] is 1 , 2 or 3. 3 is both."
};

bool tc_lsm(int32_t argc, int32_t* argv);
UU_ConsoleCommand tc_lsm_cmd = {
	"TCLSM",
	tc_lsm,
	"TCLSM [ch] [kc] [fc]\r\n\
	Set PID gain by Limit Sensitibity Method to [ch]. [ch] is 1 , 2 or 3. 3 is both.\r\n\
	kc : limit kp\r\n\
	fc : vibration freq[Hz]"
};

bool tc_bemf(int32_t argc, int32_t* argv);
UU_ConsoleCommand tc_bemf_cmd = {
	"BEMF",
	tc_bemf,
	"BEMF [ch]\r\n\
	Get BEMF value of [ch]. [ch] is 1 or 2"
};

bool tc_vout(int32_t argc, int32_t* argv);
UU_ConsoleCommand tc_vout_cmd = {
	"TCVO",
	tc_vout,
	"TCVO [ch]\r\n\
	Get Vout value of [ch]. [ch] is 1 or 2"
};

bool sc_en(int32_t argc, int32_t* argv);
UU_ConsoleCommand sc_en_cmd = {
	"SCEN",
	sc_en,
	"SC_EN [ch]\r\n\
	Enable a speed controller"
};

bool rpmcmd(int32_t argc, int32_t* argv);
UU_ConsoleCommand rpmcmd_cmd = {
	"RPM",
	rpmcmd,
	"RPM [ch] [rpm]\r\n\
	Set target speed in [rpm] for [ch]. [ch] is 1 , 2 or 3. 3 is both."
};

bool sc_gain(int32_t argc, int32_t* argv);
UU_ConsoleCommand sc_gain_cmd = {
	"SCGAIN",
	sc_gain,
	"SCGAIN [ch] [kp] [ki] [kd]\r\n\
	Set PID gain to [ch]. [ch] is 1 , 2 or 3. 3 is both."
};

bool sc_lsm(int32_t argc, int32_t* argv);
UU_ConsoleCommand sc_lsm_cmd = {
	"SCLSM",
	sc_lsm,
	"SCLSM [ch] [kc] [fc]\r\n\
	Set PID gain by Limit Sensitibity Method to [ch]. [ch] is 1 , 2 or 3. 3 is both.\r\n\
	kc : limit kp\r\n\
	fc : vibration freq[Hz]"
};

bool clk_info(int32_t argc, int32_t* argv);
UU_ConsoleCommand clk_cmd = {
	"CLK",
	clk_info,
	"CLK\r\n\
	Get Clock information"
};

bool rpm_log(int32_t argc, int32_t* argv);
UU_ConsoleCommand rpm_log_cmd = {
	"RPMLOG",
	rpm_log,
	"RPMLOG [number]\r\n\
	Start RPM log"
};

bool get_pose(int32_t argc, int32_t* argv);
UU_ConsoleCommand pose_cmd = {
	"POSE",
	get_pose,
	"POSE\r\n\
	Return current pose of the board"
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
  MX_TIM7_Init();

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
  dac_set_mv(0, 1000);
  dac_set_mv(1, 2000);

  pwm_enable();
  xputs("Enable PWM\r\n");

  tc_init();
  sc_init();

  IMU_init();
  IMU_set_acc_range(IMU_ACC_2g);
  IMU_set_gyro_range(IMU_GYRO_500dps);
  IMU_set_acc_rate(IMU_ACC_1666);
  IMU_set_gyro_rate(IMU_GYRO_1666);

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

  uu_push_command(&tc_en_cmd);
  uu_push_command(&ccmd_cmd);
  uu_push_command(&tcmd_cmd);
  uu_push_command(&tc_gain_cmd);
  uu_push_command(&tc_lsm_cmd);
  uu_push_command(&tc_bemf_cmd);
  uu_push_command(&tc_vout_cmd);

  uu_push_command(&sc_en_cmd);
  uu_push_command(&rpmcmd_cmd);
  uu_push_command(&sc_gain_cmd);
  uu_push_command(&sc_lsm_cmd);
  uu_push_command(&clk_cmd);
  uu_push_command(&rpm_log_cmd);
  uu_push_command(&pose_cmd);

  adc_cur_cal_start();

  pose_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  tick_now = HAL_GetTick();
	  if(tick_now - tick_last >= Interval){
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

		  if(rpm_log_num > 0){
			  rpm_log_num--;
			  xprintf("%d,%d,%d\r\n", sc_get_speed_bemf(MD_CH1), sc_get_speed_enc(MD_CH1),adc_cur1());
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
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV2;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV2;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 8, 0);
}

/* USER CODE BEGIN 4 */
#define BLOCK_SIZE (64)
#define NUM_TAPS (63)
int32_t ia[3], ig[3];
float fa[3], fg[3];
int32_t ig_ofst[3] = {0, 0, 0};
int32_t g_cnt = -100;
arm_fir_instance_f32 fir_struct;
uint32_t block_size = BLOCK_SIZE;
float32_t fir_state[BLOCK_SIZE + NUM_TAPS + 1];
/*
float32_t fir_coeffs[NUM_TAPS] = {
		-1.599957890020377e-04,
		-1.079992893898858e-03,
		9.980000000000000e-01,
		-1.079992893898858e-03,
		-1.599957890020377e-04
};
*/
float32_t fir_coeffs[NUM_TAPS] = {
		0.000000000000000e+00 ,
		-2.588688795350144e-05 ,
		-1.088123368621265e-04 ,
		-2.560599756378687e-04 ,
		-4.739466003157051e-04 ,
		-7.676498064290117e-04 ,
		-1.141055372585724e-03 ,
		-1.596628075556499e-03 ,
		-2.135309159660887e-03 ,
		-2.756443148364572e-03 ,
		-3.457736085075439e-03 ,
		-4.235246636048148e-03 ,
		-5.083410795294219e-03 ,
		-5.995100214663904e-03 ,
		-6.961713457688311e-03 ,
		-7.973298759509207e-03 ,
		-9.018706183366089e-03 ,
		-1.008576641229374e-02 ,
		-1.116149281773711e-02 ,
		-1.223230291838822e-02 ,
		-1.328425489489190e-02 ,
		-1.430329446959375e-02 ,
		-1.527550720364601e-02 ,
		-1.618737111277017e-02 ,
		-1.702600446166274e-02 ,
		-1.777940366682992e-02 ,
		-1.843666641746229e-02 ,
		-1.898819541024209e-02 ,
		-1.942587848074459e-02 ,
		-1.974324139310644e-02 ,
		-1.993557011055170e-02 ,
		9.800000000000000e-01 ,
		-1.993557011055170e-02 ,
		-1.974324139310644e-02 ,
		-1.942587848074459e-02 ,
		-1.898819541024209e-02 ,
		-1.843666641746229e-02 ,
		-1.777940366682992e-02 ,
		-1.702600446166274e-02 ,
		-1.618737111277017e-02 ,
		-1.527550720364601e-02 ,
		-1.430329446959375e-02 ,
		-1.328425489489190e-02 ,
		-1.223230291838822e-02 ,
		-1.116149281773711e-02 ,
		-1.008576641229374e-02 ,
		-9.018706183366089e-03 ,
		-7.973298759509207e-03 ,
		-6.961713457688311e-03 ,
		-5.995100214663904e-03 ,
		-5.083410795294219e-03 ,
		-4.235246636048148e-03 ,
		-3.457736085075439e-03 ,
		-2.756443148364572e-03 ,
		-2.135309159660887e-03 ,
		-1.596628075556499e-03 ,
		-1.141055372585724e-03 ,
		-7.676498064290117e-04 ,
		-4.739466003157051e-04 ,
		-2.560599756378687e-04 ,
		-1.088123368621265e-04 ,
		-2.588688795350144e-05 ,
		0.000000000000000e+00
};
float32_t imu_yaw_src[BLOCK_SIZE];
float32_t imu_yaw_dst[BLOCK_SIZE];

void pose_init(){
  Madgwick_init();
  Madgwick_begin(1000, 2.0);

  arm_fir_init_f32(&fir_struct, NUM_TAPS, &fir_coeffs[0], &fir_state[0], block_size);
}

void pose_zero_yaw(){

}

void pose_proc(){
  if(!Madgwick_is_init()){
	  return;
  }
  IMU_get_acc(&ia[0], &ia[1], &ia[2]);
  IMU_get_gyro(&ig[0], &ig[1], &ig[2]);

  if(g_cnt < 0){
	  g_cnt++;
  }else if(g_cnt < 250){
	  ig_ofst[0] += ig[0];
	  ig_ofst[1] += ig[1];
	  ig_ofst[2] += ig[2];
	  g_cnt++;
  }else if(g_cnt == 250){
	  ig_ofst[0] /= g_cnt;
	  ig_ofst[1] /= g_cnt;
	  ig_ofst[2] /= g_cnt;
	  xprintf("ig_ofst : %d %d %d\r\n",
			  (int32_t)(ig_ofst[0]),
			  (int32_t)(ig_ofst[1]),
			  (int32_t)(ig_ofst[2]));
	  g_cnt++;
  }

  for(int i=0; i<3; i++){
	  fa[i] = 2. * (float)ia[i] / 32768.;
	  fg[i] = 500. * (float)(ig[i] - ig_ofst[i]) / 32768.;
  }
/*
  for(int i=0; i<block_size - 1; i++){
	  imu_yaw_src[i] = imu_yaw_src[i+1];
  }
  imu_yaw_src[block_size - 1] = fg[2];
  arm_fir_f32(&fir_struct, imu_yaw_src, imu_yaw_dst, block_size);
*/
/*
  Madgwick_updateIMU(fg[0], fg[1], imu_yaw_dst[block_size - 1],
		  fa[0], fa[1], fa[2]);
*/
  // 30usec for calculation
  Madgwick_updateIMU(fg[0], fg[1], fg[2],
		  fa[0], fa[1], fa[2]);
  IMU_reflesh();
}

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

bool tc_en(int32_t argc, int32_t* argv){
	if(argc != 1){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	tc_enable(argv[0]);
	xputs("Enable torque controller.\r\n");
	return true;
}

bool ccmd(int32_t argc, int32_t* argv){
	if(argc != 2){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	tc_set_ma(argv[0], argv[1]);
	xprintf("Set target current to %d\r\n", argv[1]);
	return true;
}

bool tcmd(int32_t argc, int32_t* argv){
	if(argc != 2){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	tc_set_trq(argv[0], argv[1]);
	xprintf("Set target torque to %d\r\n", argv[1]);
	return true;
}

bool tc_gain(int32_t argc, int32_t* argv){
	if(argc != 4){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	tc_set_gain(argv[0], argv[1], argv[2], argv[3]);
	xputs("Set torque gain to...\r\n");
	xprintf("kp : %d\r\n", argv[1]);
	xprintf("ki : %d\r\n", argv[2]);
	xprintf("kd : %d\r\n", argv[3]);
	return true;
}

bool tc_lsm(int32_t argc, int32_t* argv){
	if(argc != 3){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	int32_t p,i,d;
	tc_set_gain_by_lsm(argv[0], argv[1], argv[2]);
	tc_get_gain(argv[0], &p, &i, &d);
	xputs("Set torque gain to...\r\n");
	xprintf("kp : %d\r\n", p);
	xprintf("ki : %d\r\n", i);
	xprintf("kd : %d\r\n", d);

	return true;
}

bool tc_bemf(int32_t argc, int32_t* argv){
	if(argc != 1){
		return false;
	}
	if(argv[0] > MD_CH12){
		return false;
	}

	tc_bemf_est(argv[0]);
	xprintf("BEMF [%d] : %d\r\n", argv[0], tc_bemf_est(argv[0]));
	return true;
}

bool tc_vout(int32_t argc, int32_t* argv){
	if(argc != 1){
		return false;
	}
	if(argv[0] > MD_CH12){
		return false;
	}

	tc_bemf_est(argv[0]);
	xprintf("Vout [%d] : %d mV\r\n", argv[0], tc_get_vout(argv[0]));
	return true;
}

bool sc_en(int32_t argc, int32_t* argv){
	if(argc != 1){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	sc_enable(argv[0]);
	xputs("Enabled Speed controller.\r\n");
	return true;
}

bool rpmcmd(int32_t argc, int32_t* argv){
	if(argc == 0){
		xprintf("BEMF RPM : %6d %6d\r\n", sc_get_speed_bemf(MD_CH1), sc_get_speed_bemf(MD_CH2));
		xprintf("ENC  RPM : %6d %6d\r\n", sc_get_speed_enc(MD_CH1), sc_get_speed_enc(MD_CH2));
		return true;
	}

	if(argc != 2){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	sc_set_speed(argv[0], argv[1]);
	xprintf("Set target speed to %d [rpm]\r\n", argv[1]);
	return true;
}

bool sc_gain(int32_t argc, int32_t* argv){
	if(argc != 4){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	sc_set_gain(argv[0], argv[1], argv[2], argv[3]);
	xputs("Set torque gain to...\r\n");
	xprintf("kp : %d\r\n", argv[1]);
	xprintf("ki : %d\r\n", argv[2]);
	xprintf("kd : %d\r\n", argv[3]);
	return true;
}

bool sc_lsm(int32_t argc, int32_t* argv){
	if(argc != 3){
		return false;
	}
	if(argv[0] > MD_CH_MAX){
		return false;
	}

	int32_t p,i,d;
	sc_set_gain_by_lsm(argv[0], argv[1], argv[2]);
	sc_get_gain(argv[0], &p, &i, &d);
	xputs("Set torque gain to...\r\n");
	xprintf("kp : %d\r\n", p);
	xprintf("ki : %d\r\n", i);
	xprintf("kd : %d\r\n", d);

	return true;
}

bool clk_info(int32_t argc, int32_t* argv){
	xprintf("SysClockFreq %d\r\n", HAL_RCC_GetSysClockFreq());
	xprintf("HCLKFreq %d\r\n", HAL_RCC_GetHCLKFreq());
	xprintf("PCLK1Freq %d\r\n", HAL_RCC_GetPCLK1Freq());
	xprintf("PCLK2Freq %d\r\n", HAL_RCC_GetPCLK2Freq());
	return true;
}

bool rpm_log(int32_t argc, int32_t* argv){
	if(argc != 1){
		return false;
	}
	xprintf("START LOG\r\n");
	rpm_log_num = argv[0];
	return true;
}

bool get_pose(int32_t argc, int32_t* argv){
	float r,p,y;
	int32_t ir, ip, iy;
	Madgwick_getPose(&r, &p, &y);
	ir = r;
	ip = p;
	iy = y;
	xprintf("Yaw : %7d\r\n", (int32_t)(imu_yaw_dst[block_size - 1] * 1000.));
	xprintf("Pose : %4d.%03d %4d.%03d %4d.%03d\r\n",
			ir, abs((int32_t)((r - ir) * 1000)),
			ip, abs((int32_t)((p - ip) * 1000)),
			iy, abs((int32_t)((y - iy) * 1000)) );
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
