/*
 * LSM6DS3_Driver.c
 *
 *  Created on: 2016/12/02
 *      Author: strv
 */

#include "LSM6DS3_Driver.h"
#include <stdlib.h>
#include <stdbool.h>
#include "xprintf.h"
#include "main.h"

#define	SPI_TO (2000)
#define SPI_BUF_LEN (256)
static SPI_HandleTypeDef* phspi;

static uint8_t* p_buf;
static uint8_t tx_buf[SPI_BUF_LEN];
static uint8_t rx_buf[SPI_BUF_LEN];
static bool processing = false;
static uint8_t processing_reg = 0x00;

static int32_t pres_temp;
static int32_t pres_acc[3];
static int32_t pres_gyro[3];

static GPIO_InitTypeDef GPIO_InitStruct = {
		SPI_CS_Pin,
		GPIO_MODE_OUTPUT_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_HIGH,
		0
};

#define negate() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)
#define assert() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)

static const uint8_t CTRL_REGS_DEF[] = {
	/* CTRL1_XL */ (0x8 << 4) | (0x0 << 2) | (0x0 << 0), //ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0 FS_XL1 FS_XL0 BW_XL1 BW_XL0
	/* CTRL2_G  */ (0x8 << 4) | (0x0 << 2) | (0x0 << 1), //ODR_G3 ODR_G2 ODR_G1 ODR_G0 FS_G1 FS_G0 FS_125 0
	/* CTRL3_C  */ (0x0 << 7) | (0x0 << 6) | (0x0 << 5) | (0x0 << 4) | (0x0 << 3) | (0x1 << 2) | (0x0 << 1) | (0x0 << 0),//BOOT BDU H_LACTIVE PP_OD SIM IF_INC BLE SW_RESET
	/* CTRL4_C  */ (0x0 << 7) | (0x0 << 6) | (0x0 << 5) | (0x0 << 4) | (0x0 << 3) | (0x1 << 2) | (0x0 << 1) | (0x0 << 0),
	/* CTRL5_C  */ (0x0 << 5) | (0x0 << 2) | (0x0 << 0),
	/* CTRL6_C  */ (0x0 << 7) | (0x0 << 6) | (0x0 << 5) | (0x0 << 4),
	/* CTRL7_G  */ (0x0 << 7) | (0x0 << 6) | (0x0 << 4) | (0x0 << 3) | (0x0 << 2),
	/* CTRL8_XL */ (0x1 << 7) | (0x0 << 5) | (0x0 << 2) | (0x0 << 0),
	/* CTRL9_XL */ (0x1 << 5) | (0x1 << 4) | (0x1 << 3) | (0x0 << 2),
	/* CTRL10_C */ (0x1 << 5) | (0x1 << 4) | (0x1 << 3) | (0x0 << 2) | (0x0 << 1) | (0x0 << 0)
};

static HAL_StatusTypeDef cmd_read(uint8_t reg, uint8_t* pbuf, uint16_t len){
	HAL_StatusTypeDef status;

	if(processing){
		return HAL_BUSY;
	}

	HAL_SPI_StateTypeDef spi_status = HAL_SPI_GetState(phspi);
	if(spi_status != HAL_SPI_STATE_READY){
		xprintf("SPI Not ready : %02X", spi_status);
		return HAL_BUSY;
	}

	assert();
/*
	uint8_t* p_rxbuf = (uint8_t *)malloc(len + 1);
	uint8_t* p_txbuf = (uint8_t *)malloc(len + 1);
	*p_txbuf = 0x80 | reg;
	for(uint32_t i = 0; i < len; i++){
		*(p_rxbuf + i + 1) = *(pbuf + i);
		*(p_txbuf + i + 1) = 0xFF;
	}
	HAL_SPI_StateTypeDef spi_status = HAL_SPI_GetState(phspi);
	if(spi_status != HAL_SPI_STATE_READY){
		xprintf("SPI Not ready : %02X", spi_status);
		return HAL_ERROR;
	}
	status = HAL_SPIEx_FlushRxFifo(phspi);
	if(status != HAL_OK){
		xputs("SPI Error at flush\r\n");
		xputs("Errcode : ");
		switch(status){
		case HAL_ERROR:
			xputs("ERROR");
			break;
		case HAL_BUSY:
			xputs("BUSY");
			break;
		case HAL_TIMEOUT:
			xputs("TIMEOUT");
			break;
		}
		xputs("\r\n");
	}
	spi_status = HAL_SPI_GetState(phspi);
	if(spi_status != HAL_SPI_STATE_READY){
		xprintf("!!!!! SPI Not ready : %02X", spi_status);
		return HAL_ERROR;
	}
	status = HAL_SPI_TransmitReceive(phspi, p_txbuf, p_rxbuf, len + 1, SPI_TO);
	for(uint32_t i = 0; i < len; i++){
		*(pbuf + i) = *(p_rxbuf + i + 1);
	}
	if(status != HAL_OK){
		xputs("!!!!! SPI Error at TransmitReceive\r\n");
		xputs("Errcode : ");
		switch(status){
		case HAL_ERROR:
			xputs("ERROR");
			break;
		case HAL_BUSY:
			xputs("BUSY");
			break;
		case HAL_TIMEOUT:
			xputs("TIMEOUT");
			break;
		}
		xputs("\r\n");
	}
*/
	reg |= 0x80;
	status = HAL_SPI_Transmit(phspi, &reg, 1, SPI_TO);
	if(status != HAL_OK){
		xprintf("SPI Error at sending cmd : 0x%0X\r\n", reg);
	}
	status = HAL_SPIEx_FlushRxFifo(phspi);
	if(status != HAL_OK){
		xputs("SPI Error at flush\r\n");
	}
	status = HAL_SPI_Receive(phspi, pbuf, len, SPI_TO);
	if(status != HAL_OK){
		xputs("SPI Error at reading :");
		for(uint32_t i = 0; i < len; i++){
			xprintf(" 0x%02X", *(pbuf + i));
		}
		xputs("\r\n");
	}
	negate();
	return status;
}

static HAL_StatusTypeDef cmd_write(uint8_t reg, uint8_t* pbuf, uint32_t len){
	HAL_StatusTypeDef status;
	assert();
	reg &= ~0x80;
	HAL_SPI_StateTypeDef spi_status = HAL_SPI_GetState(phspi);
	if(spi_status != HAL_SPI_STATE_READY){
		xprintf("SPI Not ready : %02X", spi_status);
		return HAL_ERROR;
	}
	status = HAL_SPI_Transmit(phspi, &reg, 1, SPI_TO);
	if(status != HAL_OK){
		xprintf("SPI Error at sending cmd : 0x%0X\r\n", reg);
	}
	spi_status = HAL_SPI_GetState(phspi);
	if(spi_status != HAL_SPI_STATE_READY){
		xprintf("SPI Not ready : %02X", spi_status);
		return HAL_ERROR;
	}
	status = HAL_SPI_Transmit(phspi, pbuf, len, SPI_TO);
	if(status != HAL_OK){
		xputs("SPI Error at writing :");
		for(uint32_t i = 0; i < len; i++){
			xprintf(" 0x%02X", *(pbuf + i));
		}
		xputs("\r\n");
	}
	negate();
	return status;
}

static HAL_StatusTypeDef cmd_read_start(uint8_t reg, uint16_t len){
	HAL_StatusTypeDef status;

	if(processing){
		return HAL_BUSY;
	}

	HAL_SPI_StateTypeDef spi_status = HAL_SPI_GetState(phspi);
	if(spi_status != HAL_SPI_STATE_READY){
		xprintf("SPI Not ready : %02X", spi_status);
		return HAL_BUSY;
	}

	status = HAL_SPIEx_FlushRxFifo(phspi);
	if(status != HAL_OK){
		xputs("SPI Error at flush\r\n");
		xputs("Errcode : ");
		switch(status){
		case HAL_ERROR:
			xputs("ERROR");
			break;
		case HAL_BUSY:
			xputs("BUSY");
			break;
		case HAL_TIMEOUT:
			xputs("TIMEOUT");
			break;
		default:
			break;
		}
		xputs("\r\n");
	}

	tx_buf[0] = reg | 0x80;
	processing = true;
	assert();
	status = HAL_SPI_TransmitReceive_DMA(phspi, tx_buf, rx_buf, len + 1);
	if(status != HAL_OK){
		xprintf("Error at SPI DMA txrx : %d\r\n", status);
		processing = false;
		negate();
		return HAL_ERROR;
	}
	return HAL_OK;
}

void IMU_init(void){
	HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);
	negate();

	phspi = spi_get_handle();
	spi_tx_cb_push(IMU_spi_tx_cb);
	spi_txrx_cb_push(IMU_spi_txrx_cb);

	p_buf = (uint8_t *)malloc(1);
	if(p_buf == NULL){
		xprintf("ERR: Could not allocate a memory to prx_buf\r\n");
	}
	if(HAL_OK == cmd_read(0x0F, p_buf, 1)){
		xprintf("IMU Device : 0x%02X\r\n", *p_buf);
	}

	if(HAL_OK == cmd_write(0x10, (uint8_t*)CTRL_REGS_DEF, 10)){
		xputs("IMU configured\r\n");
	}

	free(p_buf);
}

void IMU_set_acc_range(IMU_ACC_RANGE range){
	uint8_t reg;
	cmd_read(0x10, &reg, 1);
	reg &= ~0x0C;
	reg |= (range << 2) & 0x0C;
	cmd_write(0x10, &reg, 1);
}

void IMU_set_gyro_range(IMU_GYRO_RANGE range){
	uint8_t reg;
	cmd_read(0x11, &reg, 1);
	reg &= ~0x0E;
	reg |= (range << 1) & 0x0E;
	cmd_write(0x11, &reg, 1);
}

void IMU_set_acc_rate(IMU_ACC_RATE rate){
	uint8_t reg;
	cmd_read(0x10, &reg, 1);
	reg &= ~0xF0;
	reg |= (rate << 4) & 0xF0;
	cmd_write(0x10, &reg, 1);
}

void IMU_set_gyro_rate(IMU_GYRO_RATE rate){
	uint8_t reg;
	cmd_read(0x11, &reg, 1);
	reg &= ~0xF0;
	reg |= (rate << 4) & 0xF0;
	cmd_write(0x11, &reg, 1);
}

int32_t IMU_get_temp(void){
	int32_t t = 0;
	/*
	p_buf = (uint8_t *)malloc(2);
	if(p_buf == NULL){
		xputs("ERR: Could not allocate a memory to p_buf\r\n");
		return 0;
	}

	if(HAL_OK == cmd_read(0x20, p_buf, 2)){
		t = (int16_t)((((uint16_t)p_buf[1]) << 8) + (uint16_t)p_buf[0]);
	}else{
		xputs("IMU get temp error\r\n");
	}

	free(p_buf);
	*/
	t = (int16_t)((((uint16_t)rx_buf[2]) << 8) + (uint16_t)rx_buf[1]);
	return t;
}

void IMU_get_acc(int32_t* px, int32_t* py, int32_t* pz){
#if 0
	p_buf = (uint8_t *)malloc(8);
	if(p_buf == NULL){
		xputs("ERR: Could not allocate a memory to p_buf\r\n");
		return;
	}

	if(HAL_OK == cmd_read(0x28, p_buf, 8)){
		xputs("Received :");
		for(uint16_t i = 0; i < 8; i++){
			xprintf(" 0x%02X", p_buf[i]);
		}
		xputs("\r\n");
		/*
		*px = (int16_t)((((uint16_t)p_buf[1]) << 8) + (uint16_t)p_buf[0]);
		*py = (int16_t)((((uint16_t)p_buf[3]) << 8) + (uint16_t)p_buf[2]);
		*pz = (int16_t)((((uint16_t)p_buf[5]) << 8) + (uint16_t)p_buf[4]);
		*/
	}else{
		xputs("IMU get acc error\r\n");
	}

	free(p_buf);
#endif
	*px = (int16_t)((((uint16_t)rx_buf[10]) << 8) + (uint16_t)rx_buf[9]);
	*py = (int16_t)((((uint16_t)rx_buf[12]) << 8) + (uint16_t)rx_buf[11]);
	*pz = (int16_t)((((uint16_t)rx_buf[14]) << 8) + (uint16_t)rx_buf[13]);
}

void IMU_get_gyro(int32_t* pr, int32_t* pp, int32_t* py){
#if 0
	p_buf = (uint8_t *)malloc(8);
	if(p_buf == NULL){
		xputs("ERR: Could not allocate a memory to p_buf\r\n");
		return;
	}

	if(HAL_OK == cmd_read(0x22, p_buf, 8)){
		xputs("Received :");
		for(uint16_t i = 0; i < 8; i++){
			xprintf(" 0x%02X", p_buf[i]);
		}
		xputs("\r\n");
		/*
		*pr = (int16_t)((((uint16_t)p_buf[1]) << 8) + (uint16_t)p_buf[0]);
		*pp = (int16_t)((((uint16_t)p_buf[3]) << 8) + (uint16_t)p_buf[2]);
		*py = (int16_t)((((uint16_t)p_buf[5]) << 8) + (uint16_t)p_buf[4]);
		*/
	}else{
		xputs("IMU get gyro error\r\n");
	}

	free(p_buf);
#endif
	*pr = (int16_t)((((uint16_t)rx_buf[4]) << 8) + (uint16_t)rx_buf[3]);
	*pp = (int16_t)((((uint16_t)rx_buf[6]) << 8) + (uint16_t)rx_buf[5]);
	*py = (int16_t)((((uint16_t)rx_buf[8]) << 8) + (uint16_t)rx_buf[7]);
}

void IMU_reflesh(void){
	cmd_read_start(0x20, 14);
}

void IMU_spi_tx_cb(SPI_HandleTypeDef *hspi){
	if(processing){
		processing = false;
		negate();
	}
}

void IMU_spi_txrx_cb(SPI_HandleTypeDef *hspi){
	if(processing){
		processing = false;
		negate();
	}
}
