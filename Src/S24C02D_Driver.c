/*
 * S24C02D_Driver.c
 *
 *  Created on: 2016/12/04
 *      Author: strv
 */

#include "S24C02D_Driver.h"

static I2C_HandleTypeDef* phi2c;
static uint8_t tx_buf[EEPROM_PAGE_SIZE];
static uint8_t dev_addr;
volatile static bool processing = false;

static void eeprom_i2c_tx_cb(I2C_HandleTypeDef *hi2c){
	processing = false;
}

static void eeprom_i2c_rx_cb(I2C_HandleTypeDef *hi2c){
	processing = false;
}

HAL_StatusTypeDef eeprom_init(uint32_t addr){
	phi2c = i2c_get_handle();
	i2c_tx_cb_push(eeprom_i2c_tx_cb);
	i2c_rx_cb_push(eeprom_i2c_rx_cb);
	dev_addr = addr;
	return HAL_OK;
}

HAL_StatusTypeDef eeprom_write(uint32_t addr, uint8_t byte){
	return HAL_ERROR;
}

HAL_StatusTypeDef eeprom_write_page(uint32_t addr, uint8_t* bytes){
	HAL_StatusTypeDef status;
	if(HAL_I2C_STATE_READY != HAL_I2C_GetState(phi2c)){
		return HAL_BUSY;
	}
	if(processing){
		return HAL_BUSY;
	}

	for(uint8_t i = 0; i < EEPROM_PAGE_SIZE; i++){
		tx_buf[i] = *(bytes + i);
	}
	processing = true;
	status = HAL_I2C_Mem_Write_DMA(phi2c, dev_addr, addr, I2C_MEMADD_SIZE_8BIT, tx_buf, EEPROM_PAGE_SIZE);
	return status;
}

HAL_StatusTypeDef eeprom_read(uint32_t addr, uint8_t byte){
	return HAL_ERROR;
}

HAL_StatusTypeDef eeprom_read_page_start(uint32_t addr, uint8_t* bytes){
	HAL_StatusTypeDef status;
	if(HAL_I2C_STATE_READY != HAL_I2C_GetState(phi2c)){
		return HAL_BUSY;
	}
	if(processing){
		return HAL_BUSY;
	}

	processing = true;
	status = HAL_I2C_Mem_Read_DMA(phi2c, dev_addr, addr, I2C_MEMADD_SIZE_8BIT, bytes, EEPROM_PAGE_SIZE);
	return status;
}

bool eeprom_is_busy(void){
	return processing;
}
