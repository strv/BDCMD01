/*
 * S24C02D_Driver.h
 *
 *  Created on: 2016/12/04
 *      Author: strv
 */

#ifndef S24C02D_DRIVER_H_
#define S24C02D_DRIVER_H_

#include "i2c.h"
#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define EEPROM_PAGE_SIZE	(8)	//bytes

HAL_StatusTypeDef eeprom_init(uint32_t dev_addr);
HAL_StatusTypeDef eeprom_write(uint32_t addr, uint8_t byte);
HAL_StatusTypeDef eeprom_write_page(uint32_t addr, uint8_t* bytes);
HAL_StatusTypeDef eeprom_read(uint32_t addr, uint8_t byte);
HAL_StatusTypeDef eeprom_read_page_start(uint32_t addr, uint8_t* bytes);
bool eeprom_is_busy(void);
/*
void eeprom_i2c_tx_cb(I2C_HandleTypeDef *hi2c);
void eeprom_i2c_rx_cb(I2C_HandleTypeDef *hi2c);
*/
#endif /* S24C02D_DRIVER_H_ */
