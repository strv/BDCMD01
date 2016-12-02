/*
 * uart_util.h
 *
 *  Created on: Jul 29, 2016
 *      Author: strv
 */

#ifndef UART_UTIL_H_
#define UART_UTIL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define	UU_BUFF_LEN		(128)				// ïKÇ∏2ÇÃó›èÊÇÃílÇ…Ç∑ÇÈ
#define	UU_UART			USART2
#define	UU_IRQ_Handler	USART2_IRQHandler
#define	UU_NL_TXT		'\r'

void uu_init(void);
void uu_putc(unsigned char c);
unsigned char uu_getc(void);
uint16_t uu_rxed_nl_cnt(void);
bool uu_rx_buff_ore(void);
bool uu_tx_buff_ore(void);
void UU_IRQ_Handler(void);

#endif /* UART_UTIL_H_ */
