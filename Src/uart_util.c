/*
 * uart_util.c
 *
 *  Created on: Jul 29, 2016
 *      Author: strv
 */

#include "uart_util.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "usart.h"
#include "xprintf.h"

char rx_buff[UU_BUFF_LEN];
char tx_buff[UU_BUFF_LEN];
uint16_t rx_buff_wp;
uint16_t rx_buff_rp;
uint16_t tx_buff_wp;
uint16_t tx_buff_rp;
bool tx_ore, rx_ore;
uint16_t nl_cnt;

void uu_init(void){
	rx_buff_wp = 0;
	rx_buff_rp = 0;
	tx_buff_wp = 0;
	tx_buff_rp = 0;
	tx_ore = false;
	rx_ore = false;
	nl_cnt = 0;
	xdev_out(uu_putc);
	xdev_in(uu_getc);
}

void uu_putc(unsigned char c){
	uint16_t tmp_p;
	tmp_p = (tx_buff_wp + 1) & (UU_BUFF_LEN - 1);
	if(tmp_p != tx_buff_rp){
		UU_UART->CR1 &= ~USART_CR1_TXEIE;
		tx_buff[tmp_p] = c;
		tx_buff_wp = tmp_p;
		UU_UART->CR1 |= USART_CR1_TXEIE;
	}else{
		tx_ore = true;
	}
}

unsigned char uu_getc(void){
	char c = 0;
	if(rx_buff_rp != rx_buff_wp){
		UU_UART->CR1 &= ~USART_CR1_RXNEIE;
		rx_buff_rp++;
		rx_buff_rp &= (UU_BUFF_LEN - 1);
		c = rx_buff[rx_buff_rp];
		if(c == UU_NL_TXT){
			nl_cnt--;
		}
		UU_UART->CR1 |= USART_CR1_RXNEIE;
	}
	return c;
}

uint16_t uu_rxed_nl_cnt(void){
	return nl_cnt;
}

bool uu_rx_buff_ore(void){
	bool b = rx_ore;
	rx_ore = false;
	return b;
}

bool uu_tx_buff_ore(void){
	bool b = tx_ore;
	tx_ore = false;
	return b;
}

char rx_tmp;
uint32_t int_state;
void UU_IRQ_Handler(void){
	uint16_t tmp_p;
	int_state = UU_UART->ISR;

	if(int_state & USART_ISR_RXNE){
		tmp_p = (rx_buff_wp + 1) & (UU_BUFF_LEN - 1);
		rx_tmp = UU_UART->RDR;
		if(tmp_p != rx_buff_rp){
			uu_putc(rx_tmp);
			rx_buff[tmp_p] = rx_tmp;
			if(rx_tmp == UU_NL_TXT){
				nl_cnt++;
			}
			rx_buff_wp = tmp_p;
		}else{
			rx_ore = true;
			UU_UART->CR1 &= ~USART_CR1_RXNEIE;
		}
	}

	if(int_state & USART_ISR_TXE){
		if(tx_buff_rp != tx_buff_wp){
			tx_buff_rp++;
			tx_buff_rp &= (UU_BUFF_LEN - 1);
			UU_UART->TDR = tx_buff[tx_buff_rp];
		}else{
			UU_UART->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}
