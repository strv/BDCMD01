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
#include <string.h>

static char rx_buff[UU_BUFF_LEN];
static char tx_buff[UU_BUFF_LEN];
static uint16_t rx_buff_wp;
static uint16_t rx_buff_rp;
static uint16_t tx_buff_wp;
static uint16_t tx_buff_rp;
static bool tx_ore, rx_ore;
static uint16_t nl_cnt;
static UU_ConsoleCommand commands[UU_MAX_COMMAND_NUM];
static uint16_t command_cnt;

static int32_t str2upper(char* str);

void uu_init(void){
	rx_buff_wp = 0;
	rx_buff_rp = 0;
	tx_buff_wp = 0;
	tx_buff_rp = 0;
	tx_ore = false;
	rx_ore = false;
	nl_cnt = 0;
	command_cnt = 0;
	xdev_out(uu_putc);
	xdev_in(uu_getc);

	UU_UART->CR1 |= USART_CR1_RXNEIE;
}

void uu_putc(unsigned char c){
	static unsigned char prev_c = 0;
	static uint16_t tmp_p;
	if((prev_c == '\r') && (c != '\n')){
		uu_putc('\n');
	}
	tmp_p = (tx_buff_wp + 1) & (UU_BUFF_LEN - 1);
	if(tmp_p != tx_buff_rp){
		UU_UART->CR1 &= ~USART_CR1_TXEIE;
		tx_buff[tmp_p] = c;
		tx_buff_wp = tmp_p;
		UU_UART->CR1 |= USART_CR1_TXEIE;
		prev_c = c;
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
	UU_UART->CR1 |= USART_CR1_RXNEIE;
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
			rx_buff[tmp_p] = rx_tmp;
			if(rx_tmp == UU_NL_TXT){
				nl_cnt++;
			}
			rx_buff_wp = tmp_p;
			uu_putc(rx_tmp);
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

	if(int_state & USART_ISR_IDLE){
	}

	if(int_state & USART_ISR_ORE){
		rx_ore = true;
		UU_UART->ICR = USART_ICR_ORECF;
	}
}

void uu_proc_command(void){
	static char str[UU_CSL_STR_LEN];
	static char cmd[UU_CSL_CMD_LEN];
	static char* parg;
	static int32_t argv[UU_CSL_ARG_MAX];
	static int32_t argc = 0;
	uint32_t str_len = 0;
	uint32_t cmd_len = 0;
	bool done = false;

	if (nl_cnt > 0) { //受信バッファに改行コードが含まれるか
		xgets(str, UU_CSL_STR_LEN); //受信バッファから読み出し
		cmd_len = strcspn(str, " "); //最初のスペース文字までの長さ=コマンド長を取得
		if (cmd_len > 0 && cmd_len < UU_CSL_CMD_LEN) { //コマンド長が規定値か
			strncpy(cmd, str, cmd_len); //コマンドを使いやすいようにコピー
			cmd[cmd_len] = '\0';

			str_len = strlen(str);
			parg = &str[cmd_len + 1];
			argc = 0;
			while (parg < str + str_len) { //コマンド引数を処理
				if (!xatoi(&parg, &argv[argc])) {
					argv[argc] = 0;
					break;
				} else {
					if (argc < UU_CSL_ARG_MAX - 1) {
						argc++;
					} else {
						break;
					}
				}
			}
		}
		str2upper(cmd); //コマンドを大文字に変換
		if(!strcmp(cmd, "TEST")) { //コマンドの動作理解用テストコマンド
			xprintf("Test command\r\n");
			for (uint16_t i = 0; i < argc; i++) {
				xprintf("Arg%d:%d\r\n", i, argv[i]);
			}
			done = true;
		}else if(!strcmp(cmd, "?")){
			xputs("Built in commands\r\n");
			xputs("TEST\r\n");
			xputs("Usage : TEST arg1 arg2 ...\r\n");
			xputs("Console test command. The command echo back arguments.\r\n");
			xputs("\r\n\r\nAddon commands\r\n");
			for(uint16_t i = 0; i < command_cnt; i++){
				if(i != 0){
					xputs("--------------------\r\n");
				}
				xputs("Command : ");
				xputs(commands[i].cmd_name);
				xputs("\r\n");
				xputs("Usage : ");
				xputs(commands[i].help_msg);
				xputs("\r\n");
			}
			done = true;
		}else {
			for(uint16_t i = 0; i < command_cnt; i++){
				if(!strcmp(cmd, commands[i].cmd_name)){
					if(!commands[i].func(argc, argv)){
						xputs(commands[i].help_msg);
					}
					done = true;
					break;
				}
			}
		}
		if(!done){
			xputs("UNDEF\r\n");
		}
		xputs(">");
	}
}

bool uu_push_command(UU_ConsoleCommand* pcmd){
	if(command_cnt >= UU_MAX_COMMAND_NUM){
		return false;
	}

	commands[command_cnt].cmd_name = pcmd->cmd_name;
	commands[command_cnt].func = pcmd->func;
	commands[command_cnt].help_msg = pcmd->help_msg;

	command_cnt++;
	return true;
}

static int32_t str2upper(char* str) {
	int32_t i;
	for (i = 0; *(str + i) != '\0'; i++) {
		if (*(str + i) >= 'a' && *(str + i) <= 'z')
			*(str + i) -= 0x20;
	}
	return i;
}
