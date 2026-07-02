#ifndef __BAP_UART_H__
#define __BAP_UART_H__

#include "gd32f1x0.h"
#include <stdint.h>

#define COMM_USART              USART1
#define COMM_USART_RCU          RCU_USART1
#define COMM_USART_IRQn         USART1_IRQn
#define COMM_GPIO_PORT          GPIOA
#define COMM_GPIO_PIN           GPIO_PIN_2
#define COMM_GPIO_AF            GPIO_AF_1
#define COMM_BAUDRATE           1000000U

void bap_uart_init(void);
void bap_uart_send(uint8_t *data, uint16_t length);
uint8_t bap_uart_receive(uint8_t *data, uint16_t length);

void usart_putc(uint32_t usart, uint8_t ch);
bool usart_getc_timeout(uint32_t usart, uint8_t *ch, uint32_t timeout_loops);

bool bap_uart_tx_dma_start(const uint8_t *data, uint16_t length);
bool bap_uart_tx_dma_busy(void);

#endif
