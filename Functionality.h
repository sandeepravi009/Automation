#include "stm32f4xx_hal.h"
#include "string.h"

#define ERROR_UART				0x00
#define ERROR_UART_RX			0x01
#define ERROR_UART_TX			0x02
#define ERROR_UART_TX_RX	0x03

#define GSM		0x00
#define DEBUG 0x01

#define COMBUF_SIZE 1024
void Error_Handler(uint8_t ErrorType);
void StartRecieve(UART_HandleTypeDef *UartHandle);
void SendUartData(uint8_t UartNo, char str[],int Length);
