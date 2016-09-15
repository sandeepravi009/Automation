
#include "Functionality.h"



extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart7;
extern uint8_t rxdata;
extern 
char CmdStart ;
uint8_t RxBuffer[100];


void SendUartData(uint8_t UartNo, char str[],int Length)
{
	
	if(Length==0)
		Length = strlen(str);
		
	switch(UartNo)
	{
		case DEBUG:
			
			if( HAL_UART_Transmit(&huart7,(uint8_t*)str,Length,100) != HAL_OK)
			{
				Error_Handler(ERROR_UART_TX);
			}
			break;
		
		case GSM :
			
			if( HAL_UART_Transmit(&huart1,(uint8_t*)str,Length,100) != HAL_OK)
			{
				Error_Handler(ERROR_UART_TX);
			}
		break;
	
		
		
		default:
			break;
	}
		
}

void Error_Handler(uint8_t ErrorType)
{
	switch(ErrorType)
	{
		case ERROR_UART:
			
		break;
		
		case ERROR_UART_RX:
			
		break;
		
		case ERROR_UART_TX:
			
		break;
		
		case ERROR_UART_TX_RX:
			
			SendUartData(DEBUG,"E",0);	
			CmdStart = 0;
			StartRecieve(&huart1);
			
			
		break;
		
		default:
			
		break;
		
	}	
	
}

void StartRecieve(UART_HandleTypeDef *UartHandle)
{
	if (HAL_UART_Receive_IT(UartHandle,&rxdata,1) != HAL_OK)
	{
		Error_Handler(ERROR_UART_RX);
	}	
		
}



