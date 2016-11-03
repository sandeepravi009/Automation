
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN 0 */
#include "Functionality.h"
#include "string.h"
#include "stdlib.h"
uint8_t ButtonPress = 0;
extern int TriggerFlag ;
char ComBuf[COMBUF_SIZE];
char Commands[30][90];
int sp,ep,len;
int ComCurPtr = 0;
int CmdCurPtr = 0;
int CmdMaxPtr = 0;
int CmdServPtr = 0;
int ProcessDataFlag = 0;
int SmsNo = 0;
char RecievedSMSFlag = 0;
char GenString[30];
char SMSString[90];
char CMGR_RecievedFlag = 0;
char CMGR_PtrLoc = 0;
char OkFlag = 1;
void ProcessCommands(void);
void InterpretSMS();
void MotorOn(void);
void LightOn(void);
void MotorOff(void);
void LightOff(void);

/* USER CODE END 0 */
uint8_t rxdata,prevdata;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART7_Init(void);
static void MX_USART1_UART_Init(void);

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
//	StartRecieve(&huart1);
	StartRecieve(&huart7);
	StartRecieve(&huart1);
	SendUartData(GSM,"AT\r\n",0);
	SendUartData(DEBUG,"--DBG_PORT--",0);
	
	MotorOn();
	MotorOff();
	LightOn();
	LightOff();
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {
			if (TriggerFlag  == 1)
			{
				TriggerFlag  = 0;
			
			}
			
			if (ButtonPress ==1)
			{
				ButtonPress =0;
				SendUartData(GSM,"AT\r",0);				
			}
			if(ProcessDataFlag)
			{
				ProcessDataFlag = 0;
				ProcessCommands();
				
			}
			if (RecievedSMSFlag == 1)
			{
				RecievedSMSFlag = 0;
				sprintf(GenString,"AT+CMGR=%d\r",SmsNo);
				SendUartData(GSM,GenString,0);
				OkFlag = 0;
			}
			if (CMGR_RecievedFlag == 1 && OkFlag == 1)
			{
				CMGR_RecievedFlag = 0;
				OkFlag = 0;
				strcpy(SMSString,Commands[CMGR_PtrLoc+1]);
				InterpretSMS();
			}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 330;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* UART7 init function */
void MX_UART7_Init(void)
{

  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart7);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI4_IRQn,0,0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);


}

/* USER CODE BEGIN 4 */

char CmdStart = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
		ComBuf[ComCurPtr]=rxdata;
		ComCurPtr++;
		if(ComCurPtr==COMBUF_SIZE)
		{
			sp =0;
			ep =0;
			prevdata = 0;
			ComCurPtr = 0;
		}
		
		StartRecieve(UartHandle);
		
		
		if(!CmdStart)
		{
			if(rxdata != 0x0A && rxdata != 0x0D)
			{
				if( prevdata == 0x0A)
				{
			
			
						sp =ComCurPtr-1;
						CmdStart = 1;
				}
			}			
		}
		
		if(CmdStart)
		{
			if(rxdata == 0x0A)
			{
				ep = (ComCurPtr-2)-1;
				if(ep<sp)								// circular buffer
				{
					len = COMBUF_SIZE-sp;
					memcpy(&Commands[CmdCurPtr],&ComBuf[sp],(len+1));
					
					memcpy(&Commands[CmdCurPtr][len+1],&ComBuf[0],ep);
				}
				else
				{
					len = ep-sp;
					memcpy(&Commands[CmdCurPtr],&ComBuf[sp],(len+1));
				}
				ProcessDataFlag = 1;
				CmdMaxPtr = CmdCurPtr ; 			// this done to store the current location where the last cmd is filled 
				CmdCurPtr++;									// or i will have to reduce by one to process the last cmd
				if(CmdCurPtr == 29)
				{
					CmdCurPtr = 0;
					CmdServPtr = 0;				// Service pointer also should be zero else it ll be higher that where new data enters eg: 0 location and serv ll be at 29
					CmdMaxPtr = 0;
				//	memcpy(Commands,0x00,270);
				}
				
				CmdStart = 0;
			}
			
		}
		
		prevdata = rxdata;
		
//		ComCurPtr++;
//		if(ComCurPtr==COMBUF_SIZE)
//			ComCurPtr = 0;
//		
//		StartRecieve(UartHandle);

}

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
		Error_Handler(ERROR_UART_TX_RX);

}

void EXTI4_IRQHandler()
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	ButtonPress = 1;
		
}

void ProcessCommands()
{
	int val1,val2;
	while(CmdServPtr <= CmdMaxPtr)
	{
		
		if(!(strncmp("OK",Commands[CmdServPtr],2)))
		{
			OkFlag = 1;
		}
		
		if(!(strncmp("+CMTI: \"SM\" ,",Commands[CmdServPtr],11)))
		{
			sscanf(&Commands[CmdServPtr][12],"%d",&SmsNo);
			RecievedSMSFlag = 1;
		}
		
		if(!(strncmp("+CMGR",Commands[CmdServPtr],5)))
		{
			CMGR_PtrLoc = CmdServPtr;
			CMGR_RecievedFlag = 1;
			OkFlag = 0;
		}
		
		CmdServPtr++;
	}
	
}

void InterpretSMS()
{
	if(!(strncmp("MOTOR",&SMSString[0],5)))
	{
		if(!(strncmp("ON",&SMSString[6],2)))
		{
			MotorOn();
		}
		if(!(strncmp("OFF",&SMSString[6],3)))
		{
			MotorOff();
		}
	}
	if(!(strncmp("LIGHT",SMSString,5)))
	{
		if(!(strncmp("ON",&SMSString[6],2)))
		{
			LightOn();
		}
		if(!(strncmp("OFF",&SMSString[6],3)))
		{
			LightOff();
		}
	}
		
	
	
	
}

void MotorOn()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,1);
	
}
void MotorOff()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,0);
	
}
void LightOn()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,1);
	
}
void LightOff()
{
	
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);
}































/* USER CODE END 4 */

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
