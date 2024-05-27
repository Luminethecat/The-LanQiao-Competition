/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "led.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
u8 Uart_buff,RxBuffer[20],Tx,Rx_cnt ;
u8 Display_Flag = 0;
u32 ARR_Num,CCR2_Num ;
u8 Ctrl ;
typedef struct
{
	u8 Last ;
	u8 Now ;
}BaseData;

typedef struct
{
	int B1 ;
	int B2 ;
	int B3 ;
	
	u8 Key ;
}KeyNum;
KeyNum Key ;
KeyNum Realy[3] ;
KeyNum Now[3] ;
KeyNum Next[3] ;
__IO uint32_t KeyTick = 0 ;


u8 Juge_RXData(void)
{
	if(RxBuffer[3] == '-')
	{
		for(u8 i=0 ;i<3;i++)
		{
			if(((RxBuffer[i]>='0') && (RxBuffer[i]<='9')) && ((RxBuffer[i+4]>='0') && (RxBuffer[i+4]<='9')))
				return 1 ;
		}
	}
	else return 0 ;
	
	return 0 ;
}

u8 Entable_Key(void)
{
	if((Realy[0].Key == Key.B1) && (Realy[1].Key == Key.B2) && (Realy[2].Key == Key.B3)) 
		return 1 ;
	else return 0 ;
}
u8 Juge_Right(void)
{
	for(u8 x=0 ;x<3 ;x++)
	{
		if(Now[x].Key == Realy[x].Key)
			return 1 ;
	}
	return 0 ;
}
__IO uint32_t UartTick = 0 ;
void Juge_Uart(void)
{
	if(uwTick - UartTick < 200) return ;
	UartTick = uwTick ;
	
	if(Juge_RXData())
	{
		for(u8 i=0 ;i<3 ;i++)
		{
			Now[i].Key = (RxBuffer[i]-'0') ;
			Next[i].Key = (RxBuffer[i+4]-'0') ;
		}
		if(Juge_Right())
		{
			for(u8 i=0 ;i<3 ;i++)
			{
				Realy[i].Key = Next[i].Key ;
			}
		}

	}
	else goto Clear ;
	
	Clear:
	      memset(RxBuffer,'\0',sizeof(RxBuffer)) ;
}
u8 ENFlag = 0 ;
u8 Alerm_Flag = 0 ;
void Key_Process(void)
{
	if(uwTick - KeyTick<10) return ;
	KeyTick = uwTick ;
	
	Key_Read() ;
	
	if(!Display_Flag)
	{
		if(Trg & 0x01)
		{
			if(Key.B1 == ('@'-'0'))
				Key.B1 = -1 ;
			Key.B1 += 1 ;
			if(Key.B1 > 9) Key.B1 = 0 ;
		}
		if(Trg & 0x02)
		{
			if(Key.B2 == ('@'-'0'))
				Key.B2 = -1 ;
			Key.B2 += 1 ;
			if(Key.B2 > 9) Key.B2 = 0 ;
		}
		if(Trg & 0x04)
		{
			if(Key.B3 == ('@'-'0'))
				Key.B3 = -1 ;
			Key.B3 += 1 ;
			if(Key.B3 > 9) Key.B3 = 0 ;
		}
    }
	
	if(Trg & 0x08)
	{
		if(Entable_Key())
		{
			Alerm_Flag = 0 ;
	        ENFlag = 1 ;
	        Display_Flag = 1 ;
	        memset(RxBuffer,'\0',sizeof(RxBuffer)) ;
		}
		else
		{
			Alerm_Flag += 1 ;
			Display_Flag = 0 ;
			memset(RxBuffer,'\0',sizeof(RxBuffer)) ;
		}
	}


}


void PSD_Show(void)
{
	u8 Display_buff[20] ;
	LCD_DisplayStringLine(Line1,(unsigned char*)"       PSD          ") ;
	
	sprintf((char*)Display_buff,"    B1:%c    ",Key.B1+'0');
	LCD_DisplayStringLine(Line3,Display_buff) ;
	
	sprintf((char*)Display_buff,"    B2:%c    ",Key.B2+'0');
	LCD_DisplayStringLine(Line4,Display_buff) ;
	
	sprintf((char*)Display_buff,"    B3:%c    ",Key.B3+'0');
	LCD_DisplayStringLine(Line5,Display_buff) ;
}

void STA_Show(void)
{
	u8 Display_buff[20] ;
	LCD_DisplayStringLine(Line1,(unsigned char*)"       STA          ") ;
	
	sprintf((char*)Display_buff,"    F:%dHz    ",2000);
	LCD_DisplayStringLine(Line3,Display_buff) ;
	
	sprintf((char*)Display_buff,"    D:%d%%    ",10);
	LCD_DisplayStringLine(Line4,Display_buff) ;
	
	LCD_DisplayStringLine(Line5,(unsigned char*)"                    ") ;
}

void Display_Show(void)
{
	if(Display_Flag) 
	{
		ARR_Num = 499 ;
		CCR2_Num = 50 ;
		STA_Show() ;
		if(ENFlag)
		{
			Key.B1 = ('@'-'0') ;
			Key.B2 = ('@'-'0') ;
			Key.B3 = ('@'-'0') ;
			ENFlag = 0 ;
		}
	}
	else 
	{
		ARR_Num = 999 ;
		CCR2_Num = 500 ;
        PSD_Show() ; 
	}
	TIM2->ARR = ARR_Num ;
	TIM2->CCR2 = CCR2_Num ;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1,&Uart_buff,1) ;
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2) ;
	HAL_TIM_Base_Start_IT(&htim3) ;
	TIM2->ARR = 999 ;
	TIM2->CCR2 = 500 ;
	Realy[0].Key = 1 ;
	Realy[1].Key = 2 ;
	Realy[2].Key = 3 ;
	Key.B1 = ('@'-'0') ;
	Key.B2 = ('@'-'0') ;
	Key.B3 = ('@'-'0') ;
	LCD_Init();
	LED_Control(0x00) ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);

	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Juge_Uart() ;
	  Key_Process() ;
	  Display_Show() ;

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RxBuffer[Rx_cnt++] = Uart_buff ;
	
	HAL_UART_Receive_IT(&huart1,&Uart_buff,1) ;
}
u8 i = 0 ;
u8 x = 0 ;
u8 Flash_Flag = 0 ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(Display_Flag == 1)
		{
			Ctrl |= 0x01 ;
			i++ ;
			if(i == 50)
			{
				i = 0 ;
				Display_Flag = 0 ;
			    Ctrl &= ~0x01 ;
			}
		}
		
		if(Alerm_Flag >= 3)
		{
			x++ ;
			if(x == 50)
			{
				x = 0 ;
				Alerm_Flag = 0 ;
				Ctrl &= ~0x02 ;
			}
			else
			{
				if(!Flash_Flag)
				{
					Ctrl |= 0x02 ;
					Flash_Flag = 1 ;
				}
				else 
				{
					Ctrl &= ~0x02 ;
					Flash_Flag = 0 ;
				}
		    }
		}
		LED_Control(Ctrl) ;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
