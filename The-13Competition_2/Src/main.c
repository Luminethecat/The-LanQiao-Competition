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
#include "i2c_hal.h"
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
u8 Ctrl ;
typedef struct
{
	u8 Last ;
	u8 Now ;
}BaseData;

typedef struct
{
    u32 Num ;
	double Price ;
	u32 ALL ;
	
	u8 Key ;
}SHOP;
SHOP Shop_X ;
SHOP Shop_Y ;
__IO uint32_t KeyTick = 0 ;

char RX_BUFF[20] ;
__IO uint32_t UartTick = 0 ;
void Juge_Uart(void)
{
	if(uwTick - UartTick < 200) return ;
	UartTick = uwTick ;
	
	if(Uart_buff == '?') 
	{
		sprintf((char *)RX_BUFF,"X:%.1f,Y:%.1f",Shop_X.Price,Shop_Y.Price) ;
		HAL_UART_Transmit(&huart1,(u8 *)RX_BUFF,strlen(RX_BUFF),50) ;
		goto Clear ;
	}
	else goto Clear ;
	Clear:
	      Uart_buff = '\0' ;
}
u8 ENFlag = 0 ;
u8 Alerm_Flag = 0 ;
u8 Display = 0 ;
void Key_Process(void)
{
	if(uwTick - KeyTick<10) return ;
	KeyTick = uwTick ;
	
	Key_Read() ;

	if(Trg & 0x01)
	{
		Display += 1 ;
		if(Display == 3) Display = 0 ;
	}
	if(Trg & 0x02)
	{
		if(Display == 0)
		{
			Shop_X.Num += 1 ;
			if(Shop_X.Num > Shop_X.ALL) Shop_X.Num = 0 ;
		}
		if(Display == 1)
		{
			Shop_X.Price = (Shop_X.Price + 0.1) ;
			if(Shop_X.Price > 2.1) Shop_X.Price = 1.0 ;
			
			EEPROM_Write(2,Shop_X.Price*10) ;
		}
		if(Display == 2) 
		{
			Shop_X.ALL += 1 ;
			EEPROM_Write(0,Shop_X.ALL) ;
		}
	}
	if(Trg & 0x04)
	{
		if(Display == 0)
		{
			Shop_Y.Num += 1 ;
			if(Shop_Y.Num > Shop_Y.ALL) Shop_Y.Num = 0 ;
		}
		if(Display == 1)
		{
			Shop_Y.Price = (Shop_Y.Price + 0.1) ;
			if(Shop_Y.Price > 2.1) Shop_Y.Price = 1.0 ;
			
			EEPROM_Write(3, Shop_Y.Price*10) ;
		}
		if(Display == 2) 
		{
			Shop_Y.ALL += 1 ;
			EEPROM_Write(1,Shop_Y.ALL) ;
		}
	}

	if(Trg & 0x08)
	{
		char buff[20] ;
		Shop_X.ALL -= Shop_X.Num ;
		Shop_Y.ALL -= Shop_Y.Num ;
		EEPROM_Write(1,Shop_Y.ALL) ;
		EEPROM_Write(0,Shop_X.ALL) ;
		Display_Flag = 1 ;
		
		if((Shop_X.ALL == 0) && (Shop_Y.ALL == 0)) Alerm_Flag = 1 ;
		
		sprintf((char *)buff,"X:%d,Y:%d,Z:%.1f",Shop_X.Num,Shop_Y.Num,((Shop_X.Num*Shop_X.Price)+(Shop_Y.Num*Shop_Y.Price))) ;
		HAL_UART_Transmit(&huart1,(u8 *)buff,strlen(buff),50) ;
		Shop_X.Num = 0 ;
		Shop_Y.Num = 0 ;
	}


}


void SHOP_Show(void)
{
	u8 Display_buff[20] ;

	LCD_DisplayStringLine(Line1,(unsigned char*)"        SHOP          ") ;
	
	sprintf((char*)Display_buff,"     X:%d    ",Shop_X.Num);
	LCD_DisplayStringLine(Line3,Display_buff) ;
	
	sprintf((char*)Display_buff,"     Y:%d    ",Shop_Y.Num);
	LCD_DisplayStringLine(Line4,Display_buff) ;
	
}

void PRICE_Show(void)
{
	u8 Display_buff[20] ;
	LCD_DisplayStringLine(Line1,(unsigned char*)"        PRICE         ") ;
	
	sprintf((char*)Display_buff,"     X:%.1f      ",Shop_X.Price);
	LCD_DisplayStringLine(Line3,Display_buff) ;
	
	sprintf((char*)Display_buff,"     Y:%.1f      ",Shop_Y.Price);
	LCD_DisplayStringLine(Line4,Display_buff) ;
	
}

void REP_Show(void)
{
	u8 Display_buff[20] ;
	LCD_DisplayStringLine(Line1,(unsigned char*)"        REP          ") ;
	
	sprintf((char*)Display_buff,"     X:%d      ",Shop_X.ALL);
	LCD_DisplayStringLine(Line3,Display_buff) ;
	
	sprintf((char*)Display_buff,"     Y:%d      ",Shop_Y.ALL);
	LCD_DisplayStringLine(Line4,Display_buff) ;

}
void Display_Show(void)
{
	if(Display == 0 ) SHOP_Show() ;
	if(Display == 1 ) PRICE_Show() ;
	if(Display == 2 ) REP_Show() ;
	
	
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
	TIM2->ARR = 499 ;
	TIM2->CCR2 = 25 ;
	
	Shop_X.Num = 0 ;
	Shop_Y.Num = 0 ;
	I2CInit() ;


	if((EEPROM_Read(11) != 11) && (EEPROM_Read(22) != 22))
	{		
		Shop_X.ALL = 10 ;
		Shop_Y.ALL = 10 ;
		Shop_X.Price = 1.0 ;
		Shop_Y.Price = 1.0 ;
		
		EEPROM_Write(0,10) ;
		EEPROM_Write(1,10) ;
		EEPROM_Write(2,10) ;
		EEPROM_Write(3,10) ;
		
		
		EEPROM_Write(11,11);
		EEPROM_Write(22,22);
	}
	else 
	{
		Shop_X.ALL = EEPROM_Read(0) ;
		Shop_Y.ALL = EEPROM_Read(1) ;
		Shop_X.Price = EEPROM_Read(2)/10.f ;
		Shop_Y.Price = EEPROM_Read(3)/10.f ;

	}
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
	  Display_Show() ;
	  Key_Process() ;
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
			TIM2->CCR2 = 150 ;
			Ctrl |= 0x01 ;
			i++ ;
			if(i == 50)
			{
				i = 0 ;
				Display_Flag = 0 ;
			    Ctrl &= ~0x01 ;
				TIM2->CCR2 = 25 ;
			}
		}
		
		if(Alerm_Flag)
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
