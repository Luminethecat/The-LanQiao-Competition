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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "led.h"
#include "key.h"
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
u32 ADC_Value ;
double ADC_Volt ;
__IO uint32_t ADCTick = 0 ;

u32 PA7_D = 0;
u32 PA7_F  = 1000;
u32 D = 0;

u8 Ctrl = 0 ;
u8 led_flag = 0,led_ST = 0;

void ADC_Process(void)
{
	if(uwTick - ADCTick <50) return ;
	ADCTick = uwTick ;
	HAL_ADC_Start_DMA(&hadc2,&ADC_Value,1) ;

	ADC_Volt = (ADC_Value)/4095.0f*3.3f ;
	
	if(ADC_Volt> 0 && ADC_Volt<=1.0) PA7_D = 40 ;
	if(ADC_Volt>1.0 && ADC_Volt < 2.0) PA7_D = ((ADC_Volt - 1.0)*40+40) ;
	if(ADC_Volt>=2.0) PA7_D = 80 ;
	
	if(ADC_Volt>1.0) 
	{
		led_ST = 1 ;
		Ctrl &= ~0x01 ;
	}
	else led_ST = 0 ;
}
__IO uint32_t KeyTick = 0 ;
u8 Key_Flag = 0 ;
void Key_Process(void)
{ 
    if(uwTick - KeyTick <100) return ;
    KeyTick = uwTick ;
	
	Key_Read() ;
	
	if((Trg & 0x01) && (Key_Flag == 0))
	{
		if(PA7_F<= 9000)
		PA7_F += 1000 ;
	}
	if((Trg & 0x02) && (Key_Flag == 0))
	{
		if(PA7_F >=2000)
		PA7_F -= 1000 ;
	}
	if(Trg & 0x04)
		Key_Flag ^= 1 ;
	
	if(Key_Flag == 1) Ctrl |= 0x02 ;
	else Ctrl &= ~0x02 ;

	D = PA7_D*((1000000/PA7_F)/100.0f) ;
	HAL_TIM_PWM_Start_DMA(&htim17,TIM_CHANNEL_1,&D,1) ;
	TIM17->ARR = (1000000/PA7_F)-1;
}
//void PWM_Process(void) 
//{

//}

void LCD_Process(void)
{
	u8 Display_buff[20] ;
	
	LCD_DisplayStringLine(Line1,(unsigned char *)"        DATA        ") ;
	
	sprintf((char*)Display_buff,"    Volt:%.2f       ",ADC_Volt) ;
	LCD_DisplayStringLine(Line3,Display_buff) ;
	
	sprintf((char*)Display_buff,"    D:%d%%          ",PA7_D) ;
	LCD_DisplayStringLine(Line5,Display_buff) ;
	
	sprintf((char*)Display_buff,"    F:%dHz          ",PA7_F) ;
	LCD_DisplayStringLine(Line7,Display_buff) ;


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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM17_Init();

  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1) ;

	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);

	LCD_Clear(Black);
    LED_Control(0x00) ;


	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ADC_Process() ;
	  Key_Process() ;
	  LCD_Process() ;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if(led_ST == 1)
		{
			if(led_flag == 1)
			{
				Ctrl &= ~0x01 ;
				led_flag = 0;
			}
			else
			{
				Ctrl |= 0x01 ;
				led_flag = 1;
			}
	    	LED_Control(Ctrl) ;
        }
		else 
		{
			Ctrl &= ~0x01 ;
			LED_Control(Ctrl) ;
		}

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
