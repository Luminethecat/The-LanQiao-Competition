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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "led.h"
#include "key.h"
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
double ADC2_Volt ;
u8 Ctrl = 0 ;
typedef struct
{
	char PWM_Mode ;
	int Mode ;//0-L.1-H
	float Speed,Old_Speed;
	int K,R ;
	u16 Num ;
	float Max_H,Max_L ;
	u8 LCD_Show ;
	u8 Mode_Flag ;
	u8 Select ;
	u8 Cont_Flag ;
	double D_PWM ;
	u32 PWM_Change,D_Change;
	double D ;
}BaseData ;
BaseData Data ;
#define PWM_H (1000000/8000)-1
#define PWM_L (1000000/4000)-1
#define PI 3.14
void Recd_Show(void) ;
void Para_Show(void) ;
void Data_Show(void) ;
void ADC_Process(void)
{
	HAL_ADC_Start(&hadc2) ;
	ADC2_Volt = HAL_ADC_GetValue(&hadc2)/4096.0f*3.3f ;
	
	if(!Data.Cont_Flag)
	{
		if(ADC2_Volt <= 1.0) Data.D = 10 ;
		else if(ADC2_Volt<3.0) Data.D = ((ADC2_Volt-1)/2*75)+10 ;
		else if(ADC2_Volt>=3.0) Data.D = 85 ;
		
		if(Data.Mode)
			Data.D_PWM = ((PWM_H+1)/100.0f)*Data.D ;
		else
			Data.D_PWM = ((PWM_L+1)/100.0f)*Data.D ;
		
		Ctrl &= ~0x04 ;
	}
	else 
	{
		Ctrl |= 0x04 ;
	}
}
float PA7_D ;
u32 PA7_F,T1,T2;
u8 Tim3_start ;

u8 ERROR_Flag ;
float PA7_D1 ;
u32 PA7_F1,T11,T22,oled_F;
u8 Tim2_start ;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3) 
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			T1 = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1)+1 ;
			PA7_D = T1*100.0f/T2 ;
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			T2 = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2)+1 ;

			PA7_F = 1000000/T2 ;
		}
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2) ;
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1) ;
    }
}

__IO uint32_t KeyTick = 0 ;
void Key_Process(void)
{
	if(uwTick - KeyTick<10) return ;
	KeyTick = uwTick ;
	
	Key_Read() ;
	
	if(Trg & 0x01)
	{
		Data.LCD_Show += 1 ;
		if(Data.LCD_Show == 3) Data.LCD_Show = 0 ;
	}
	if(Trg & 0x02)
	{
		if((Data.LCD_Show == 0) && (Data.Mode_Flag == 0))
		{
			Data.Mode = !Data.Mode ;
			Data.Num += 1 ;
			Data.Mode_Flag = 1 ;
		}
		if(Data.LCD_Show == 1)
		{
			Data.Select = !Data.Select ;
		}
	}
	if(Trg & 0x04)
	{
		if(Data.LCD_Show == 1)
		{
			if(!Data.Select) 
			{
				Data.R += 1 ;
				if(Data.R == 11) Data.R = 1 ;
			}
			else 
			{
				Data.K += 1 ;
				if(Data.K == 11) Data.K = 1 ;
			}
	    }
	}
	
	if(Trg & 0x08)
	{
		if(Data.LCD_Show == 0)
		{
			Data.Cont_Flag = 0 ;
		}
		else if(Data.LCD_Show == 1)
		{
			if(!Data.Select) 
			{
				Data.R -= 1 ;
				if(Data.R == 0) Data.R = 10 ;
			}
			else 
			{
				Data.K -= 1 ;
				if(Data.K == 0) Data.K = 10 ;
			}
     	}
	}
	
	
	if(Data.Mode_Flag == 0)
	{
		if(!Data.Mode) 
		{
			TIM2->ARR = PWM_L ;
			TIM2->CCR2 = Data.D_PWM ;
			Data.PWM_Mode = 'L' ;
		}
		else 
		{
			TIM2->ARR = PWM_H ;
			TIM2->CCR2 = Data.D_PWM ;
			Data.PWM_Mode = 'H' ;
		}
    }
	else 
	{
		if(!Data.Mode) 
		{
			TIM2->ARR = (1000000/(8000-Data.PWM_Change)-1) ;
			TIM2->CCR2 = (1000000/(8000-Data.PWM_Change)/100.0f)*Data.D ;
		}
		else 
		{
			TIM2->ARR = (1000000/(Data.PWM_Change+4000)-1) ;
			TIM2->CCR2 = (1000000/(Data.PWM_Change+4000)/100.0f)*Data.D ;
		}
	}
	
	if(Data.Cont_Flag == 1)  ;
	else  ;
}

__IO uint32_t SpeedTick = 0 ;
void ERROR_Process(void)
{
	Data.Speed = ((PA7_F*2*PI*Data.R)/(100*Data.K)) ;
	if(uwTick - SpeedTick<200) return;
	SpeedTick = uwTick ;
	Data.Old_Speed = Data.Speed ;
		if((PA7_F<8000) && (PA7_F>4000))
	{ 
		ERROR_Flag = 1;
		oled_F = PA7_F ;
	}
	else	ERROR_Flag = 0 ;
}

void LCD_Process(void)
{
	
	if(Data.LCD_Show == 0)
		Data_Show() ;
	else if(Data.LCD_Show == 1) 
		Para_Show() ;
	else if(Data.LCD_Show == 2) 
		Recd_Show() ;
}
void Data_Show(void)
{
	char buff[20] ;                    
	LCD_DisplayStringLine(Line1,(u8 *)"        DATA        ") ;
	
	sprintf((char *)buff,"     M=%c          ",Data.PWM_Mode) ;
	LCD_DisplayStringLine(Line3,(u8 *)buff) ;
	
	sprintf((char *)buff,"     P=%.0f%%      ",PA7_D) ;
	LCD_DisplayStringLine(Line4,(u8 *)buff) ;
	
//	sprintf((char *)buff,"     P=%04dHz      ",PA7_F) ;
//	LCD_DisplayStringLine(Line6,(u8 *)buff) ;
	
	sprintf((char *)buff,"     V=%.1f        ",Data.Speed) ;
	LCD_DisplayStringLine(Line5,(u8 *)buff) ;
}

void Para_Show(void)
{
	char buff[20] ;                    
	LCD_DisplayStringLine(Line1,(u8 *)"        PARA        ") ;
	
	sprintf((char *)buff,"     R=%d        ",Data.R) ;
	LCD_DisplayStringLine(Line3,(u8 *)buff) ;
	
	sprintf((char *)buff,"     K=%d        ",Data.K) ;
	LCD_DisplayStringLine(Line4,(u8 *)buff) ;
	
	LCD_DisplayStringLine(Line5,(u8 *)"                   ") ;
	LCD_DisplayStringLine(Line6,(u8 *)"                   ") ;
}

void Recd_Show(void)
{
	char buff[20] ;                    
	LCD_DisplayStringLine(Line1,(u8 *)"        RECD        ") ;
	
	sprintf((char *)buff,"     N=%d        ",Data.Num) ;
	LCD_DisplayStringLine(Line3,(u8 *)buff) ;
	
	sprintf((char *)buff,"     MH=%.1f        ",Data.Max_H) ;
	LCD_DisplayStringLine(Line4,(u8 *)buff) ;
	
	sprintf((char *)buff,"     ML=%.1f        ",Data.Max_L) ;
	LCD_DisplayStringLine(Line5,(u8 *)buff) ;
	LCD_DisplayStringLine(Line6,(u8 *)"                   ") ;
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
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2) ;
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1) ;
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2) ;
	TIM2->ARR = PWM_L ;
	TIM2->CCR2 = 0;
	Data.K = 1;
	Data.R = 1;
	HAL_TIM_Base_Start_IT(&htim1) ;
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LED_Control(0x00) ;

	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ADC_Process() ;
	  LCD_Process() ;
	  Key_Process() ;
	  ERROR_Process() ;

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

u8 x,i,y,z,c;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if(Data.LCD_Show == 0)
			Ctrl |= 0x01;
		else Ctrl &= ~0x01;
		if(Data.Old_Speed == Data.Speed)
		{
			z++ ;
			if(z == 20)
			{
				if(Data.Mode)
				{
					if(Data.Speed>Data.Max_H)
					      Data.Max_H = Data.Speed ;
				    z = 0;
				}
				else 
				{
					if(Data.Speed>Data.Max_L)
					      Data.Max_L = Data.Speed ;
				    z = 0;
				}
			}
		}
		else z = 0;
		if(ERROR_Flag)
		{
			if(PA7_F == oled_F)
			{
				y++ ;
				if(y % 5 ==0)
				{
					__HAL_TIM_SetCounter(&htim2,0);//�������
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2) ;
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1) ;
					y = 0;
				}
			}
		}
		if(Data.Mode_Flag)
		{
			i++ ;
			c++ ;
			Data.PWM_Change += 80 ;
			if(c == 1)
			{
				Ctrl |= 0x02 ;
			}
			else if(c == 2)
			{
				Ctrl &= ~0x02 ;
				c = 0 ;
			}

			if(i % 50 == 0)
			{
				Data.Mode_Flag = 0 ;
				Data.PWM_Change = 0 ;
				Ctrl &= ~0x02 ;
				i = 0 ;
			}
		}
		if(Data.LCD_Show == 0)
		{
			if(Cont & 0x08)
			{
				x++ ;
				if(x % 20 == 0)
				{
				    Data.Cont_Flag = 1 ;
					x = 0 ;
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
