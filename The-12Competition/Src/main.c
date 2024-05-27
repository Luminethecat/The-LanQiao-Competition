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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
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
u8  RxBuffer[30],uart_buf,rx_cnt ;
	
u8 buff[50];
u8 error[20];
u8 Busy[20] ;
u8 ccc ;
float Money_V = 2.00 ,Money_C = 3.50;
u8 V_Num = 0 ,C_Num = 0 ,ALL = 8;
typedef struct {
    u8 CN[5] ;
    u8 id[5];
    u8 Year ;
    u8 Mouth ;
    u8 Day ;
    u8 Hour ;
    u8 Min ;
    u8 Sec ;

    u8 Flag;
}DataBase;
DataBase NewCar ;
DataBase CarNum[8] ;

typedef struct Money_Car{
    u8 Year;
    u8 Mouth ;
    u8 Day ;
    u8 Hour ;
    u8 Min ;
    u8 Sec ;
}Money_Def;
Money_Def Money ;
Money_Def INCar ;
Money_Def OutCar_Money ;

struct Data{
    u32 Times ;
    double Money ;
}Data;

u8 Juge_RXData(void)
{
  if(rx_cnt !=22)
  return 0 ;

  if(((RxBuffer[0] == 'C') || (RxBuffer[0] == 'V')) && (RxBuffer[1] == 'N') && (RxBuffer[2] == 'B') && (RxBuffer[3] == 'R') && (RxBuffer[4] == ':') && (RxBuffer[9] ==':'))
  {
        u8 i ;
		for(i=10;i<22;i++)
		{
			if((RxBuffer[i]>'9')&&(RxBuffer[i]<'0'))
				return 0;
		}
        return 1 ;
  }
  else 
  return 0 ;
}
u8 Juge_OUTorIN(u8* Car_ID)
{
  for(u8 i=0 ;i<8 ;i++)
  {
    if(strcmp((char*)Car_ID,(char*)CarNum[i].id) == 0)
    return i ;
  }
  return 0xff ;
}

u8 Juge_FreePace(void)
{
  for(u8 i=0 ;i<8;i++)
  {
    if(CarNum[i].Flag == 0) return i ;
  }
  return 0xff ;
}

void Get_Str(u8* Get,u8* Give,u8 Num,u8 Len)
{
  for(u8 i=0 ;i<Len ;i++)
    Get[i] = Give[i+Num] ;
  Get[Len] = '\0' ;
}
__IO uint32_t DataTick = 0;
void Data_process(void)
{
  if(uwTick - DataTick < 100) return ;
  DataTick = uwTick ;


  if( Juge_RXData()) 
  {
    NewCar.Year = (RxBuffer[10] - '0')*10 + (RxBuffer[11] - '0') ;
    NewCar.Mouth = (RxBuffer[12] - '0')*10 + (RxBuffer[13] - '0') ;
    NewCar.Day = (RxBuffer[14] - '0')*10 + (RxBuffer[15] - '0') ;
    NewCar.Hour = (RxBuffer[16] - '0')*10 + (RxBuffer[17] - '0') ;
    NewCar.Min = (RxBuffer[18] - '0')*10 + (RxBuffer[19] - '0') ;
    NewCar.Sec = (RxBuffer[20] - '0')*10 + (RxBuffer[21] - '0') ;

    if((NewCar.Mouth>12) || (NewCar.Day>31) || (NewCar.Hour>24) || (NewCar.Min>60) || (NewCar.Sec>60))
    goto ERROR ;
	
	  Get_Str(NewCar.CN,RxBuffer,0,4) ;
	  Get_Str(NewCar.id,RxBuffer,5,4) ;

		u8 OUT_Num = Juge_OUTorIN(NewCar.id) ;
	  if(OUT_Num == 0xff)
	  {
		u8 Space_Num = Juge_FreePace() ;
		if( Space_Num == 0xff)
		goto busy ;

		Get_Str(CarNum[Space_Num].CN,NewCar.CN,0,4) ;
		Get_Str(CarNum[Space_Num].id,NewCar.id,0,4) ;
		CarNum[Space_Num].Year = NewCar.Year ;
		CarNum[Space_Num].Mouth = NewCar.Mouth ;
		CarNum[Space_Num].Day = NewCar.Day ;
		CarNum[Space_Num].Hour = NewCar.Hour ;
		CarNum[Space_Num].Min = NewCar.Min ;
		CarNum[Space_Num].Sec = NewCar.Sec ;
		CarNum[Space_Num].Flag = 1 ;

		if(CarNum[Space_Num].CN[0] == 'C') C_Num += 1 ;
		else if(CarNum[Space_Num].CN[0] == 'V') V_Num += 1 ;


		ALL = (8 - C_Num - V_Num) ;
	  }
	  if(OUT_Num != 0xff)
	  {
		if(strcmp((char*)NewCar.CN,(char*)CarNum[OUT_Num].CN) != 0)
		goto ERROR ;

		Data.Times = ((NewCar.Year - CarNum[OUT_Num].Year)*365*24) + ((NewCar.Mouth - CarNum[OUT_Num].Mouth)*30*24)+ 
					 ((NewCar.Day - CarNum[OUT_Num].Day)*24) + + (NewCar.Hour - CarNum[OUT_Num].Hour) ;
		if((NewCar.Min - CarNum[OUT_Num].Min) > 0 || (NewCar.Sec - CarNum[OUT_Num].Sec) > 0)
		Data.Times +=1 ;

		if(CarNum[OUT_Num].CN[0] == 'C')
		Data.Money = Data.Times*Money_C ;
		else if(CarNum[OUT_Num].CN[0] == 'V')
		Data.Money = Data.Times*Money_V ;

		
		if(CarNum[OUT_Num].CN[0] == 'C') C_Num -= 1 ;
		else if(CarNum[OUT_Num].CN[0] == 'V') V_Num -= 1 ;


		ALL = (8 - C_Num - V_Num) ;

		sprintf((char*)buff,"%s:%s:%d:%.2f\n",CarNum[OUT_Num].CN,CarNum[OUT_Num].id,Data.Times,Data.Money) ;
    HAL_UART_Transmit_DMA(&huart1,buff,strlen(buff)) ; 
		memset(&CarNum[OUT_Num],0x00,sizeof(CarNum[OUT_Num])) ;
		CarNum[OUT_Num].Flag = 0 ;
	  }
    }
	  	goto Clear ;

  ERROR:
         sprintf((char*)error,"The Date ERROR!\r\n") ;
         HAL_UART_Transmit_DMA(&huart1,error,strlen(error)) ;
  busy:
         sprintf((char*)Busy,"Don't have Space!\r\n") ;
         HAL_UART_Transmit_DMA(&huart1,Busy,strlen(Busy)) ; 
  Clear:
        memset(RxBuffer,0x00,sizeof(RxBuffer)) ;
        rx_cnt = 0;
}

void Data_display(void)
{
  char display_buff[20] ;
	LCD_DisplayStringLine(Line1 ,(unsigned char *)"      Data         ");

  sprintf(display_buff,"   CNBR:%d        ",C_Num) ;
	LCD_DisplayStringLine(Line3 ,(unsigned char *)display_buff);

  sprintf(display_buff,"   VNBR:%d        ",V_Num) ;
	LCD_DisplayStringLine(Line5 ,(unsigned char *)display_buff);

  sprintf(display_buff,"   IDLE:%d        ",ALL) ;
	LCD_DisplayStringLine(Line7 ,(unsigned char *)display_buff);
	
}
void Para_display(void)
{
  char display_buff[20] ;
	LCD_DisplayStringLine(Line1 ,(unsigned char *)"      Para         ");

  sprintf(display_buff,"   CNBR:%.2f        ",Money_C) ;
	LCD_DisplayStringLine(Line3 ,(unsigned char *)display_buff);

  sprintf(display_buff,"   VNBR:%.2f        ",Money_V) ;
	LCD_DisplayStringLine(Line5 ,(unsigned char *)display_buff);
	
	LCD_DisplayStringLine(Line7 ,(unsigned char *)"                   ");
}

void gpio_init() ;
//}
u8 KeyMode = 0 ,PWMFlag = 0;
__IO uint32_t KeyTick = 0;
void Key_Process(void)
{
	if(uwTick - KeyTick <10) return ;
	KeyTick = uwTick ;
	
	Key_Read() ;
	
	if(Trg & 0x01) 
		KeyMode = !KeyMode ;
	if(KeyMode)
	{
		if(Trg & 0x02)
		{
			Money_C += 0.50 ;
			Money_V += 0.50 ;
		}
		if(Trg & 0x04)
		{
			if(Money_C >= 0.50)
			Money_C -= 0.50 ;
			if(Money_V >=0.50)
			Money_V -= 0.50 ;
		}
    }
	if(Trg & 0x08) 
		PWMFlag ^= 1 ;
	
	if(!KeyMode) Data_display() ;
	else Para_display() ;
	
	if(!PWMFlag) 
	{
	  TIM17->CCR1 = 0 ;
	}
	else if(PWMFlag) 
	{

	  TIM17->CCR1 = 100 ;
	}
}
u8 LED_CTRL ;
void LED_Process(void)
{

	if(ALL != 0 )
		LED_CTRL |= 0x01 ;
	else LED_CTRL &= ~0x01 ;
	
	if(PWMFlag)
		LED_CTRL |= 0x02 ;
	else LED_CTRL &= ~0x02 ;
	
	LED_Control(LED_CTRL ) ;
	
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
//    TIM17 ->ARR = 499 ;

	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);

	HAL_UART_Receive_DMA(&huart1,&uart_buf,1);
  LED_Control(0x00) ;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Key_Process() ;
    Data_process() ;
	  LED_Process() ;

    // Data_display() ;
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
	// char buff[20] ;
 RxBuffer[rx_cnt++] = uart_buf ;


  HAL_UART_Receive_DMA(&huart1,&uart_buf,1);
}

// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
// {
//   if(htim->Instance == TIM4)
//    {

//    }
// }

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
