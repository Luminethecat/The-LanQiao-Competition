#include "led.h"

void LED_Control(u8 Ctrl)
{
	HAL_GPIO_WritePin(GPIOC,0xff00,GPIO_PIN_SET) ;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET) ;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET) ;
	
	HAL_GPIO_WritePin(GPIOC,Ctrl<<8,GPIO_PIN_RESET) ;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET) ;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET) ;
}
