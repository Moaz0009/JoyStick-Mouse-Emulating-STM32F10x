#include "LCD.h"
//Data pins, A0,A1,A2,A3,A4,A5,A6,A7
//RS PIN C15
//E PIN C14
#define RS GPIO_PIN_15
#define E GPIO_PIN_14
void Com (uint8_t comm )
{
GPIOA->ODR=(GPIOA->ODR & 0xFF00)|(uint16_t)comm;
HAL_GPIO_WritePin(GPIOC, RS, RESET);
HAL_GPIO_WritePin(GPIOC, E, SET);
HAL_GPIO_WritePin(GPIOC, E, RESET);
HAL_Delay(10);
}
void Data(uint8_t Data_ )
{
GPIOA->ODR=(GPIOA->ODR & 0xFF00)|(uint16_t)Data_;
HAL_GPIO_WritePin(GPIOC, RS, SET);
HAL_GPIO_WritePin(GPIOC, E, SET);
HAL_GPIO_WritePin(GPIOC, E, RESET);
HAL_Delay(10);
}
void LCD_intialise()
{Com(0x38);
Com(0x0E);
Com(0x01);
}
void Clear_Single_Digit()
{Com(0x10);
Data(' ');
Com(0x10);}
void Show_Message(uint8_t *mess)
{
	Com(0xc0);
		while(*mess!=0x0)
		{
			Data(*mess);
			mess++;
		}

	}

