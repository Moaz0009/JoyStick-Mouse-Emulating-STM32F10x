/*
 * LCD.h
 *
 *  Created on: May 5, 2024
 *      Author: NOUR FARES
 */


#ifndef LCD_H_
#define LCD_H_
#include"main.h"
void Com (uint8_t comm );
void Data (uint8_t Data_ );
void LCD_intialise();
void Clear_Single_Digit();
void Show_Message(uint8_t *mess);



#endif /* LCD_H_ */
