


#include "Functions.h"
#include "LCD.h"

#define true 1
#define false 0

unsigned long POW ( unsigned char n,unsigned char p)
{
	unsigned char i;
	unsigned  long result=1;
	for (i=0;i<p;i++)
	{
		result=result*n;
	}
		return result;
}
unsigned char Isdigit(unsigned char num)
{
	if(num>=0x30 && num<=0x39)
	{
		return true;
	}
	else
		return false;

}
unsigned char Ismark(unsigned char mark)
{
	if(mark ==0x2b || mark ==0x2a || mark ==0x2d || mark ==0x2f)
	    return true;
	else return false;
}
unsigned char Num_len(unsigned int temp)
{unsigned char count=0;
	 while(temp>9)
 {
	 temp=temp/10;
	 count++;
 }
 count++;
 return count;
}

void Show_Num (unsigned long op,unsigned char flag)
{
	unsigned long temp1;
	unsigned char Init_len,len,sent_data;
	Init_len=Num_len(op);
	if(flag>0)
	{
		uint8_t d='-';
		//HAL_UART_Transmit(&huart3 , &d , sizeof(d) ,100);

		Data('-');

	}
	while(Init_len>0){
	temp1=op;
	len=Num_len(op);
	temp1=(temp1/(POW(10,len-1)));
	op=op-(temp1*(POW(10,len-1)));
	temp1 |=0x30;
	Data(temp1);
  ///sent_data=temp1;
	//HAL_UART_Transmit(&huart3 , &sent_data , sizeof(sent_data) ,100);
	Init_len-=1;
	}}

void Show_Div (double op,unsigned char flag0)
{
	unsigned long Decimal;
	double F_loat=op;
    uint8_t decimal_point='.';
   unsigned char d1 = 0,d2=0;
  unsigned long first_digit,second_digit;
  first_digit = (unsigned long) (F_loat * 10);// 119.5 ----> 119 char
  second_digit= (unsigned long) (F_loat * 100);
  Decimal= (unsigned long) op;
	Show_Num(Decimal,flag0);
	Data(decimal_point);
	//HAL_UART_Transmit(&huart3 , &decimal_point , sizeof(decimal_point) ,100);
   d1 = (first_digit % 10) | 0x30;  //119%10=9
  Data(d1);
   //HAL_UART_Transmit(&huart3 , &d1 , sizeof(d1) ,100);
   d2=(second_digit % 10)| 0x30;
  // HAL_UART_Transmit(&huart3 , &d2 , sizeof(d2) ,100);
  Data(d2);
}




unsigned char Clear_Equall(unsigned char c_e)
{
	if(c_e == 'c') //clear
	{return 0;}
  else if(c_e == '=' ) //equall
	{return 1;}
}






unsigned char RRShift(unsigned char x,unsigned char y)
{ // REG=RRShift(REG,"Shift Amount")
	//x --> Shifted Register
	//y --> The Shift Amount
	unsigned char LSB,i,temp;
	LSB=0x01;
	for ( i=0;i<y;i++)
	{
	temp=LSB & x;
	temp=temp<<7;
	x=x>>1;
	x=x | temp;}
  return x;}



unsigned char RLShift(unsigned char x,unsigned char y)
{ // REG=RLShift(REG,"Shift Amount")
	//x --> Shifted Register
	//y --> The Shift Amount
	unsigned char MSB,i,temp;
	MSB=0x80;
	for ( i=0;i<y;i++)
	{
	temp=MSB & x;
	temp=temp>>7;
	x=x<<1;
	x=x | temp;}
  return x;}



