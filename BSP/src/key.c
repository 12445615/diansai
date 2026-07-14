#include "board.h"
#include "BSP/inc/key.h"
#define KB1 keyce1
#define KB2 keyce2
#define KB3 keyce3
#define KB4 keyce4
#define KEYPORT KB1|(KB2<<1)|(KB3<<2)|(KB4<<3)|0XF0
extern int A,B,C,D;
unsigned char dan;
unsigned char chang;
unsigned char readpin;
  uint32_t keyce1,keyce2,keyce3,keyce4;
  void keygai()
  {
if(DL_GPIO_readPins(KEY_key_B25_PORT ,KEY_key_B25_PIN )!=0)
{keyce1=1;}
else
{keyce1=0;}
if(DL_GPIO_readPins(KEY_key_B24_PORT ,KEY_key_B24_PIN )!=0)
{keyce2=1;}
else {
keyce2=0;
}
if(DL_GPIO_readPins(KEY_key_A24_PORT ,KEY_key_A24_PIN )!=0)
{keyce3=1;}
else {
keyce3=0;
}
if(DL_GPIO_readPins(KEY_key_A25_PORT ,KEY_key_A25_PIN )!=0)
{keyce4=1;}
else {
keyce4=0;
}
  }
void key_read()
{
readpin =(KEYPORT )^0XFF;//取反操作
dan=readpin&(readpin^chang);
	chang=readpin ;//chang是实时变量

}


void anjian()
{
	keygai();
key_read();
if(dan&0x01)
{

   A=1;

}
if(dan&0x02)
{
	B=1 ;

}
if(dan&0x04)
{
C=1 ;
}
if(dan&0x08)
{
D=1;
}
}

   