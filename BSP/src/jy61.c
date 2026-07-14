#include "BSP/inc/jy61.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "ti_msp_dl_config.h"
#include "board.h"
#include "BSP/inc/bsp_tb6612.h"
#include "oled.h"

#include "BSP/inc/bsp_encoder.h"
#include <stdio.h>
#include "BSP/inc/uart.huart.h"
extern uint8_t tuoluo_date;

int i;
struct SAcc
{
    short a[3];
    short T;
};
struct SGyro
{
    short w[3];
    short T;
};
struct SAngle
{
    short Angle[3];
    short T;
};

uint8_t rx_buff1;
struct SAcc ACC;
struct SAngle Angle;
struct SGyro Gyro;

float angle_x;
float angle_y;
float angle_z;

float acc_x;
float acc_y;
float acc_z;

float gyro_x;
float gyro_y;
float gyro_z;

void CopeSerial2Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
    static unsigned char ucRxCnt = 0;    
    
    ucRxBuffer[ucRxCnt++]=ucData;
    if (ucRxBuffer[0]!=0x55)
    {
        i=4;
        ucRxCnt=0;
        return;
    }
    
    if (ucRxCnt<11) {return;}
    else
    {
        switch(ucRxBuffer[1])
        {
            i=7;
            case 0x51: memcpy(&ACC,&ucRxBuffer[2],8);break;
            case 0x52: memcpy(&Gyro,&ucRxBuffer[2],8);break;
            case 0x53: memcpy(&Angle,&ucRxBuffer[2],8);break;
        }
        ucRxCnt=0;
    }
}

void sendcmd(char data[])
{
    for(int i=0;i<3;i++)
    {


    }
}

void get_data()
{
    acc_x = (float)ACC.a[0]/32768*16;
    acc_y = (float)ACC.a[1]/32768*16;
    acc_z = (float)ACC.a[2]/32768*16;
        
    gyro_x = (float)Gyro.w[0]/32768*2000;
    gyro_y = (float)Gyro.w[1]/32768*2000;
    gyro_z = (float)Gyro.w[2]/32768*2000;
        
    angle_x = (float)Angle.Angle[0]/32768*180;
    angle_y = (float)Angle.Angle[1]/32768*180;
    angle_z = (float)Angle.Angle[2]/32768*180;
}

uint16_t k;
void printfshu()
{
    get_data();
    
  
}