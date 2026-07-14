#ifndef __BSP_PID_H
#define __BSP_PID_H
#include "ti_msp_dl_config.h"
// #include "board.h"
#include "BSP/inc/bsp_tb6612.h"
// #include "oled.h"
 #include "BSP/inc/bsp_encoder.h"
float normalize_angle(float angle);



int Incremental_PID1(int reality,int target);
int Incremental_PID2(int reality,int target);
int Position_PID2(int reality, int target);
int Position_PID1(int reality, int target);



int yuIncrementay_PID1(int reality,int target);
int yuIncrementax_PID1(int reality,int target);
void Turn_In_Place(float current_yaw, int target_yaw, float kp, float kd);
float Moto_Speed(int encoder_cnt,uint16_t ppr,uint16_t ratio,uint16_t cnt_time);
int Rpm_Encoder_Cnt(float rpm,uint16_t ppr,uint16_t ratio,uint16_t cnt_time);
int Num_Encoder_Cnt(float num,uint16_t ppr,float ratio);
void qinling();
int Xianfu_Pwm(int moto);
void Moto_Stop(void);
int Xianfu(int data,int max);
void Set_Pwm2(int moto);//you轮;
void Set_Pwm1(int moto);//左轮;
void pid_INST_IRQHandler(void);
float fwhabs( float c);
int whabs(int c);
extern int Target_Position1; 
extern int Encoder_cnt1;                  
extern int Target_Position2; 
extern int Encoder_cnt2; 

#define AIN1(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN)) )
#define AIN2(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN)) )

#define AIN3(X) ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN)) )
#define AIN4(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN)) )


 #define Amplitude    6200      /* PWM满幅是7200*/
#define Dead_Voltage  600     /* 死区电压 */   
#endif