
#include "ti_msp_dl_config.h"
#include "board.h"
#include "BSP/inc/bsp_tb6612.h"
#include "OLED/oled.h"
#include "OLED/bmp.h"
#include "BSP/inc/bsp_encoder.h"
#include <stdio.h>
#include "BSP/inc/bsp_sr04.h"
#include "BSP/inc/gw_grayscale_sensor.h"
#include "BSP/inc/IIC.h"
#include "BSP/inc/huidu.h"
#include "BSP/inc/pid.h"
#include "BSP/inc/jy61.h"
#include "BSP/inc/uart.huart.h"
#include "BSP/inc/key.h"

#include "BSP/inc/huiduwumcu.h"
#include "BSP/inc/xunji.h"
#include "BSP/inc/jidianqi.h"
uint8_t stage[10];
uint8_t a;
extern unsigned char current_data ;    // 当前数据
extern unsigned char last_data; 
uint8_t buffer1[10];
//测试的变量
uint8_t aaa[10];
extern uint8_t test1;
 extern int why;
//外部声明角度的值
extern float angle_z;
int timbiao;
//初始值
//用于匿名上位机返回
float num1=0;//调试的时候统一用num1
float num2=0;
int rpm1=0;
int rpm2=0;
extern int shifoupid;
extern int shifoujiaodu;
extern int jiaodu;
//声明jy61一些命令
char YAWCMD[3] = {0XFF,0XAA,0X52};//进行Z轴角度清零
char ACCCMD[3] = {0XFF,0XAA,0X67};//加速度校准
char SLEEPCMD[3] = {0XFF,0XAA,0X60};
char UARTMODECMD[3] = {0XFF,0XAA,0X61};
char IICMODECMD[3] = {0XFF,0XAA,0X62};
int A=0,B=0,C=0,D=0;
///速度位置声明
extern int Target_Velocity1,Reality_Velocity1;   /* 目标速度，实际速度 */
extern  int Target_Velocity2,Reality_Velocity2;   /* 目标速度，实际速度 */
 extern int Target_Position1; 
 extern int Encoder_cnt1;                  
 extern int Target_Position2; 
 extern int Encoder_cnt2; 

//云台
 extern uint8_t y_axis;
	 extern uint8_t x_axis;
    extern  uint8_t data;
   extern int yuntaitest;

   //灰度
unsigned short Anolog[8]={0};
unsigned short white[8]={2888,2804,2884,2763,2603,2777,2622,2661};//2888-2804-2884-2763-2603-2777-2622-2661
unsigned short black[8]={76,82,89,101,95,93,88,84}; //
unsigned short Normal[8];
unsigned char rx_buff[256]={0};
	No_MCU_Sensor sensor;
		unsigned char Digtal;
		//测试
		uint8_t ceshi[10];


int main(void)
{
    SYSCFG_DL_init();
    Set_Pwm1(0);
    Set_Pwm2(0);
    Motor_Init();
	OLED_Init();
    hc_sr04_Init();
    NVIC_EnableIRQ(Tuoluoyi_INST_INT_IRQN);
	OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示  
  
    //初始状态先设置初始状态再使能
    num1=5;   num2=-5;
    rpm1=30;  rpm2=30;
   shifoupid=1;
   Target_Position1=Num_Encoder_Cnt(num1 ,500,20);  /* 根据转数求出对应脉冲数 */
	Target_Position2=Num_Encoder_Cnt(num2 ,500,20);  /* 根据转数求出对应脉冲数 */
   		//pid增量式//得到目标速度值
    Target_Velocity1=Rpm_Encoder_Cnt(rpm1,500,20,10);   /* 将转速转化为10ms的脉冲数，目标速度 */ 		
     Target_Velocity2=Rpm_Encoder_Cnt(rpm2,500,20,10);   /* 将转速转化为10ms的脉冲数，目标速度 */ 	
     
     	
    //主要控制使能
    NVIC_ClearPendingIRQ(pid_INST_INT_IRQN );
    NVIC_EnableIRQ(pid_INST_INT_IRQN );

    //串口陀螺仪使能
    NVIC_ClearPendingIRQ(Tuoluoyi_INST_INT_IRQN);
    NVIC_EnableIRQ(Tuoluoyi_INST_INT_IRQN);


   //感为
	//初始化传感器，不带黑白值
		No_MCU_Ganv_Sensor_Init_Frist(&sensor);
		No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		Get_Anolog_Value(&sensor,Anolog);
	//此时打印的ADC的值，可用通过这个ADC作为黑白值的校准
	//也可以自己写按键逻辑完成一键校准功能
		//lc_printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
	
		delay_ms(100);
		memset(rx_buff,0,256);
	//得到黑白校准值之后，初始化传感器
		No_MCU_Ganv_Sensor_Init(&sensor,white,black);
	
		delay_ms(100);
//Set_Pwm1(600);   
//Set_Pwm2(-600);   
//紫色灯管开关
    // Set_Relay_Switch(1);
    while (1)
    {

    sprintf(aaa, "%d", Reality_Velocity1*10);
     OLED_ShowString(0,12,aaa,12,1);
     sprintf(aaa, "%d", Target_Velocity1*10);
     OLED_ShowString(0,36,aaa,12,1);
//得到目标位置值pid位置式
    Target_Position1=Num_Encoder_Cnt(num1 ,500,20);  /* 根据转数求出对应脉冲数 */
	Target_Position2=Num_Encoder_Cnt(num2 ,500,20);  /* 根据转数求出对应脉冲数 */
   		//pid增量式//得到目标速度值
    Target_Velocity1=Rpm_Encoder_Cnt(rpm1,500,20,10);   /* 将转速转化为10ms的脉冲数，目标速度 */ 		
     Target_Velocity2=Rpm_Encoder_Cnt(rpm2,500,20,10);   /* 将转速转化为10ms的脉冲数，目标速度 */ 		
	
           //得到陀螺仪的数
          printfshu(); 
       
     	OLED_Refresh();
        anjian();


  

 
    }
}


   //     // //lora发送数据测试
//     send_lora_char(UART_LOAR_INST, 'A');
//  delay_ms(1000);
    //lora接收数据测试
//   
  
//     OLED_ShowString(0,12,"AAA",12,1);
//   OLED_Refresh();
//   ProcessReceivedMessage();


  //双车通讯
//    SendLEDCommand("LED ON");
//     delay_ms(10000);
//     SendLEDCommand("LED OFF");
  

//  uart_lora_sendString(UART_Regs *uart,char* str);

//hc-sr04模块检测
// start_trig_sr04();
// get_distance();

//感为灰度传感器模块检测
    // current_data= GetGraySensorDigitalValue();
    // lc_printf("current_data:%d\r\n",current_data);
    // OLED_Binary(0,0,current_data,16,1);
    // OLED_Refresh();

    

    //循迹模块检测
    // LineTrackingControl();

    //编码器模块检测
    // lc_printf("[  %d  ]  |  [  %d  ]\r\n",Motor_Get_Encoder(0),Motor_Get_Encoder(1));
    // delay_ms(500);

   //spiole屏幕检测
// OLED_ShowPicture(0,0,128,64,BMP1,1);
	
// 		delay_ms(500);
// 		OLED_Clear();
// 		OLED_ShowChinese(0,0,0,16,1);//中
// 		OLED_ShowChinese(18,0,1,16,1);//景
// 		OLED_ShowChinese(36,0,2,16,1);//园
// 		OLED_ShowChinese(54,0,3,16,1);//电
// 		OLED_ShowChinese(72,0,4,16,1);//子
// 		OLED_ShowChinese(90,0,5,16,1);//技
// 		OLED_ShowChinese(108,0,6,16,1);//术
// 		OLED_ShowString(8,16,"ZHONGJINGYUAN",16,1);
// 		OLED_ShowString(20,32,"2014/05/01",16,1);
// 		OLED_ShowString(0,48,"ASCII:",16,1);  
// 		OLED_ShowString(63,48,"CODE:",16,1);
// 		OLED_ShowChar(48,48,t,16,1);//显示ASCII字符	   
// 		t++;
// 		if(t>'~')t=' ';
// 		OLED_ShowNum(103,48,t,3,16,1);
// 		OLED_Refresh();
// 		delay_ms(500);
// 		OLED_Clear();
       
  
		
	

		//电机转动检测tb6612
//         for(int i = 0; i < 1000; i += 50)
//         {
//             AO_Control(1, i);// A端电机转动 速度最大1000
//             BO_Control(1, i);// B端电机转动 速度最大1000

//         //   lc_printf("Distance = %.2fCM\r\n", a);
// 	    //    delay_ms(500);
//         }
//         TB6612_Motor_Stop(); // 停止转动
//         delay_ms(1000);
        // for(int i = 0; i < 1000; i += 50)
        // {
        //     AO_Control(0, i);// A端电机转动 速度最大1000
        //     BO_Control(0, i);// B端电机转动 速度最大1000

        //     lc_printf("Dir[ 0 ]   i[ %d ]\r\n",i);

        //     delay_ms(500);
        // }
        // TB6612_Motor_Stop(); // 停止转动
        // delay_ms(1000);

 
     
    
