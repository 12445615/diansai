
#include "BSP/inc/bsp_tb6612.h"
#include "oled.h"
#include "board.h"
#include <stdio.h>
#include "BSP/inc/huiduwumcu.h"
#include "BSP/inc/pid.h"
#include "BSP/inc/key.h"
/* 定义常量 */
#define SENSOR_COUNT      8       // 传感器数量
#define BASE_DUTY_CYCLEA   2500    // 基础占空比 (0-100%)
#define BASE_DUTY_CYCLEB   2200    // 基础占空比 (0-100%)
#define MIN_DUTY_CYCLE    0      // 最小占空比
#define MAX_DUTY_CYCLE   6000

// 最大占空比
#define T_STRAIGHT           1    // 直行转弯系数（T=0.5时左右轮速度相等）
#define T_LOST_LEFT          0.5  // 丢失轨迹左转搜索系数
#define T_LOST_RIGHT         1.5    // 丢失轨迹右转搜索系数
#define SEARCH_TIMEOUT       100     // 搜索超时次数
#define SMOOTH_FACTOR        0.8     // 速度平滑系数

/* 全局变量 */
static uint8_t last_direction = 0;    // 上一时刻方向：0=左，1=右
static uint8_t search_counter = 0;    // 搜索尝试计数器
static int last_left_duty = 0;        // 上一次左轮占空比
static int last_right_duty = 0;       // 上一次右轮占空比
int left_duty, right_duty;
float turn_factor1 ;
float turn_factor2 ;
int xunjishifouwancheng;
unsigned char sensor_value;
extern	No_MCU_Sensor sensor;
uint8_t mode=1;
uint8_t time=0;
uint8_t shedingzhi;
extern int A,B,C,D;
/* 主控制函数 */

//注意是不能反转现在相当于，因为已经限制了最小值
void LineTrackingControl(void) {
     sensor_value = Get_Digtal_For_User(&sensor);
	 
     
	 if(A==1)
	 {
shedingzhi=5;
	 }
	 else if(B==1)
	 {
shedingzhi=9;
	 }
	 else if(C==1)
	 {
shedingzhi=13;
	 }
	 	 else if(D==1)
	 {
shedingzhi=17;
	 }

    char buffer[16];
  
    /* 状态检测：全白为丢失轨迹，全黑为十字线 */
    uint8_t is_lost = (sensor_value == 0xff);   // 全1：所有传感器检测到白线（丢失轨迹）
    uint8_t is_cross = (sensor_value == 0x00);  // 全0：所有传感器检测到黑线（十字线）
    extern int t;
    /* 处理丢失轨迹 */
	if(shedingzhi!=0)
	{if(mode==1)
	{
       if (is_lost) 
			{
           
				mode=0;
				time++;

            } 
// if (is_lost||0b00001111||0b00000111||0b00011111) 
// 			{
           
// 				mode=0;
// 				time++;

//             } 
    /* 正常循迹 */
       else {

        /* 根据传感器值确定转弯系数T */
        switch (sensor_value) {

							/*********直线入圈，减小了中间值**********/
                case 0b11000011:
                case 0b11100111:
                turn_factor1 = 1;
				turn_factor2 = 1;
					break;
                  /* 左偏1： */
				case 0b11110111:
				case 0b11100011:
				case 0b11110011:
				turn_factor1 = 1.2;
				turn_factor2 = 1.2;
					break;
				
			
					/*左偏2： */
				case 0b11111011:
				case 0b11110001:
					turn_factor1 = 1.3;
					turn_factor2 = 1.3;
					break;
					/* 左偏3： */
				case 0b11111001:
					turn_factor1 = 1.4 ;
					turn_factor2 = 1.4;
					break;
					/* 左偏4： */
				case 0b11111101:
				case 0b11111000:
					turn_factor1 = 1.5;
					turn_factor2 = 1.5;
					break;

					/* 左偏5： */
				case 0b11111100:
					turn_factor1 = 1.52;
					turn_factor2 = 1.52;
					break;
				
					/* 左偏6： */
				case 0b11111110:
					turn_factor1 = 1.54;
					turn_factor2 = 1.54;
					break;

				
				
              /* 右偏1： */
				case 0b11101111:
				case 0b11000111:
				case 0b11001111:
					turn_factor1 =0.9;
					turn_factor2 =0.9;
					break;
				/* 右偏2： */
	           case 0b11011111:
		       case 0b10001111:
					turn_factor1 = 0.85;
					turn_factor2 = 0.85;
					break;
					/* 右偏3： */
				case 0b10011111:
					turn_factor1 = 0.81;
					turn_factor2 = 0.81;
					break;

				case 0b00001111:
			    case 0b00000111:
			    case 0b00011111:
				turn_factor1 = 1.5;
				turn_factor2 = 0;
				break;

					/* 右偏4： */
				case 0b10111111:
	
					turn_factor1 = 0.7;
					turn_factor2 = 0.7;
					break;

					/* 右偏5： */
				case 0b00111111:
					turn_factor1 = 0.6;
					turn_factor2 = 0.6;
					break;
				
					/* 右偏6： */
				case 0b01111111:
					turn_factor1 = 0.5;
					turn_factor2 = 0.5;
					break;
		}
	}
	}
	else if(mode==0)
		{   
           if (sensor_value == 0b11100111 || 
    sensor_value == 0b11101111 || 
    sensor_value == 0b11110111 || 
    sensor_value == 0b11001111 || 
    sensor_value == 0b11110011) 
	         {
            mode=1;
			
	        }
             else
	        {

		
		    switch(sensor_value)
			
          {
			  case 0b11111111:
		        turn_factor1 =2.8;
				turn_factor2 =1;
				break;
               //此处左偏不左转
			   	/* 左偏4： */
				case 0b11111101:
				case 0b11111000:
					turn_factor1 = 1.1;
					turn_factor2 = 1.1;
					break;
					/* 左偏5： */
				case 0b11111100:
					turn_factor1 =1.15;
					turn_factor2 =1.15;
					break;
					/* 左偏6： */
				case 0b11111110:
				
					turn_factor1 = 1.2;
					turn_factor2 = 1.2;
					break;
         
				

	   }


		}
		}
		else if(mode==2)
		{

		      turn_factor1 = 2;
					turn_factor2 = 0;


		}

       if(time>=shedingzhi)
		{
				shedingzhi=0;			
			 mode=2;
			  time=0;
			
       
        }
	}
	else {
	       turn_factor1 = 2;
			  turn_factor2 = 0;
	}
	
				/* 更新上一时刻方向 */
    // last_direction = (turn_factor1 < T_STRAIGHT) ? 1 : 0;//如果小于，那么就是这样左轮比右快，右偏，为1
    
    /* 计算左右轮比（左轮 = S*(2-T)，右轮 = S*T） */
    left_duty = (int)(BASE_DUTY_CYCLEA * (2 - turn_factor1));
    right_duty = (int)(BASE_DUTY_CYCLEB * turn_factor2);
    
    /* 限制占空比范围 */
    if (left_duty > MAX_DUTY_CYCLE) left_duty = MAX_DUTY_CYCLE;
    if (right_duty > MAX_DUTY_CYCLE) right_duty = MAX_DUTY_CYCLE;
    if (left_duty < MIN_DUTY_CYCLE) left_duty = MIN_DUTY_CYCLE;
    if (right_duty < MIN_DUTY_CYCLE) right_duty = MIN_DUTY_CYCLE;
    
//    /* 速度平滑处理 */
//    left_duty = (int)(last_left_duty * SMOOTH_FACTOR + left_duty * (1 - SMOOTH_FACTOR));
//    right_duty = (int)(last_right_duty * SMOOTH_FACTOR + right_duty * (1 - SMOOTH_FACTOR));
    
//    /* 更新历史速度 */
//    last_left_duty = left_duty;
//    last_right_duty = right_duty;
    
    /* 控制电机 */
   Set_Pwm1(left_duty);//右轮
   Set_Pwm2(-right_duty);//左轮//左边电机为右，右边电机为左边,相当于这里面right_duty对应的是右边的轮子就行
	
    /* 显示实时状态 */
	 sprintf(buffer, "time:%d", time);
    OLED_ShowString(0, 48, buffer, 12,1);
    sprintf(buffer, "L:%d", right_duty);
    OLED_ShowString(25, 24, buffer, 12,1);
    sprintf(buffer, "R:%d",left_duty);
    OLED_ShowString(25, 36, buffer, 12,1);
	OLED_Refresh();
   
    
    
}
