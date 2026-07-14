#include "BSP/inc/pid.h"
#include "BSP/inc/bsp_tb6612.h"
#include "board.h"
#include "stdint.h"
#include "BSP/inc/huiduwumcu.h"
#include "BSP/inc/xunji.h"
#include "BSP/inc/key.h"
  int why;//测试
 int jiaodu;//目标角度值
 //各种pid
 float Position_KP1=0.34779,Position_KI1=0.00148,Position_KD1=0.29; /* PID系数 */
float Position_KP2=-0.34776,Position_KI2=-0.00128,Position_KD2=-0.27; /* PID系数 */
float Incremental_KP1=-0.4,Incremental_KI1=-0.04275,Incremental_KD1=0.2;   /* 增量式PID系数 */
float Incremental_KP2=-0.4,Incremental_KI2=-0.04275,Incremental_KD2=0.2;   /* 增量式PID系数 */
float Position_KP3=0.2,Position_KI3=0,Position_KD3=-0.27; /* PID系数 */
float Heading_KP=0.5,Heading_KI=0.5,Heading_KD=0;
float yuntaixkp,yuntaixki=0.0008,yuntaixkd;//云台的
float yuntaiykp,yuntaiyki,yuntaiykd;
int yuntaitai=0;//是否用云台
int zhixiantingxia;
 int shifoujiaoduhuan=0;
 int shifoupid=0;
 int shifoujiaodu;
 extern float angle_z;
int Target_Velocity1,Reality_Velocity1;   /* 目标速度，实际速度 */
 int Target_Velocity2,Reality_Velocity2;   /* 目标速度，实际速度 */
 int Target_Position1; 
 int Encoder_cnt1;                  
 int Target_Position2; 
 int Encoder_cnt2; 
 extern uint16_t x_axis ;
extern uint16_t y_axis ;
//定义一个绝对值函数
extern	No_MCU_Sensor sensor;
extern int A,B,C,D;
int whabs(int c)
{
if(c<0)return (-c);
return c;


}
float fwhabs( float c)
{
if(c<0)return (-c);
return c;


}
void pid_INST_IRQHandler(void)
{
   // 如果产生了定时器中断
    if( DL_TimerA_getPendingInterrupt(pid_INST)==DL_TIMER_IIDX_ZERO)
    {
       //灰度
           // No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
	        //LineTrackingControl();
     
    if(shifoujiaoduhuan)//转向
		 {Turn_In_Place(angle_z ,jiaodu, 4,10);
		 }

     Reality_Velocity1 = -Motor_Get_Encoder(0);  //这里等于的是测量的脉冲值
     Reality_Velocity2 =Motor_Get_Encoder(1);

   // LineTrackingControl();//灰度循迹	

    if(shifoupid==1)//直线
	{
    static int Moto1 = 0; 
    static int Moto2 = 0; 
	//第一个轮子
		 
        Encoder_cnt1 += Reality_Velocity1;                         /* 实际脉冲数累积 */
        Moto1 = Position_PID1(Encoder_cnt1,Target_Position1);  /* 位置式位置控制 */        
        Moto1 = Xianfu(Moto1,Target_Velocity1);                    /* 位置环输出限幅 */
       
		 if(whabs( Encoder_cnt1-Target_Position1)<100)                      /* 已经稳定 */
        { 
         Set_Pwm1(whabs( Encoder_cnt1-Target_Position1)*(0.01)*Incremental_PID1(Reality_Velocity1, Moto1)); 

		 if(whabs( Encoder_cnt1-Target_Position1)<40)                      /* 已经稳定 */
        { 
					zhixiantingxia=1;			
			
						Set_Pwm1(0);    
                shifoupid=0;					
					/* 停止输出 */          
   		
        }}
       else
       {
            Moto1 =Incremental_PID1(Reality_Velocity1,Moto1 );  /* 位置PID控制器 */
            Set_Pwm1(Moto1);  				 /* 赋值 */
			zhixiantingxia=0;
				 
       }
			 
				
			//第二个轮子

			
       Encoder_cnt2 += Reality_Velocity2;                        /* 实际脉冲数累积 */
        Moto2 = Position_PID2(Encoder_cnt2,Target_Position2);  /* 位置式位置控制 */  
        Moto2 = Xianfu(Moto2,whabs(Target_Velocity2));                    /* 位置环输出限幅 */
        if(whabs( Target_Position2-Encoder_cnt2)<100)                     /* 已经稳定 */
        {
					 Set_Pwm2(whabs( Encoder_cnt2-Target_Position2)*(0.01)*Incremental_PID2(Reality_Velocity2, Moto2)); 
					 if(whabs( Target_Position2-Encoder_cnt2)<40)                     /* 已经稳定 */
        {
				zhixiantingxia=1;			
				Set_Pwm2(0);                                         /* 停止输出 */            
        }}
        else
       {
			
         Moto2 =Incremental_PID2(Reality_Velocity2, Moto2);  /* 位置PID控制器 *///          
				 Set_Pwm2(Moto2);   /* 赋值 */
				 		
       }  

				
				
   }



		
}
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm1(int moto)//左轮
 {
   
    if(moto>0)      /* 正转 */
    {     AIN1_OUT(1);
        AIN2_OUT(0);
      

         DL_TimerG_setCaptureCompareValue(PWM_0_INST,Xianfu_Pwm(whabs(moto)), GPIO_PWM_0_C1_IDX);    
    }
     if(moto<0)         /* 反转 */
    {
     AIN1_OUT(0);
        AIN2_OUT(1);
           moto=whabs(moto);

         DL_TimerG_setCaptureCompareValue(PWM_0_INST,Xianfu_Pwm(whabs(moto)), GPIO_PWM_0_C1_IDX);    
    }
		 if(moto==0)
		 {
	    AIN1_OUT(0);
        AIN2_OUT(0);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
		 }
     
 
}
void Set_Pwm2(int moto)//you轮
{
    if(moto>0)      /* 正转 */
    {
       
            BIN1_OUT(0);
        BIN2_OUT(1);
        moto=whabs(moto);

          DL_TimerG_setCaptureCompareValue(PWM_0_INST, Xianfu_Pwm(moto), GPIO_PWM_0_C0_IDX);
    }
    if(moto<0)          /* 反转 */
    {
      
        
          BIN1_OUT(1);
        BIN2_OUT(0);
         moto=whabs(moto);
          DL_TimerG_setCaptureCompareValue(PWM_0_INST, Xianfu_Pwm(moto), GPIO_PWM_0_C0_IDX);
    }
		if(moto==0)
		{
		      BIN1_OUT(0);
        BIN2_OUT(0);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
		}
     
    
   
        
    }
 

/**************************************************************************
函数功能：限幅 
入口参数：电机PWM值
返回  值：限制后的值
**************************************************************************/
int Xianfu(int data,int max)
{	
    if(data<-max) data=-max;	
    if(data> max) data= max;	
	
    return data;
}

/**************************************************************************
函数功能：电机停止左右轮都停止
入口参数：无
返回  值：无
**************************************************************************/
void Moto_Stop(void)
{ 
   Set_Pwm2(0);
    AIN1(0);
    AIN2(0);
Set_Pwm1(0);
    AIN3(0);
    AIN4(0);    
}
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：电机PWM值
返回  值：限制后的值
**************************************************************************/
int Xianfu_Pwm(int moto)
{	
    if(moto>0 && moto<Dead_Voltage) moto = Dead_Voltage;
    if(moto<-Amplitude) moto =-Amplitude;	
    if(moto> Amplitude) moto = Amplitude;	
    return moto;
}




//脉冲全部清零重新计数
void qinling()
{
Encoder_cnt1=0;
Encoder_cnt2=0;


}


/**************************************************************************
功    能: 计算实际转速
输    入: encoder_cnt：脉冲数；ppr：码盘数；ratio：减速比；cnt_time：计数时间(ms)
返回  值: 车轮转速 rpm
**************************************************************************/
float Moto_Speed(int encoder_cnt,uint16_t ppr,uint16_t ratio,uint16_t cnt_time)
{
    return ((float)whabs(encoder_cnt) * 60.0f * 1000.0f) /
           (2.0f * (float)ppr * (float)ratio * (float)cnt_time);
}

//位置计算==要转多少转就要多少脉冲通过对比脉冲来得到是否转到了对应的数
/**************************************************************************
功    能: 计算转数对应编码器脉冲数
输    入: num：转数；ppr：码盘数；ratio：减速比
返 回 值: 电机脉冲数 
**************************************************************************/
int Num_Encoder_Cnt(float num,uint16_t ppr,float ratio)
{
    return (int)(num * ratio * (float)ppr * 2.0f);

	
}

//大概就是多少转速对应多少脉冲//但是不太理解这里的意义明明已经求出来了转速
/**************************************************************************
功    能: 计算转速对应编码器脉冲数
输    入: rpm：转速；ppr：码盘数；ratio：减速比；cnt_time：计数时间(ms)
返 回 值: 电机脉冲数 
**************************************************************************/
int Rpm_Encoder_Cnt(float rpm,uint16_t ppr,uint16_t ratio,uint16_t cnt_time)
{
    return (int)((rpm * (float)ratio * (float)ppr * 2.0f *
                   (float)cnt_time) /
                  (60.0f * 1000.0f));
}


/**************************************************************************
函数功能：位置式PID控制器
入口参数：实际位置，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,...,k;
pwm代表输出
**************************************************************************/
// 在位置环PID计算中，增加积分项清零逻辑（左右轮PID函数均需修改）
int Position_PID1(int reality, int target)
{ 	
    static float Bias, Pwm, Last_Bias, Integral_bias = 0.0f;
    
    Bias = (float)target - (float)reality;                            /* 计算偏差 */
    
    // 接近目标时（如误差<10脉冲），清除积分项
    if(fwhabs(Bias) < 200.0f)
    {
        Integral_bias = 0.0f;  // 避免积分累积导致超调
    }
    else
    {
        Integral_bias += Bias;	                        /* 正常累积积分 */
        if(Integral_bias > 5000.0f) Integral_bias = 5000.0f;
        if(Integral_bias < -5000.0f) Integral_bias = -5000.0f;
    }
    
    Pwm = (Position_KP1 * Bias) 
         + (Position_KI1 * Integral_bias) 
         + (Position_KD1 * (Bias - Last_Bias));
    
    Last_Bias = Bias;
    return (int)Pwm;
}
int Position_PID2(int reality, int target)
{ 	
    static float Bias, Pwm, Last_Bias, Integral_bias = 0.0f;
    
    Bias = (float)target - (float)reality;                            /* 计算偏差 */
    
    // 接近目标时（如误差<10脉冲），清除积分项
    if(fwhabs(Bias) < 200.0f)
    {
        Integral_bias = 0.0f;  // 避免积分累积导致超调
    }
    else
    {
        Integral_bias += Bias;	                        /* 正常累积积分 */
        if(Integral_bias > 5000.0f) Integral_bias = 5000.0f;
        if(Integral_bias < -5000.0f) Integral_bias = -5000.0f;
    }
    
    Pwm = (Position_KP1 * Bias) 
         + (Position_KI1 * Integral_bias) 
         + (Position_KD1 * (Bias - Last_Bias));
    
    Last_Bias = Bias;
    return (int)Pwm;
}
/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值，目标值
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
**************************************************************************/
int Incremental_PID1(int reality,int target)
{ 	
	 static float Bias,Pwm,Last_bias=0.0f,Prev_bias=0.0f;
    
	 Bias=(float)target-(float)reality;                                   /* 计算偏差 */
    
	 Pwm += (Incremental_KP1*(Bias-Last_bias))               /* 比例环节 */
           +(Incremental_KI1*Bias)                           /* 积分环节 */
           +(Incremental_KD1*(Bias-2.0f*Last_bias+Prev_bias));  /* 微分环节 */ 
    
     Prev_bias=Last_bias;                                   /* 保存上上次偏差 */
	 Last_bias=Bias;	                                    /* 保存上一次偏差 */
    
	 return (int)Pwm;                                            /* 输出结果 */

}
/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值，目标值
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
**************************************************************************/
int Incremental_PID2(int reality,int target)
{ 	
	 static float Bias,Pwm,Last_bias=0.0f,Prev_bias=0.0f;
    
	 Bias=(float)target-(float)reality;                                   /* 计算偏差 */
    
	 Pwm+= (Incremental_KP2*(Bias-Last_bias))               /* 比例环节 */
           +(Incremental_KI2*Bias)                           /* 积分环节 */
           +(Incremental_KD2*(Bias-2.0f*Last_bias+Prev_bias));  /* 微分环节 */ 
    
     Prev_bias=Last_bias;                                   /* 保存上上次偏差 */
	 Last_bias=Bias;	                                    /* 保存上一次偏差 */
    
	 return (int)Pwm;                                            /* 输出结果 */
	
}


// int tai=0;
// int chushijiao;//每次用角度前调用
// int tai1=1;
// void zhuanxiangjiaodu()
// {
	
// 	if(tai1==0&tai==1)
// {
// chushijiao=angle_z;
// 	tai1=1;
// 	tai=0;

// }
	
// 	if(abs(angle_z-jiaodu)<8)
// {
// 	Moto_Stop(); //先停止输出
//   	tai=1;
// 	tai1=0;
// 	if(Reality_Velocity1==0&Reality_Velocity2==0)
// 	{qinling();
// 	shifoupid=0;
		
// 		shifoujiaodu=0;
	
	
// }

// }


// if(tai==0&tai1==1)
// 		{
// 			shifoupid=0;
// 			if(jiaodu>0&chushijiao>=0)
// 			{
// 				Set_Pwm1(0);
// 			Set_Pwm2(600);}
// 			if(jiaodu<0&chushijiao<=0) 
// 					{Set_Pwm2(0);
// 			Set_Pwm1(-600);}
// 		if(jiaodu<0&chushijiao>0)
// 			{
// 				Set_Pwm1(0);
// 			Set_Pwm2(600);}
// 			if(jiaodu>0&chushijiao<0) 
// 					{Set_Pwm2(0);
// 			Set_Pwm1(-600);}
// 	  }


// }

float normalize_angle(float angle) {
    // 方法1：使用数学模运算（可能需要循环多次）
    if(angle >= 180.0) angle -= 360.0;
    if(angle < -180.0) angle += 360.0;
    return angle;
    
    // 方法2：更高效的单步计算（适用于大多数情况）
    // angle = fmod(angle + 180.0, 360.0);
    // return angle >= 0 ? angle - 180.0 : angle + 180.0;
}
/*转向环
 * 基于陀螺仪的原地转向控制（纯角度控制，不考虑前进）
 * 使机器人围绕中心旋转到指定目标角度
 * 
 * @param current_yaw 当前陀螺仪测量的偏航角
 * @param target_yaw 目标偏航角（如0、90、180等，单位由陀螺仪决定）
 * @param kp 比例系数，控制转向力度
 * @param kd 微分系数，控制转向稳定性
 * @param 5000 最大转向速度限幅（绝对值）
 * @return 是否完成转向（误差小于阈值）
 */
void Turn_In_Place(float current_yaw, int target_yaw, float kp, float kd)
{
	
    static float last_error = 0;  // 上一次的角度误差
    // 标准化目标角度和当前角度
    current_yaw = normalize_angle(current_yaw);
    target_yaw = normalize_angle(target_yaw);
    
    // 计算标准化后的角度误差（确保选择最短路径）
    float error = target_yaw - current_yaw;
    
    // 处理跨180°边界的情况
    if (error > 180.0) {
        error -= 360.0;  // 选择顺时针旋转
    } else if (error < -180.0) {
        error += 360.0;  // 选择逆时针旋转
    }
    
    // 检查是否已到达目标角度（误差小于阈值）
   
    if (fwhabs(error) <= 5) {
      Moto_Stop ();
			//uint32_t pidTick;
			//if(uwTick -pidTick <2000)return ;
			//{
                shifoujiaoduhuan=0;
                //}
            // 转向完成
    }
    else
		{
    // PD控制计算转向输出
    float p_output = kp * error;  // 比例项：与当前误差成正比
    float d_output = kd * (error - last_error);  // 微分项：与误差变化率成正比
    float turn_output = p_output + d_output;  // PD总输出
    
    // 保存当前误差用于下次计算
    last_error = error;
    
    // 输出限幅，防止转向速度过大
    if (turn_output > 5000) {
        turn_output = 5000;
    } else if (turn_output < -5000) {
        turn_output = -5000;
    }
    
    // 应用左右轮速度
   
		Set_Pwm1(turn_output);
		Set_Pwm2(turn_output);

	}
   
}    




 