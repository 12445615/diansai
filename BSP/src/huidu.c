#include"BSP/inc/huidu.h"
#include"BSP/inc/hardware_iic.h"
unsigned char  shuju;
char buffer[10];
unsigned char current_data = 0;    // 当前数据
unsigned char last_data = 0; 
unsigned char  GetGraySensorDigitalValue(void) 
{

	   if(Ping()!= 0) return 0x11; 	// 错误码
//	else return 0x00;
//    if(!IIC_WriteByte(0x4F, 0x00, 0xDD)) return 0x22;
    return IIC_Get_Digtal();      // 返回0或1
}