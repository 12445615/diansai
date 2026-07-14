#include "BSP/inc/uart.huart.h"

#include "ti_msp_dl_config.h"
#include "board.h"
#include "BSP/inc/bsp_tb6612.h"
#include "oled.h"

#include "BSP/inc/bsp_encoder.h"
#include <stdio.h>
#include "BSP/inc/jy61.h"

//串口的中断服务函数
uint8_t tuoluo_date;
void Tuoluoyi_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(Tuoluoyi_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            //接发送过来的数据保存在变量中
            tuoluo_date = DL_UART_Main_receiveData(Tuoluoyi_INST);
            //将保存的数据再发送出去
            	CopeSerial2Data(tuoluo_date);
          // lc_printf("%d",tuoluo_date);

            break;

        default://其他的串口中断
            break;
    }
}