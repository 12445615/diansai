/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */
#include "ti_msp_dl_config.h"
#include "BSP/inc/bsp_sr04.h"
#include "board.h"
#include <string.h>
#include <stdio.h>
uint32_t test=0;
float distance = 0;
uint8_t buffer3[20];
float timeDiffUs ; // 因为定时器周期 2.5μs 
float timeDiffSec;

// 全局标志位和数据缓冲区
#define SAMPLE_SIZE 5
volatile bool gDataReady = false;       // 数据就绪标志位
volatile uint32_t gCaptureCnt1 = 0;     // 上升沿捕获值
volatile uint32_t gCaptureCnt2 = 0;     // 下降沿捕获值
float distance_samples[SAMPLE_SIZE];    // 距离样本缓冲区
uint8_t sample_index = 0;               // 样本索引
uint8_t valid_samples = 0;              // 有效样本数
float distance;  

float filter_distance();
void hc_sr04_Init(void)
{
    DL_TimerG_setCoreHaltBehavior(hongwai_INST,DL_TIMER_CORE_HALT_IMMEDIATE);
    NVIC_EnableIRQ(hongwai_INST_INT_IRQN);
    DL_TimerG_startCounter(hongwai_INST);


}

void start_trig_sr04(void)
{
    DL_GPIO_setPins(SR04_PORT,SR04_Trig_PIN);
    delay_us(15);
    DL_GPIO_clearPins(SR04_PORT,SR04_Trig_PIN);
    delay_ms(200);

}



void hongwai_INST_IRQHandler()
{
   
switch(DL_TimerG_getPendingInterrupt(hongwai_INST))
{ 


case DL_TIMERG_IIDX_CC0_UP://这是上升沿捕获，因为是0通道，0通道都是上升沿的捕获
    test++;

   gCaptureCnt1 = DL_TimerG_getCaptureCompareValue(
                    hongwai_INST, DL_TIMER_CC_0_INDEX);//因为上升沿的时候，会存入那个通道对应的计数器值

break;

     case DL_TIMERG_IIDX_CC1_UP://这是上升沿捕获，因为是1通道，1通道都是下降沿的捕获

// DL_GPIO_togglePins(LED_PORT,LED_PIN_PIN);

   gCaptureCnt2 = DL_TimerG_getCaptureCompareValue(
                    hongwai_INST, DL_TIMER_CC_1_INDEX);//因为下降沿的时候，会存入那个通道对应的计数器值
     gDataReady = true;   

break;
default:
break;
}
}



void get_distance(void)
{
    if(gDataReady)                  // 检测到标志位时处理
        {
            gDataReady = false;         // 清除标志位，防止重复处理
            
            // 计算距离（仅在计数值有效时）
            if(gCaptureCnt2 > gCaptureCnt1)
            {
                uint32_t diff = gCaptureCnt2 - gCaptureCnt1;
                float timeDiffUs = diff * 2.5f;          // 定时器周期2.5μs
                float timeDiffSec = timeDiffUs / 1000000.0f;
                distance = (timeDiffSec * 346.7f) / 2 * 100;  // 25℃声速，单位cm
                
                // 存入样本缓冲区
                distance_samples[sample_index] = distance;
                sample_index = (sample_index + 1) % SAMPLE_SIZE;
                if(valid_samples < SAMPLE_SIZE)
                    valid_samples++;
                
                // 样本足够时进行滤波并显示
                if(valid_samples >= 3)
                {
                    float filtered_distance = filter_distance();
                    char buffer3[32];
                    sprintf(buffer3, "Filtered:%.2fcm", filtered_distance);
                    OLED_ShowString(0, 48, buffer3, 12, 1);
                 
                }
                else
                {
                    // 样本不足时显示原始值
                    char buffer3[32];
                    sprintf(buffer3, "Raw:%.2fcm", distance);
                    OLED_ShowString(0, 60, buffer3, 12, 1);
                }
            }
        }
}


// 滤波函数：剔除最大最小值后取平均
float filter_distance(void)
{
    float temp[SAMPLE_SIZE];
    // 复制有效样本到临时数组
    for(int i = 0; i < valid_samples; i++)
        temp[i] = distance_samples[i];
    
    // 冒泡排序（从小到大）
    for(int i = 0; i < valid_samples - 1; i++)
        for(int j = 0; j < valid_samples - i - 1; j++)
            if(temp[j] > temp[j+1])
            {
                float tmp = temp[j];
                temp[j] = temp[j+1];
                temp[j+1] = tmp;
            }
    
    // 计算剔除首尾后的平均值
    float sum = 0;
    int count = (valid_samples > 2) ? (valid_samples - 2) : valid_samples;
    int start = (valid_samples > 2) ? 1 : 0;
    int end = (valid_samples > 2) ? (valid_samples - 1) : valid_samples;
    
    for(int i = start; i < end; i++)
        sum += temp[i];
    
    return sum / count;
}