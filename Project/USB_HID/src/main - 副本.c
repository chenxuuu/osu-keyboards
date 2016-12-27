#include "stm32f10x.h"
#include "delay.h"
#include "stm32f10x_tim.h"
#include "misc.h"
int led = 0;
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);

void IRQHandler(void)   //中断执行函数
{
    /*
    if(led == 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_8);
        led = 1;
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        led = 0;
    }*/
    if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    }else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_8);
    }
}


int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //初始化GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    
    TIM3_Int_Init(9,7199);   //1us执行一次
    /*
    TIM3_Int_Init(4999,7199);
    ((4999+1)*(7199+1))/7200=5000  10Khz的计数频率，计数到5000为500ms
    
    Tout = ( (arr + 1) * (psc + 1) ) / Tclk
    Tclk：TIM3 的输入时钟频率（单位为 Mhz）
    Tout：TIM3 溢出时间（单位为 us） 
    */
    
    //delay_init();
    
    while(1)
    {
        //GPIO_SetBits(GPIOC, GPIO_Pin_13);
        /*
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        delay_ms(25);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        GPIO_ResetBits(GPIOC, GPIO_Pin_14);
        delay_ms(25);
        GPIO_SetBits(GPIOC, GPIO_Pin_14);
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        delay_ms(25);
        GPIO_SetBits(GPIOC, GPIO_Pin_15);*/
        
        /*
        if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        }else
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_8);
        }
        */
        
        
    }
    
}

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
    
    //定时器TIM3初始化
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值    
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


    TIM_Cmd(TIM3, ENABLE);  //使能TIMx                     
}

//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志
        IRQHandler();
    }
}

