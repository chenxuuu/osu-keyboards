#define GLOBALS
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "stdio.h"	   
#include "demo.h"
#include "hw_config.h"
#include "delay.h"
#include "stm32f10x_tim.h"
#include "misc.h"
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
int led = 1;

extern void USB_Disconnect_Config(void);
__IO uint8_t PrevXferComplete = 1;

int config_mode = 0, key_bit = 0, key_buff = 0, light_time = 0, ledcount = 0, R, G, B, light_choice = 0,color = 1;

void delay(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=120;  // 自己定义
      while(i--) ;    
   }
}
void delay_fast(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  // 自己定义
      while(i--) ;    
   }
}


void mode_set()
{
    if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) && key_bit == 0)
    {
        delay(50);
        if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
        {
            key_bit = 1;
            if(config_mode == 0)
            {
                config_mode = 0;
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
                GPIO_SetBits(GPIOB, GPIO_Pin_4);
                GPIO_SetBits(GPIOB, GPIO_Pin_9);
                
                GPIO_SetBits(GPIOA, GPIO_Pin_13);
                GPIO_ResetBits(GPIOA, GPIO_Pin_14);
                GPIO_SetBits(GPIOA, GPIO_Pin_15);
                
                light_time = 500;
                delay(1000);
                config_mode = 1;
            }
            else if(config_mode == 1)
            {
                config_mode = 0;
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
                GPIO_ResetBits(GPIOB, GPIO_Pin_4);
                GPIO_ResetBits(GPIOB, GPIO_Pin_9);
                GPIO_ResetBits(GPIOA, GPIO_Pin_13);
                GPIO_ResetBits(GPIOA, GPIO_Pin_14);
                GPIO_ResetBits(GPIOA, GPIO_Pin_15);
                delay(500);
                config_mode = 2;
            }
            else if(config_mode == 2)
            {
                config_mode = 0;
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
                GPIO_SetBits(GPIOB, GPIO_Pin_4);
                GPIO_SetBits(GPIOB, GPIO_Pin_9);
                GPIO_SetBits(GPIOA, GPIO_Pin_13);
                GPIO_SetBits(GPIOA, GPIO_Pin_14);
                GPIO_SetBits(GPIOA, GPIO_Pin_15);
                //delay(500);
                config_mode = 3;
            }else if(config_mode == 3)
            {
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
                GPIO_SetBits(GPIOB, GPIO_Pin_4);
                GPIO_SetBits(GPIOB, GPIO_Pin_9);
                GPIO_SetBits(GPIOA, GPIO_Pin_13);
                GPIO_SetBits(GPIOA, GPIO_Pin_14);
                GPIO_SetBits(GPIOA, GPIO_Pin_15);
                config_mode = 0;
            }
        }
    }
    else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
    {
        key_bit = 0;
    }
}
void light(int x)
{
    if(x == 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);//全灭
    }
    else if(config_mode == 1 && x == 1 && color == 1)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
        //color++;
    }
    else if(config_mode == 1 && x == 2 && color == 1)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_ResetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
        //color++;
    }
    
    else if(config_mode == 1 && x == 1 && color == 2)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
        //color++;
    }
    else if(config_mode == 1 && x == 2 && color == 2)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_ResetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
        //color++;
    }
    
    else if(config_mode == 1 && x == 1 && color == 3)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
        //color = 1;
    }
    else if(config_mode == 1 && x == 2 && color == 3)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        //color = 1;
    }
    /*
    else if(config_mode == 2 && x == 1)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
    }
    else if(config_mode == 2 && x == 2)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    }
    
    else if(config_mode == 3 && x == 1)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_3);
        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
    }
    else if(config_mode == 3 && x == 2)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
        GPIO_ResetBits(GPIOA, GPIO_Pin_14);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
    }
    */
}

void light_kira()
{
    if(config_mode == 1)
    {
        if(light_time != 0)
        {
            light_time -= 5;
            light(light_choice);
            delay(light_time);
            light(0);
            delay(500 - light_time);
        }
        else if(light_time <=5)
        {
            light(0);
        }
    }
    else if(config_mode == 3)
    {
        if(ledcount >= 6000)
        {
            ledcount = 0;
        }
        else if(ledcount < 2000)
        {
            R = 100 - ledcount/20;
            G = ledcount/20;
            B = 0;
            ledcount += 1;
        }
        else if(ledcount < 4000)
        {
            R = 0;
            G = 100 - (ledcount - 2000)/20;
            B = (ledcount - 2000)/20;
            ledcount += 1;
        }
        else if(ledcount < 6000)
        {
            R = (ledcount - 4000)/20;
            G = 0;
            B = 100 - (ledcount - 4000)/20;
            ledcount += 1;
        }

        GPIO_ResetBits(GPIOB, GPIO_Pin_3);
        GPIO_ResetBits(GPIOA, GPIO_Pin_14);
        delay(R);
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        GPIO_SetBits(GPIOA, GPIO_Pin_14);

        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        delay(G);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
            
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
        GPIO_ResetBits(GPIOA, GPIO_Pin_13);
        delay(B);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        GPIO_SetBits(GPIOA, GPIO_Pin_13);
    }
}

void color_change()
{
    if(color == 3)
        color = 1;
    else
        color++;
}

/****************************************************************************
* 名    称：void RCC_Configuration(void)
* 功    能：系统时钟配置为72MHZ， 外设时钟配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/ 
void RCC_Configuration(void){

  SystemInit();	  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}


/****************************************************************************
* 名    称：void NVIC_Configuration(void)
* 功    能：中断源配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void NVIC_Configuration(void)
{
  /*  结构声明*/
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* 优先级组 说明了抢占优先级所用的位数，和子优先级所用的位数  */    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	  
  
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	    //USB低优先级中断请求
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//抢占优先级 1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//子优先级为1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;			//USB高优先级中断请求
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//抢占优先级 1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

 
}


void IRQHandler(void)   //中断执行函数
{
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) && GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8))
        {
            if(key_buff != 0)
            {
                Joystick_Send(0, 0);
                //light_choice = 0;
                //light_time = 0;
                key_buff = 0;
            }
        }
        else if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) && !GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8))
        {
            if(key_buff == 1)
            {
                Joystick_Send(0, 0x1D);
                light_choice = 1;
                light_time = 500;
                color_change();
                key_buff = 3;
            }
            if(key_buff == 2)
            {
                Joystick_Send(0, 0x1B);
                light_choice = 2;
                light_time = 500;
                color_change();
                key_buff = 3;
            }
        }
        else if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
        {
            if(key_buff != 1)
            {
                Joystick_Send(0, 0x1B);
                light_choice = 2;
                light_time = 500;
                color_change();
                key_buff = 1;
            }
        }
        else if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8))
        {
            if(key_buff != 2)
            {
                Joystick_Send(0, 0x1D);
                light_choice = 1;
                light_time = 500;
                color_change();
                key_buff = 2;
            }
        }
}

/****************************************************************************
* 名    称：int main(void)
* 功    能：主函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//不使用jlink接口
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //初始化GPIO时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //初始化GPIO时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    GPIO_SetBits(GPIOB, GPIO_Pin_3);
    GPIO_SetBits(GPIOB, GPIO_Pin_4);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    //GPIO_SetBits(GPIOA, GPIO_Pin_9);
    GPIO_SetBits(GPIOA, GPIO_Pin_13);
    GPIO_SetBits(GPIOA, GPIO_Pin_14);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);//全灭
    
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    TIM3_Int_Init(99,71);  //中断1us
    /*
    TIM3_Int_Init(4999,7199);
    ((4999+1)*(7199+1))/7200=5000  10Khz的计数频率，计数到5000为500ms
    
    Tout = ( (arr + 1) * (psc + 1) ) / Tclk
    Tclk：TIM3 的输入时钟频率（单位为 Mhz）
    Tout：TIM3 溢出时间（单位为 us） 
    */
    
  RCC_Configuration(); 		//设置系统时钟 
  USB_Disconnect_Config();	//设置USB连接控制线    	   
  NVIC_Configuration();     //USB通信中断配置
  USB_Init();				//USB初始化
  while (1)
  {
    mode_set();
    light_kira();
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
