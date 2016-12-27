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
      i=120;  // �Լ�����
      while(i--) ;    
   }
}
void delay_fast(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  // �Լ�����
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
        GPIO_SetBits(GPIOA, GPIO_Pin_15);//ȫ��
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
* ��    �ƣ�void RCC_Configuration(void)
* ��    �ܣ�ϵͳʱ������Ϊ72MHZ�� ����ʱ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/ 
void RCC_Configuration(void){

  SystemInit();	  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}


/****************************************************************************
* ��    �ƣ�void NVIC_Configuration(void)
* ��    �ܣ��ж�Դ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void NVIC_Configuration(void)
{
  /*  �ṹ����*/
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* ���ȼ��� ˵������ռ���ȼ����õ�λ�����������ȼ����õ�λ��  */    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	  
  
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	    //USB�����ȼ��ж�����
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//��ռ���ȼ� 1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//�����ȼ�Ϊ1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;			//USB�����ȼ��ж�����
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//��ռ���ȼ� 1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//�����ȼ�Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

 
}


void IRQHandler(void)   //�ж�ִ�к���
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
* ��    �ƣ�int main(void)
* ��    �ܣ�������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//��ʹ��jlink�ӿ�
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //��ʼ��GPIOʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //��ʼ��GPIOʱ��
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
    GPIO_SetBits(GPIOA, GPIO_Pin_15);//ȫ��
    
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
    TIM3_Int_Init(99,71);  //�ж�1us
    /*
    TIM3_Int_Init(4999,7199);
    ((4999+1)*(7199+1))/7200=5000  10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms
    
    Tout = ( (arr + 1) * (psc + 1) ) / Tclk
    Tclk��TIM3 ������ʱ��Ƶ�ʣ���λΪ Mhz��
    Tout��TIM3 ���ʱ�䣨��λΪ us�� 
    */
    
  RCC_Configuration(); 		//����ϵͳʱ�� 
  USB_Disconnect_Config();	//����USB���ӿ�����    	   
  NVIC_Configuration();     //USBͨ���ж�����
  USB_Init();				//USB��ʼ��
  while (1)
  {
    mode_set();
    light_kira();
  }
}
//ͨ�ö�ʱ��3�жϳ�ʼ��

//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
    
    //��ʱ��TIM3��ʼ��
    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ    
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

    //�ж����ȼ�NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


    TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx                     
}

//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx�����жϱ�־
        IRQHandler();
    }
}
