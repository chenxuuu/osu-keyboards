#include "stm32f10x.h"
#include "delay.h"
#include "stm32f10x_tim.h"
#include "misc.h"
int led = 0;
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);

void IRQHandler(void)   //�ж�ִ�к���
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
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //��ʼ��GPIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    
    TIM3_Int_Init(9,7199);   //1usִ��һ��
    /*
    TIM3_Int_Init(4999,7199);
    ((4999+1)*(7199+1))/7200=5000  10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms
    
    Tout = ( (arr + 1) * (psc + 1) ) / Tclk
    Tclk��TIM3 ������ʱ��Ƶ�ʣ���λΪ Mhz��
    Tout��TIM3 ���ʱ�䣨��λΪ us�� 
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

