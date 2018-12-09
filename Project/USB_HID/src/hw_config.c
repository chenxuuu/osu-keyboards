/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include "demo.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"	 
#include "usb_pwr.h"
#include "usb_lib.h" 


ErrorStatus HSEStartUpStatus;
extern __IO uint8_t PrevXferComplete;

/*******************************************************************************
* Function Name  : USB_Disconnect_Config
* Description    : Disconnect pin configuration
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Disconnect_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* USB_DISCONNECT_PIN used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

#ifdef STM32L1XX_MD
  Device_Serial0 = *(uint32_t*)(0x1FF80050);
  Device_Serial1 = *(uint32_t*)(0x1FF80054);
  Device_Serial2 = *(uint32_t*)(0x1FF80064);
#else  
  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);
#endif /* STM32L1XX_MD */
  
  Device_Serial0 += Device_Serial2;
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
//  /* Set the device state to suspend */
//  bDeviceState = SUSPENDED;
//
//  /* Clear EXTI Line18 pending bit */
//  EXTI_ClearITPendingBit(KEY_BUTTON_EXTI_LINE);
//
//  /* Request to enter STOP mode with regulator in low power mode */
//  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
  {}
  
#ifdef  STM32F10X_CL
  /* Enable PLL2 */
  RCC_PLL2Cmd(ENABLE);  

  /* Wait till PLL2 is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
  {}
#endif /* STM32F10X_CL */
  
  /* Enable PLL1 */
  RCC_PLLCmd(ENABLE);

  /* Wait till PLL1 is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {}

  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  /* Wait till PLL is used as system clock source */
#ifdef STM32L1XX_MD  
  while (RCC_GetSYSCLKSource() != 0x0C)
#else   
  while (RCC_GetSYSCLKSource() != 0x08)
#endif /* STM32L1XX_MD */ 
  {}  
  
  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}


/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{  
  if (NewState != DISABLE)
  {
    //GPIO_ResetBits(GPIOC, GPIO_Pin_13);
  }
  else
  {
    //GPIO_SetBits(GPIOC, GPIO_Pin_13);
  }	
}

/*******************************************************************************
* Function Name : JoyState.
* Description   : Decodes the Joystick direction.
* Input         : None.
* Output        : None.
* Return value  : The direction value.
*******************************************************************************/
u8 JoyState(void)
{
  /* "right" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOE, JOY_RIGHT))
  {
    return RIGHT;
  }
  /* "left" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOE, JOY_LEFT))
  {
    return LEFT;
  }
  /* "up" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOD, JOY_UP))
  {
    return UP;
  }
  /* "down" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOD, JOY_DOWN))
  {
    return DOWN;
  }
  if (!GPIO_ReadInputDataBit(GPIOC, JOY_LEFT_BUTTON))
  {
    return LEFT_BUTTON;
  }
   if (!GPIO_ReadInputDataBit(GPIOC, JOY_RIGHT_BUTTON))
  {
    return RIGHT_BUTTON;
  }
  /* No key is pressed */
  else
  {
    return 0;
  }
}
#define KEY_SEL    0x01
#define KEY_RIGHT  0x02
#define KEY_LEFT   0x04
#define KEY_DOWN   0x10
#define KEY_UP     0x08
#define KEY_2      0x20
#define KEY_3      0x40
/*******************************************************************************
* Function Name : Joystick_Send.
* Description   : prepares buffer to be sent containing Joystick event infos.
* Input         : Keys: keys received from terminal.
* Output        : None.
* Return value  : None.
*******************************************************************************/
void Joystick_Send(u8 win_buf,u8 key_buf)
{
u8 Mouse_Buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
while(GetEPTxStatus(ENDP1) == EP_TX_VALID);  //check last send is over
/* prepare buffer to send */
Mouse_Buffer[0]=win_buf;    
Mouse_Buffer[2]=key_buf;       
/*copy mouse position info in ENDP1 Tx Packet Memory Area*/
UserToPMABufferCopy(Mouse_Buffer, GetEPTxAddr(ENDP1), 8);
/* enable endpoint for transmission */
SetEPTxValid(ENDP1);
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
