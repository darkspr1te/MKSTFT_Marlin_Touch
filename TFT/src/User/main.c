#include "includes.h"

HOST  infoHost;  // Information interaction with Marlin
MENU  infoMenu;  // Menu structure

void test_tx_rx()
{
  GPIO_InitTypeDef GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOB, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5;
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
  //GPIOD->BSRR=1<<6 ;
  //GPIOD->BRR=1<<6; 
  //GPIOD->BRR=1<<5 ;

    GPIOD->BSRR=1<<6 ;
       GPIOD->BSRR=1<<5 ;Delay_ms(50);
    GPIOD->BRR=1<<5 ;
while (0)
{
  
 // GPIOD->BSRR=1<<6 ;
   GPIOD->BSRR=1<<5 ;
   Delay_ms(150);
   GPIOD->BRR=1<<6;
   Delay_ms(500);
   GPIOD->BRR=1<<5 ; 
   Delay_ms(1000);
   

}
}
void Hardware_GenericInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_init(F_CPUM);  
  OS_TimerInit(9999, F_CPUM-1);  // System clock timer, cycle 10ms
  
#ifdef DISABLE_DEBUG 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //disable JTAG & SWD
#endif
  
 
#ifdef DISABLE_JTAG
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#endif
/*
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18


*/

  //GPIO_Remap_USART2
  //GPIO_InitTypeDef GPIO_InitStructure;
  
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
 //GPIO_PinRemapConfig(GPIO_Remap_USART2|GPIO_PartialRemap_USART3, ENABLE);
 ////GPIO_PinRemapConfig(GPIO_Remap_USART3, ENABLE);
   Serial_Init(115200);
   
   while(0)
   {
 

   Serial_Puts(SERIAL_PORT,"M301\n\r"); 
      Delay_ms(500);
   //  Serial_Puts(SERIAL_PORT_2,"M500\n\r"); 
     //   Delay_ms(200);
   //Serial_Puts(SERIAL_PORT_3,"Uart 3 Start\n\r"); 
   Delay_ms(170);
   }
  // Serial_Puts(SERIAL_PORT_2,"Testing 2");
  // Serial_Puts(SERIAL_PORT_3,"Testing 3");
  XPT2046_Init();
  W25Qxx_Init();
  LCD_Init();
  readStoredPara();
  LCD_RefreshDirection();  //refresh display direction after reading settings
  GUI_Clear(BLACK);
  GUI_DispString(100, 0, (u8*)"System Start");
  Delay_ms(500);
  scanUpdates();
  //SD_DeInit();
  /*
for (int x=0;x<0xfff;x++)
{
  W25Qxx_EraseSector( x);
  Delay_ms(10);
  Serial_Puts(SERIAL_PORT_2,"write\n\r");
  //Serial_Puts(SERIAL_PORT_2,x);
}
GUI_Clear(BLACK);
  GUI_DispString(100, 0, (u8*)"erase done ");
  //while(1);*/
#if LCD_ENCODER_SUPPORT
  LCD_EncoderInit();
#endif

#ifdef PS_ON_PIN
  PS_ON_Init();
#endif

#ifdef FIL_RUNOUT_PIN
  FIL_Runout_Init();
#endif
//storePara();
  if(readStoredPara() == false) // Read settings parameter
  {    
  TSC_Calibration();
   storePara();
  }
    
 
  infoMenuSelect();
}

int main(void)
{

  SCB->VTOR = VECT_TAB_FLASH;
 
  Hardware_GenericInit();
  
  for(;;)
  {
    (*infoMenu.menu[infoMenu.cur])();
  }
}
