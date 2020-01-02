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

void gp_config()
{
    GPIO_InitTypeDef GPIO_InitStructure;   
   
#ifdef USE_STM3210C_EVAL     
  /* Enable the USART2 Pins Software Remapping */   
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);     
#elif defined USE_STM3210B_EVAL || defined USE_STM32100B_EVAL   
  /* Enable the USART2 Pins Software Remapping */   
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);   
#endif   
   GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);   
  /* Configure USARTy Rx as input floating */   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_OD;   
  GPIO_Init(GPIOD, &GPIO_InitStructure);   
      
  /* Configure USARTy Tx as alternate function push-pull */   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;   
  GPIO_Init(GPIOD, &GPIO_InitStructure);   


RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);

//RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);


}
//Setup all GPIO not just LCD based ones -- darkspr1te 
void enable_uart2()
{
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
/* Enable UART clock */



gp_config();
 Serial_Init(115200);
 while (0){
  //Serial_Puts(SERIAL_PORT,"UART 1\n\r"); 
  Serial_Puts(SERIAL_PORT_2,"UART 2\n\r");
  Delay_ms(1000);
 }

/*
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
 
  //GPIO_InitTypeDef GPIO_InitStructure;
   //GPIO_PinRemapConfig(GPIO_Remap_USART2|GPIO_PartialRemap_USART3, ENABLE);
 ////GPIO_PinRemapConfig(GPIO_Remap_USART3, ENABLE);
  // Serial_Init(115200);
   //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
while(0)
   {
 

   Serial_Puts(SERIAL_PORT,"M301\n\r"); 
      Delay_ms(500);
   //  Serial_Puts(SERIAL_PORT_2,"M500\n\r"); 
     //   Delay_ms(200);
   //Serial_Puts(SERIAL_PORT_3,"Uart 3 Start\n\r"); 
   Delay_ms(170);
   }
*/



 //GPIO_Remap_USART2 
 /*
 (GPIO_InitTypeDef GPIO_InitStructure;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
GPIOD->BRR=1<<5;
GPIOD->BRR=1<<6;

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
*/


}
void Hardware_GenericInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_init(F_CPUM);  
  OS_TimerInit(9999, F_CPUM-1);  // System clock timer, cycle 10ms
  enable_uart2();
#ifdef DISABLE_DEBUG 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //disable JTAG & SWD
#endif
  
 
#ifdef DISABLE_JTAG
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#endif

 // while(1);
  /*
  */


//gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5|GPIO6);
//
//Enable serial early for debug, UART1 - wifi plug TXD/RXD, UART2 AUX1 TX/RX, UART3 TX/RX Wifi plug 
//

  
  // Serial_Puts(SERIAL_PORT_3,"Uart 3 Start");
  XPT2046_Init();
  W25Qxx_Init();
  LCD_Init();
  readStoredPara();
  LCD_RefreshDirection();  //refresh display direction after reading settings
  GUI_Clear(BLACK);
  GUI_DispString(100, 0, (u8*)"System Start");
  Delay_ms(500);
  scanUpdates();
  //causes a hang but no code is executed , bug 
  //SD_DeInit();

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
