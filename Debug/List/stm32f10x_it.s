///////////////////////////////////////////////////////////////////////////////
//
// IAR ANSI C/C++ Compiler V7.40.3.8902/W32 for ARM       19/Jul/2016  18:34:16
// Copyright 1999-2015 IAR Systems AB.
//
//    Cpu mode     =  thumb
//    Endian       =  little
//    Source file  =  E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\stm32f10x_it.c
//    Command line =  
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\stm32f10x_it.c -D
//        USE_STDPERIPH_DRIVER -D STM32F10X_MD_VL -lA
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\Debug\List\ -o
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\Debug\Obj\ --debug
//        --endian=little --cpu=Cortex-M3 -e --fpu=None --dlib_config
//        D:\APP\IAR7.40\arm\INC\c\DLib_Config_Full.h -I
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\Libraries\CMSIS\CM3\CoreSupport\
//        -I
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\
//        -I
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\Libraries\STM32F10x_StdPeriph_Driver\inc\
//        -I E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\ -Ohs -I
//        D:\APP\IAR7.40\arm\CMSIS\Include\
//    List file    =  
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\Debug\List\stm32f10x_it.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__SystemLibrary", "DLib"
        AAPCS BASE,INTERWORK
        PRESERVE8
        REQUIRE8

        #define SHT_PROGBITS 0x1

        PUBLIC BusFault_Handler
        PUBLIC DebugMon_Handler
        PUBLIC HardFault_Handler
        PUBLIC MemManage_Handler
        PUBLIC NMI_Handler
        PUBLIC PendSV_Handler
        PUBLIC SVC_Handler
        PUBLIC SysTick_Handler
        PUBLIC Time
        PUBLIC UsageFault_Handler
        
          CFI Names cfiNames0
          CFI StackFrame CFA R13 DATA
          CFI Resource R0:32, R1:32, R2:32, R3:32, R4:32, R5:32, R6:32, R7:32
          CFI Resource R8:32, R9:32, R10:32, R11:32, R12:32, R13:32, R14:32
          CFI EndNames cfiNames0
        
          CFI Common cfiCommon0 Using cfiNames0
          CFI CodeAlign 2
          CFI DataAlign 4
          CFI ReturnAddress R14 CODE
          CFI CFA R13+0
          CFI R0 Undefined
          CFI R1 Undefined
          CFI R2 Undefined
          CFI R3 Undefined
          CFI R4 SameValue
          CFI R5 SameValue
          CFI R6 SameValue
          CFI R7 SameValue
          CFI R8 SameValue
          CFI R9 SameValue
          CFI R10 SameValue
          CFI R11 SameValue
          CFI R12 Undefined
          CFI R14 SameValue
          CFI EndCommon cfiCommon0
        
// E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\stm32f10x_it.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
//    4   * @author  MCD Application Team
//    5   * @version V3.5.0
//    6   * @date    08-April-2011
//    7   * @brief   Main Interrupt Service Routines.
//    8   *          This file provides template for all exceptions handler and 
//    9   *          peripherals interrupt service routine.
//   10   ******************************************************************************
//   11   * @attention
//   12   *
//   13   * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//   14   * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
//   15   * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
//   16   * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
//   17   * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
//   18   * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//   19   *
//   20   * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
//   21   ******************************************************************************
//   22   */
//   23 
//   24 /* Includes ------------------------------------------------------------------*/
//   25 #include "stm32f10x_it.h"
//   26 
//   27 /** @addtogroup STM32F10x_StdPeriph_Template
//   28   * @{
//   29   */
//   30 
//   31 /* Private typedef -----------------------------------------------------------*/
//   32 /* Private define ------------------------------------------------------------*/
//   33 /* Private macro -------------------------------------------------------------*/
//   34 /* Private variables ---------------------------------------------------------*/

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   35   __IO uint32_t Time=0;
Time:
        DS8 4
//   36 /* Private function prototypes -----------------------------------------------*/
//   37 /* Private functions ---------------------------------------------------------*/
//   38 
//   39 /******************************************************************************/
//   40 /*            Cortex-M3 Processor Exceptions Handlers                         */
//   41 /******************************************************************************/
//   42 
//   43 /**
//   44   * @brief  This function handles NMI exception.
//   45   * @param  None
//   46   * @retval None
//   47   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function NMI_Handler
          CFI NoCalls
        THUMB
//   48 void NMI_Handler(void)
//   49 {
//   50 }
NMI_Handler:
        BX       LR               ;; return
          CFI EndBlock cfiBlock0
//   51 
//   52 /**
//   53   * @brief  This function handles Hard Fault exception.
//   54   * @param  None
//   55   * @retval None
//   56   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function HardFault_Handler
          CFI NoCalls
        THUMB
//   57 void HardFault_Handler(void)
//   58 {
//   59   /* Go to infinite loop when Hard Fault exception occurs */
//   60   while (1)
HardFault_Handler:
??HardFault_Handler_0:
        B.N      ??HardFault_Handler_0
//   61   {
//   62   }
//   63 }
          CFI EndBlock cfiBlock1
//   64 
//   65 /**
//   66   * @brief  This function handles Memory Manage exception.
//   67   * @param  None
//   68   * @retval None
//   69   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function MemManage_Handler
          CFI NoCalls
        THUMB
//   70 void MemManage_Handler(void)
//   71 {
//   72   /* Go to infinite loop when Memory Manage exception occurs */
//   73   while (1)
MemManage_Handler:
??MemManage_Handler_0:
        B.N      ??MemManage_Handler_0
//   74   {
//   75   }
//   76 }
          CFI EndBlock cfiBlock2
//   77 
//   78 /**
//   79   * @brief  This function handles Bus Fault exception.
//   80   * @param  None
//   81   * @retval None
//   82   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function BusFault_Handler
          CFI NoCalls
        THUMB
//   83 void BusFault_Handler(void)
//   84 {
//   85   /* Go to infinite loop when Bus Fault exception occurs */
//   86   while (1)
BusFault_Handler:
??BusFault_Handler_0:
        B.N      ??BusFault_Handler_0
//   87   {
//   88   }
//   89 }
          CFI EndBlock cfiBlock3
//   90 
//   91 /**
//   92   * @brief  This function handles Usage Fault exception.
//   93   * @param  None
//   94   * @retval None
//   95   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function UsageFault_Handler
          CFI NoCalls
        THUMB
//   96 void UsageFault_Handler(void)
//   97 {
//   98   /* Go to infinite loop when Usage Fault exception occurs */
//   99   while (1)
UsageFault_Handler:
??UsageFault_Handler_0:
        B.N      ??UsageFault_Handler_0
//  100   {
//  101   }
//  102 }
          CFI EndBlock cfiBlock4
//  103 
//  104 /**
//  105   * @brief  This function handles SVCall exception.
//  106   * @param  None
//  107   * @retval None
//  108   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function SVC_Handler
          CFI NoCalls
        THUMB
//  109 void SVC_Handler(void)
//  110 {
//  111 }
SVC_Handler:
        BX       LR               ;; return
          CFI EndBlock cfiBlock5
//  112 
//  113 /**
//  114   * @brief  This function handles Debug Monitor exception.
//  115   * @param  None
//  116   * @retval None
//  117   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function DebugMon_Handler
          CFI NoCalls
        THUMB
//  118 void DebugMon_Handler(void)
//  119 {
//  120 }
DebugMon_Handler:
        BX       LR               ;; return
          CFI EndBlock cfiBlock6
//  121 
//  122 /**
//  123   * @brief  This function handles PendSVC exception.
//  124   * @param  None
//  125   * @retval None
//  126   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function PendSV_Handler
          CFI NoCalls
        THUMB
//  127 void PendSV_Handler(void)
//  128 {
//  129 }
PendSV_Handler:
        BX       LR               ;; return
          CFI EndBlock cfiBlock7
//  130 
//  131 /**
//  132   * @brief  This function handles SysTick Handler.
//  133   * @param  None
//  134   * @retval None
//  135   */

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function SysTick_Handler
          CFI NoCalls
        THUMB
//  136   void SysTick_Handler(void)
//  137 {
//  138   if(Time!=0) Time--;
SysTick_Handler:
        LDR.N    R0,??DataTable0
        LDR      R1,[R0, #+0]
        CBZ.N    R1,??SysTick_Handler_0
        LDR      R1,[R0, #+0]
        SUBS     R1,R1,#+1
        STR      R1,[R0, #+0]
//  139 }
??SysTick_Handler_0:
        BX       LR               ;; return
          CFI EndBlock cfiBlock8

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable0:
        DC32     Time

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//  140 
//  141 /******************************************************************************/
//  142 /*                 STM32F10x Peripherals Interrupt Handlers                   */
//  143 /*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
//  144 /*  available peripheral interrupt handler's name please refer to the startup */
//  145 /*  file (startup_stm32f10x_xx.s).                                            */
//  146 /******************************************************************************/
//  147 
//  148 /**
//  149   * @brief  This function handles PPP interrupt request.
//  150   * @param  None
//  151   * @retval None
//  152   */
//  153 /*void PPP_IRQHandler(void)
//  154 {
//  155 }*/
//  156 
//  157 /**
//  158   * @}
//  159   */ 
//  160 
//  161 
//  162 /******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
// 
//  4 bytes in section .bss
// 34 bytes in section .text
// 
// 34 bytes of CODE memory
//  4 bytes of DATA memory
//
//Errors: none
//Warnings: none
