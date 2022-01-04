///////////////////////////////////////////////////////////////////////////////
//
// IAR ANSI C/C++ Compiler V7.40.3.8902/W32 for ARM       19/Jul/2016  18:34:37
// Copyright 1999-2015 IAR Systems AB.
//
//    Cpu mode     =  thumb
//    Endian       =  little
//    Source file  =  E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\main.c
//    Command line =  
//        E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\main.c -D
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
//    List file    =  E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\Debug\List\main.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__SystemLibrary", "DLib"
        AAPCS BASE,INTERWORK
        PRESERVE8
        REQUIRE8

        #define SHT_PROGBITS 0x1

        EXTERN GPIO_Init
        EXTERN GPIO_ToggleBits
        EXTERN RCC_APB2PeriphClockCmd
        EXTERN SystemCoreClock
        EXTERN Time

        PUBLIC Delay
        PUBLIC GPIO_Configuration
        PUBLIC GPIO_InitStructure
        PUBLIC main
        
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
        
// E:\STM32F1\GPIO_ICViet\GPIO_ICViet\GPIO\main.c
//    1 #include "stm32f10x.h"
//    2 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//    3 GPIO_InitTypeDef  GPIO_InitStructure;
GPIO_InitStructure:
        DS8 4
//    4 
//    5 void GPIO_Configuration(void);
//    6 void Delay(__IO uint32_t nCount);
//    7 extern __IO uint32_t Time;
//    8 

        SECTION `.text`:CODE:NOROOT(2)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function main
        THUMB
//    9 int main()
//   10 {
main:
        PUSH     {R4,R5,LR}
          CFI R14 Frame(CFA, -4)
          CFI R5 Frame(CFA, -8)
          CFI R4 Frame(CFA, -12)
          CFI CFA R13+12
        SUB      SP,SP,#+4
          CFI CFA R13+16
//   11  GPIO_Configuration();
        MOVS     R1,#+1
        MOVS     R0,#+24
        BL       RCC_APB2PeriphClockCmd
          CFI FunCall RCC_APB2PeriphClockCmd
        LDR.N    R5,??DataTable2
        LDR.N    R4,??DataTable2_1  ;; 0x40011000
        MOV      R0,#+8192
        STRH     R0,[R5, #+0]
        MOV      R1,R5
        MOVS     R0,#+3
        STRB     R0,[R5, #+2]
        MOVS     R0,#+16
        STRB     R0,[R5, #+3]
        MOV      R0,R4
        BL       GPIO_Init
          CFI FunCall GPIO_Init
        MOVS     R0,#+1
        STRH     R0,[R5, #+0]
        MOV      R1,R5
        MOVS     R0,#+72
        STRB     R0,[R5, #+3]
        LDR.N    R0,??DataTable2_2  ;; 0x40010800
        BL       GPIO_Init
//   12  SysTick_Config(SystemCoreClock / 1000); 
          CFI FunCall GPIO_Init
        LDR.N    R0,??DataTable2_3
        LDR      R0,[R0, #+0]
        MOV      R1,#+1000
        UDIV     R0,R0,R1
        CMP      R0,#+16777216
        BCS.N    ??main_0
        LSLS     R0,R0,#+8
        LDR.N    R1,??DataTable2_4  ;; 0xe000e010
        LSRS     R0,R0,#+8
        SUBS     R0,R0,#+1
        STR      R0,[R1, #+4]
        MOVS     R0,#+240
        STRB     R0,[R1, #+3347]
        MOVS     R0,#+0
        STR      R0,[R1, #+8]
        MOVS     R0,#+7
        STR      R0,[R1, #+0]
//   13  while(1)        
//   14  {
//   15   GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
??main_0:
        MOV      R1,#+8192
        MOV      R0,R4
        BL       GPIO_ToggleBits
//   16   Delay(500);
          CFI FunCall GPIO_ToggleBits
        MOV      R0,#+500
        STR      R0,[SP, #+0]
        LDR      R1,[SP, #+0]
        LDR.N    R0,??DataTable2_5
        STR      R1,[R0, #+0]
??main_1:
        LDR      R1,[R0, #+0]
        CMP      R1,#+0
        BNE.N    ??main_1
        B.N      ??main_0
//   17  }
//   18 }
          CFI EndBlock cfiBlock0

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function GPIO_Configuration
        THUMB
//   19 void GPIO_Configuration(void)
//   20 {
GPIO_Configuration:
        PUSH     {R4,LR}
          CFI R14 Frame(CFA, -4)
          CFI R4 Frame(CFA, -8)
          CFI CFA R13+8
//   21  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);
        MOVS     R1,#+1
        MOVS     R0,#+24
        BL       RCC_APB2PeriphClockCmd
//   22  
//   23  /* Configure PB0 PB1 in output pushpull mode */
//   24  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
          CFI FunCall RCC_APB2PeriphClockCmd
        LDR.N    R4,??DataTable2
        MOV      R0,#+8192
        STRH     R0,[R4, #+0]
//   25  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   26  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//   27  GPIO_Init(GPIOC, &GPIO_InitStructure);
        MOV      R1,R4
        MOVS     R0,#+3
        STRB     R0,[R4, #+2]
        MOVS     R0,#+16
        STRB     R0,[R4, #+3]
        LDR.N    R0,??DataTable2_1  ;; 0x40011000
        BL       GPIO_Init
//   28  /* Configure PA0 in input mode */
//   29  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
          CFI FunCall GPIO_Init
        MOVS     R0,#+1
        STRH     R0,[R4, #+0]
//   30  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//   31  GPIO_Init(GPIOA, &GPIO_InitStructure);                           
        MOV      R1,R4
        MOVS     R0,#+72
        STRB     R0,[R4, #+3]
        POP      {R4,LR}
          CFI R4 SameValue
          CFI R14 SameValue
          CFI CFA R13+0
        LDR.N    R0,??DataTable2_2  ;; 0x40010800
        B.W      GPIO_Init
          CFI FunCall GPIO_Init
//   32 }
          CFI EndBlock cfiBlock1

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function Delay
          CFI NoCalls
        THUMB
//   33 void Delay(__IO uint32_t nCount)
//   34 {
Delay:
        PUSH     {R0}
          CFI CFA R13+4
//   35  Time = nCount;
        LDR.N    R0,??DataTable2_5
        LDR      R1,[SP, #+0]
        STR      R1,[R0, #+0]
//   36  while(Time!=0);
??Delay_0:
        LDR      R1,[R0, #+0]
        CMP      R1,#+0
        BNE.N    ??Delay_0
//   37 }
        ADD      SP,SP,#+4
          CFI CFA R13+0
        BX       LR               ;; return
          CFI EndBlock cfiBlock2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2:
        DC32     GPIO_InitStructure

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_1:
        DC32     0x40011000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_2:
        DC32     0x40010800

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_3:
        DC32     SystemCoreClock

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_4:
        DC32     0xe000e010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_5:
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
//   38   
//   39 #ifdef  USE_FULL_ASSERT
//   40 void assert_failed(uint8_t* file, uint32_t line)
//   41  { 
//   42  /* User can add his own implementation to report the file name and line number,
//   43  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   44 
//   45  /* Infinite loop */
//   46  while (1)
//   47  {
//   48  }
//   49 }
//   50 #endif
//   51 
//   52 
//   53 
//   54    
// 
//   4 bytes in section .bss
// 222 bytes in section .text
// 
// 222 bytes of CODE memory
//   4 bytes of DATA memory
//
//Errors: none
//Warnings: none
