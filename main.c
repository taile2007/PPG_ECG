#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stdio.h"
#include "stdint.h"

/* Declare variable*/

volatile uint16_t x,y,i;
volatile uint16_t ADCValue[2];
char USB_Buffer[12];

GPIO_InitTypeDef  						GPIO_InitStructure;
NVIC_InitTypeDef 							NVIC_InitStructure;
TIM_TimeBaseInitTypeDef  			TIM_TimeBaseStructure;
DMA_InitTypeDef 							DMA_InitStructure;
ADC_InitTypeDef 							ADC_InitStructure; 
USART_InitTypeDef							USART_InitStructure;


void GPIO_Configuration(void);
void ADC_Configuration(void);
void TIM2_IRQHandler(void);
void TIM2base_Configuration(void);
void USART_Configuration(unsigned int BaudRate);
void Send_String(char *s);

int main()
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	
	
	GPIO_Configuration();
	ADC_Configuration();
	TIM2base_Configuration();
	USART_Configuration(230400);

 while(1)        
	{		
		if(i)
		{
			i=0;
		sprintf(USB_Buffer,"%d@\n%d$\n", x,y);
		Send_String(USB_Buffer);
		}
	}
}
void Send_String(char *s)
{
	while(*s)
	{
		USART_SendData(USART1, (uint16_t)*s++);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)== RESET);
	}
}
void GPIO_Configuration(void)
{
/******** Configure PB0 PB1 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
/*********** PD2*******************/
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOD, &GPIO_InitStructure);  	
	
/* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		/* Configure ADC Pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void USART_Configuration(unsigned int BaudRate)
{   
			/* USART1 configuration */

    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
    USART_Init(USART1, &USART_InitStructure);
    /* Enable USART1 */
		/* Configure the NVIC Preemption Priority Bits */		
		
    USART_Cmd(USART1, ENABLE);   
}

void ADC_Configuration(void){
	
	/* DMA Configure */ 
	DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCValue; // address of array data
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	
  DMA_InitStructure.DMA_BufferSize = 2;  // kich thuoc mang du lieu tuong ung so phan tu cua ADCValue
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh ;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* DMA1_Stream0 enable */ 
  DMA_Cmd(DMA1_Channel1, ENABLE);
	/* ADC Common Init */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2; //so kenh ADC chuyen doi
  ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5);
	
	/* Enable ADC DMA */ 
	ADC_DMACmd(ADC1, ENABLE);
	/* Enable ADC1 */
 ADC_Cmd(ADC1, ENABLE);
  
 /* Enable ADC1 reset calibration register */   
 ADC_ResetCalibration(ADC1);
 /* Check the end of ADC1 reset calibration register */
 while(ADC_GetResetCalibrationStatus(ADC1));
  
 /* Start ADC1 calibration */
 ADC_StartCalibration(ADC1);
 /* Check the end of ADC1 calibration */
 while(ADC_GetCalibrationStatus(ADC1));
      
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void TIM2base_Configuration(void)
 {   
   /* Time base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = 360-1;     // 36MHz/360 = 100khz
   TIM_TimeBaseStructure.TIM_Period = 200-1;  // 1khz
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
   TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
   TIM_Cmd(TIM2, ENABLE);
   
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);   
 }

void TIM2_IRQHandler(void)
{	
   if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		 {
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
			GPIO_ToggleBits(GPIOB ,GPIO_Pin_1);	
				y = ADCValue[0];
				x = ADCValue[1];	
				i=1;
					 
     }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
 { 
 /* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

 /* Infinite loop */
 while (1)
 {
 }
}
#endif 
