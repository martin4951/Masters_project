






 #include<string.h>
#include<stdio.h>
#include "stm32f407xx.h"
#include "stm32f4xx_adc.h"
#include "scheduler.h"
#include "tasks.h"






RCC_RegDef_t *cclock;
GPIO_RegDef_t *cGPIO;
TIM_TypeDef *cTIMER;

uint16_t ConvertedValueP1;
uint16_t ConvertedValueP2;
uint16_t ConvertedValueP3;

int a = 5;

int count = 0;





unsigned char executeTaskFlag = 0;




char task0[] = "Task0";
char task1[] = "Task1";
char task2[] = "Task2";
char task3[] = "Task3";
char task4[] = "Task4";
char task5[] = "Task5";

//Parameters for counting clock cycles
unsigned long periodT1 = 0;
unsigned long periodT2 = 0;
unsigned long periodT3 = 0;


unsigned long firstEdgeP1 = 0;
unsigned long secondEdgeP1 = 0;
unsigned long firstEdgeP2 = 0;
unsigned long secondEdgeP2 = 0;
unsigned long firstEdgeP3 = 0;
unsigned long secondEdgeP3 = 0;




void delay(void)
{
	for(uint32_t i = 0; i < 2500000; i++);

}

void delay1(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);

}

void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  NVIC->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

//Timer interrupt for calling task execution
void TIM_init(void)
{
	cclock = RCC;
	cGPIO = GPIOA;


	cTIMER = TIM2;


	cclock->APB1ENR |= (1<<0);   //TIM2 enabled
	NVIC_EnableIRQ(TIM2_IRQn);
	cTIMER->CNT=0;
	cTIMER->PSC=0;
	cTIMER->ARR=19999999;
	cTIMER->DIER=0x01;

	cTIMER->CR1 &= 0x00000000;
	cTIMER->CR1 |= 0x00000001;  //Counter enable



}

/*
 * SP1:
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 -> SPI1_SCLK
 * PA4 --> SPI1_NSS
 * Alt mode : 5
 */


void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins1;



	SPIPins1.pGPIOx = GPIOA;
	SPIPins1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins1.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins1);

	//MOSI
	SPIPins1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins1);

	//MISO
	SPIPins1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins1);


	//NSS
	SPIPins1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins1);


}


/*
 * SP2:
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * Alt mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins2;



	SPIPins2.pGPIOx = GPIOB;
	SPIPins2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins2.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins2);

	//MOSI
	SPIPins2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins2);

	//MISO
	SPIPins2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins2);


	//NSS
	SPIPins2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins2);


}


/*
 * SP3:
 * PC11 --> SPI3_MISO
 * PC12 --> SPI3_MOSI
 * PC10 -> SPI3_SCLK
 * PC9 --> SPI3_NSS
 * Alt mode : 5
 */


void SPI3_GPIOInits(void)
{
	GPIO_Handle_t SPIPins3;



	SPIPins3.pGPIOx = GPIOC;
	SPIPins3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins3.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&SPIPins3);

	//MOSI
	SPIPins3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins3);

	//MISO
	SPIPins3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&SPIPins3);


	//NSS
	SPIPins3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&SPIPins3);


}


void SPI1_Inits(void)
{

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;// 2MHz clk
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //SS pin

	SPI_Init(&SPI1handle);
}


void SPI2_Inits(void)
{

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;// 2MHz clk
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //SS pin

	SPI_Init(&SPI2handle);
}

void SPI3_Inits(void)
{

	SPI_Handle_t SPI3handle;

	SPI3handle.pSPIx = SPI3;
	SPI3handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI3handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI3handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;// 2MHz clk
	SPI3handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI3handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI3handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI3handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //SS pin
	SPI_Init(&SPI3handle);
}




//For testing
void GPIO_ButtonInit(void)
{

	GPIO_Handle_t GPIOBtn;

	
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GPIOBtn);
}

//Pins from slave processor for feedback
void GPIO_Pin1Init(void)
{

	GPIO_Handle_t GPIOPin1;


	GPIOPin1.pGPIOx = GPIOB;
	GPIOPin1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIOPin1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOPin1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOPin1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);

	GPIO_Init(&GPIOPin1);
}

void GPIO_Pin2Init(void)
{

	GPIO_Handle_t GPIOPin2;


	GPIOPin2.pGPIOx = GPIOC;
	GPIOPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIOPin2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOPin2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOPin2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC,ENABLE);

	GPIO_Init(&GPIOPin2);
}

void GPIO_Pin3Init(void)
{

	GPIO_Handle_t GPIOPin3;


	GPIOPin3.pGPIOx = GPIOD;
	GPIOPin3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIOPin3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOPin3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOPin3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GPIOPin3);
}

void GPIO_Pin4Init(void)
{

	GPIO_Handle_t GPIOPin4;


	GPIOPin4.pGPIOx = GPIOE;
	GPIOPin4.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIOPin4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOPin4.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOPin4.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GPIOPin4);
}

void GPIO_Pin5Init(void)
{

	GPIO_Handle_t GPIOPin5;


	GPIOPin5.pGPIOx = GPIOE;
	GPIOPin5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOPin5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOPin5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOPin5.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GPIOPin5);
}

void GPIO_Pin6Init(void)
{

	GPIO_Handle_t GPIOPin6;


	GPIOPin6.pGPIOx = GPIOE;
	GPIOPin6.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIOPin6.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOPin6.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOPin6.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GPIOPin6);
}


//For testing
void GPIO_LED(void)
{
GPIO_Handle_t GpioLed;

			GpioLed.pGPIOx = GPIOD;
			GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
			GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
			GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
			GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
			GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

			GPIO_PeriClockControl(GPIOD, ENABLE);

			GPIO_Init(&GpioLed);


}

//Configure ADC to receive current consumption value
void Configure_ADC1(void)
{
GPIO_Handle_t GpioADC1;

			GpioADC1.pGPIOx = GPIOC;
			GpioADC1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
			GpioADC1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
			GpioADC1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
			GpioADC1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

			GPIO_PeriClockControl(GPIOC, ENABLE);

			ADC1_PCLK_EN();

			GPIO_Init(&GpioADC1);


			 ADC_DeInit();

ADC_InitTypeDef ADC_InitStruct1;

			 ADC_InitStruct1.ADC_ScanConvMode=DISABLE;
			 ADC_InitStruct1.ADC_Resolution=ADC_Resolution_12b;
			 ADC_InitStruct1.ADC_ContinuousConvMode=ENABLE;
			 ADC_InitStruct1.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
			 ADC_InitStruct1.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
			 ADC_InitStruct1.ADC_DataAlign=ADC_DataAlign_Right;
			 ADC_InitStruct1.ADC_NbrOfConversion=1;
			 ADC_Init(ADC1, &ADC_InitStruct1);
			 ADC_Cmd(ADC1, ENABLE);
			 ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles);

}


void Configure_ADC2(void)
{
GPIO_Handle_t GpioADC2;

			GpioADC2.pGPIOx = GPIOC;
			GpioADC2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
			GpioADC2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
			GpioADC2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
			GpioADC2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

			GPIO_PeriClockControl(GPIOC, ENABLE);

			ADC2_PCLK_EN();

			GPIO_Init(&GpioADC2);


			 ADC_DeInit();

ADC_InitTypeDef ADC_InitStruct2;

			 ADC_InitStruct2.ADC_ScanConvMode=DISABLE;
			 ADC_InitStruct2.ADC_Resolution=ADC_Resolution_12b;
			 ADC_InitStruct2.ADC_ContinuousConvMode=ENABLE;
			 ADC_InitStruct2.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
			 ADC_InitStruct2.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
			 ADC_InitStruct2.ADC_DataAlign=ADC_DataAlign_Right;
			 ADC_InitStruct2.ADC_NbrOfConversion=1;
			 ADC_Init(ADC2, &ADC_InitStruct2);
			 ADC_Cmd(ADC2, ENABLE);
			 ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 2, ADC_SampleTime_144Cycles);

}


void Configure_ADC3(void)
{
GPIO_Handle_t GpioADC3;

			GpioADC3.pGPIOx = GPIOC;
			GpioADC3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
			GpioADC3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
			GpioADC3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
			GpioADC3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

			GPIO_PeriClockControl(GPIOC, ENABLE);

			ADC3_PCLK_EN();

			GPIO_Init(&GpioADC3);


			 ADC_DeInit();

ADC_InitTypeDef ADC_InitStruct3;

			 ADC_InitStruct3.ADC_ScanConvMode=DISABLE;
			 ADC_InitStruct3.ADC_Resolution=ADC_Resolution_12b;
			 ADC_InitStruct3.ADC_ContinuousConvMode=ENABLE;
			 ADC_InitStruct3.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
			 ADC_InitStruct3.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
			 ADC_InitStruct3.ADC_DataAlign=ADC_DataAlign_Right;
			 ADC_InitStruct3.ADC_NbrOfConversion=1;
			 ADC_Init(ADC3, &ADC_InitStruct3);
			 ADC_Cmd(ADC3, ENABLE);
			 ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 3, ADC_SampleTime_144Cycles);

}



int Get_ADC_Converted_ValueP1()
{
  ADC_SoftwareStartConv(ADC1);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
  {
    return ADC_GetConversionValue(ADC1);
  }
}

int Get_ADC_Converted_ValueP2()
{
  ADC_SoftwareStartConv(ADC2);
  while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC))
  {
    return ADC_GetConversionValue(ADC2);
  }
}

int Get_ADC_Converted_ValueP3()
{
  ADC_SoftwareStartConv(ADC3);
  while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC))
  {
    return ADC_GetConversionValue(ADC3);
  }
}





int main(void)
{

	count = 1;
	a = 6;


	TIM_init();



	GPIO_ButtonInit();

	GPIO_LED();



	Configure_ADC1();
	Configure_ADC2();
	Configure_ADC3();





	SPI2_GPIOInits();

	SPI1_Inits();
	SPI2_Inits();
	SPI3_Inits();

	SPI_SSIConfig(SPI1,ENABLE);
	SPI_SSOEConfig(SPI1, ENABLE);
	SPI_SSIConfig(SPI2,ENABLE);
	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_SSIConfig(SPI3,ENABLE);
	SPI_SSOEConfig(SPI3, ENABLE);



	//Create  tasks
	scheduler_create(t1, 1, 1);
	scheduler_create(t2, 1, 1);
	scheduler_create(t3, 1, 1);
	scheduler_create(t4, 2, 1);
	scheduler_create(t5, 3, 1);
	scheduler_create(t6, 1, 1);




	//Enable feedback interrupts
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI1,ENABLE);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI4,ENABLE);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI3,ENABLE);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI2,ENABLE);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);



	while(1)
	{






	if (executeTaskFlag == 1)
		{
			executeTaskFlag = 0;
			scheduler_execute();

		}




	}



	return 0;

}




void TIM2_IRQHandler(void)
{


	executeTaskFlag = 1;


	if(cTIMER->SR &=0x00000001)
	{
		cGPIO->ODR ^= (1<<5);
		cTIMER->SR &=~1;
	}
}

//For testing
void EXTI0_IRQHandler(void)

{

	GPIO_IRQHandling(GPIO_PIN_NO_0); //clear pending event from exti line
	executeTaskFlag = 1;

}






//PB1
void EXTI1_IRQHandler(void)

{
	GPIO_IRQHandling(GPIO_PIN_NO_1); //Clear interrupt from EXTI line
	firstEdgeP1 = ARM_DWT_CYCCNT;  //Start execution cycle timer
	ConvertedValueP1 = Get_ADC_Converted_ValueP1(); //Sample ADC current value for processor


}

//PC4
void EXTI4_IRQHandler(void)

{
	GPIO_IRQHandling(GPIO_PIN_NO_4); 
	secondEdgeP1 = ARM_DWT_CYCCNT;
	periodT1 = secondEdgeP1 - firstEdgeP1;

}

//PD3
void EXTI3_IRQHandler(void)

{
	GPIO_IRQHandling(GPIO_PIN_NO_3); 
	firstEdgeP2 = ARM_DWT_CYCCNT;
	ConvertedValueP2 = Get_ADC_Converted_ValueP2();

}

//PE2
void EXTI2_IRQHandler(void)

{
	GPIO_IRQHandling(GPIO_PIN_NO_2); 
	secondEdgeP2 = ARM_DWT_CYCCNT;
	periodT2 = secondEdgeP2 - firstEdgeP2;

}

//E5
void EXTI9_5_IRQHandler(void)

{
	GPIO_IRQHandling(GPIO_PIN_NO_5); 
	firstEdgeP3 = ARM_DWT_CYCCNT;
	ConvertedValueP3 = Get_ADC_Converted_ValueP3();

}

//E14
void EXTI15_10_IRQHandler(void)

{
	GPIO_IRQHandling(GPIO_PIN_NO_14); 
	secondEdgeP3 = ARM_DWT_CYCCNT;
	periodT3 = secondEdgeP3 - firstEdgeP3;

}
