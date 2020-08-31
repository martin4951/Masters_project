/*
 * tasks.c
 *
 *  Created on: 9 Aug 2020
 *      Author: Martino
 */
#include "tasks.h"
#include "stm32f407xx.h"
#include<string.h>
#include<stdio.h>




extern unsigned long periodT1;
extern unsigned long periodT2;
extern unsigned long periodT3;


extern uint16_t ConvertedValueP1;
extern uint16_t ConvertedValueP2;
extern uint16_t ConvertedValueP3;

//Tasks with migration enabled

void t1(void)
{

	if(ConvertedValueP1 <= 2000 && periodT1 <= 1620000)
	{

		SPI_PeripheralControl(SPI2,ENABLE);

		//send task
		SPI_SendData(SPI2,(uint8_t*)task0,strlen(task0));

		//wait till SPI isn't busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable SPI
		SPI_PeripheralControl(SPI2,DISABLE);

	}
	else if(ConvertedValueP1 > 2000 || periodT1 > 1620000)
	{
		SPI_PeripheralControl(SPI1,ENABLE);

		//send task
		SPI_SendData(SPI1,(uint8_t*)task0,strlen(task0));

		//wait till SPI isn't busy
		while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

		//Disable SPI
		SPI_PeripheralControl(SPI1,DISABLE);

	}


}

void t2(void)
{

	if(ConvertedValueP1 <= 2037 && periodT1 <= 2520000)
	{
		SPI_PeripheralControl(SPI2,ENABLE);

		SPI_SendData(SPI2,(uint8_t*)task1,strlen(task1));

		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );


		SPI_PeripheralControl(SPI2,DISABLE);

	}
	else if(ConvertedValueP1 > 2037 || periodT1 > 2520000)
	{
		SPI_PeripheralControl(SPI1,ENABLE);

		SPI_SendData(SPI1,(uint8_t*)task1,strlen(task1));

		while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );


		SPI_PeripheralControl(SPI1,DISABLE);
	}
}

void t3(void)
{
	if(ConvertedValueP1 <= 2044 && periodT1 <= 3420000)
	{
		SPI_PeripheralControl(SPI2,ENABLE);

		SPI_SendData(SPI2,(uint8_t*)task2,strlen(task2));


		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );


		SPI_PeripheralControl(SPI2,DISABLE);

	}
	else if(ConvertedValueP2 > 2044 || periodT2 > 3420000)
	{

		SPI_PeripheralControl(SPI1,ENABLE);

		SPI_SendData(SPI1,(uint8_t*)task2,strlen(task2));


		while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );


		SPI_PeripheralControl(SPI1,DISABLE);


	}
}

void t4(void)
{
	if(ConvertedValueP2 >= 2071 && periodT2 >= 4600000)
	{

		SPI_PeripheralControl(SPI1,ENABLE);

		SPI_SendData(SPI1,(uint8_t*)task3,strlen(task3));

		while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

		SPI_PeripheralControl(SPI1,DISABLE);

	}
	else if(ConvertedValueP2 < 2071 || periodT2 < 4600000)
	{
		SPI_PeripheralControl(SPI2,ENABLE);

		SPI_SendData(SPI2,(uint8_t*)task3,strlen(task3));

		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		SPI_PeripheralControl(SPI2,DISABLE);
	}
}

void t5(void)
{


	if(ConvertedValueP2 >= 2084 && periodT2 >= 5200000 )
		{

			SPI_PeripheralControl(SPI1,ENABLE);

			SPI_SendData(SPI1,(uint8_t*)task4,strlen(task4));

			while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

			SPI_PeripheralControl(SPI1,DISABLE);

		}
		else if(ConvertedValueP2 < 2084 || periodT2 < 5200000 )
		{
			SPI_PeripheralControl(SPI2,ENABLE);

			SPI_SendData(SPI2,(uint8_t*)task4,strlen(task4));

			while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

			SPI_PeripheralControl(SPI2,DISABLE);
		}
}

void t6(void)
{
	if(ConvertedValueP3 <= 3031 && periodT3 <= 5900000 )
	{

		SPI_PeripheralControl(SPI3,ENABLE);

		SPI_SendData(SPI3,(uint8_t*)task5,strlen(task5));

		while( SPI_GetFlagStatus(SPI3,SPI_BUSY_FLAG) );

		SPI_PeripheralControl(SPI3,DISABLE);
	}
	else if(ConvertedValueP3 > 3031 || periodT3 > 5900000 )
	{
		SPI_PeripheralControl(SPI3,ENABLE);

		SPI_SendData(SPI2,(uint8_t*)task5,strlen(task5));

		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		SPI_PeripheralControl(SPI2,DISABLE);
	}


}
