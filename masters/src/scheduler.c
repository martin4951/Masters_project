/*
 * scheduler.c
 *
 *  Created on: 9 Aug 2020
 *      Author: Martino
 */


#include "scheduler.h"



Task tList[8];
unsigned int tCount = 0;


void scheduler_delete(int index);


void scheduler_create(void (*func)(void),int period, unsigned char repeat)
{
    Task task;
    task.tFunction = func;
    task.period = period;
    task.repeat = repeat;
    task.eTime = 0;

    tList[tCount++] = task;
}


void scheduler_execute(void)
{
    unsigned int i;
    for(i = 0; i < tCount; i++)
    {

       tList[i].tFunction();


       if(tList[i].repeat == 0)
       {
          scheduler_delete(i);
       }
    }
}



void scheduler_delete(int index)
{
   int i;
   for(i = index; i < tCount - 1; i++)
   {
       tList[i] = tList[i + 1];
   }

   tCount--;
}
