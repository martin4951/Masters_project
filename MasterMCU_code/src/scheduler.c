/*
 * scheduler.c
 *
 *  Created on: 9 Aug 2020
 *      Author: Martino
 */


#include "scheduler.h"


//Task queue
Task tList[8];
unsigned int tCount = 0;


void scheduler_delete(int index);

//Task creation with callback function
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
       //Increment timer each time function is called
       tList[i].eTime++;
       //If task is due to be executed based on the period, then execute and reset the timer for that task
       if(tList[i].eTime >= tList[i].period)
       {
          tList[i].eTime = 0;
          tList[i].tFunction();

       }
       //Remove task in not periodic
       if(tList[i].repeat == 0)
       {
          scheduler_delete(i);
       }
    }
}


//Delete task from the task queue
void scheduler_delete(int index)
{
   int i;
   for(i = index; i < tCount - 1; i++)
   {
       tList[i] = tList[i + 1];
   }

   tCount--;
}
