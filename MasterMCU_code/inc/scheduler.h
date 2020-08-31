/*
 * scheduler.h
 *
 *  Created on: 9 Aug 2020
 *      Author: Martino
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_


typedef struct Task {
    void (*tFunction)(void);
    unsigned long period;
    unsigned char repeat;
    unsigned long eTime;
} Task;


void scheduler_create(void (*func)(void), int period unsigned char repeat);
void scheduler_execute(void);





#endif /* SCHEDULER_H_ */
