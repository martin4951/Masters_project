#include <Snooze.h>
#include <SnoozeBlock.h>

#include <assert.h>

#include <Audio.h>
#include <avr/power.h>

#include "TSPISlave.h"
#include <SPI.h>

#include <vector>

//SnoozeTimer timer;

AudioInputAnalog         adc1(A2);
AudioAnalyzeFFT256       fft1;
AudioConnection          patchCord2(adc1, fft1);

SnoozeDigital digital;

// install drivers to a SnoozeBlock
SnoozeBlock config(digital);

uint8_t miso = 12;
uint8_t mosi = 11;
uint8_t sck = 13;
uint8_t cs = 10;
uint8_t spimode = 8;

uint16_t buffer;

int array[7];
int interrupt = 0;
int count = 0;
int sleepFlag = 0;

uint16_t dataLen = 0;
uint32_t dataSize = 0;
uint8_t i = 0;
int size = 22;


TSPISlave mySPI = TSPISlave(SPI, miso, mosi, sck, cs, spimode);

void setup() {
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  AudioMemory(16);
  Serial.begin(9600);
  fft1.averageTogether(8);
  digital.pinMode(10, INPUT_PULLUP, RISING);//pin, mode, type
  Serial.begin(9600);
  mySPI.onReceive(myFunc); // callback for mySPI
 
}



void loop() {
  //sleepFlag = 0;
  Serial.println(millis());
  delay(1000);
  for(int k=0; k<6; k++)
  {
  Serial.println("Array value:");
  Serial.println(array[k]);
  }

//  Serial.println("buffer");
//  Serial.println(buffer);


   
//  Serial.println("array[0]");
//  Serial.println(array[0]);
//  Serial.println("array[1]");
//  Serial.println(array[1]);
//  Serial.println("array[2]");
//  Serial.println(array[2]);
//  Serial.println("array[3]");
//  Serial.println(array[3]);
//  Serial.println("array[4]");
//  Serial.println(array[4]);
//  Serial.println("array[5]");
//  Serial.println(array[5]);




 
  if(array[6] == 0)
  {
  
  if(buffer == "Task0")
  {
    Serial.println("Task0");
    array[count++] = buffer;

    buffer=0;
  }
  else if(buffer == "Task1")
  {
    Serial.println("Task1");
    array[count++] = buffer;

    buffer=0;
  }
  else if(buffer == "Task2")
  {
    Serial.println("Task2");
    array[count++] = buffer;

    buffer=0;
  }
  else if(buffer == "Task3")
  {
    Serial.println("Task3");
    array[count++] = buffer;

    buffer=0;
  }
  else if(buffer == "Task4")
  {
    Serial.println("Task4");
    array[count++] = buffer;

    buffer=0;
  }
  else if(buffer == "Task5")
  {
    Serial.println("Task5");
    array[count++] = buffer;

    buffer=0;
  }
  }

  if(array[0] != 0)
  {
  if(array[0] == "Task0")
  {
      
      
      for (uint16_t i = 0; i < 399; i += 1) 
      {
        
        sqrt(i)

      }
      
    Serial.println("Task [0] complete");
    array[0] = 0;
  }

  if(array[0] == "Task1")
  {
      
      for (uint16_t i = 0; i < 399; i += 1) 
      {
        sin(i)
      }
      
    Serial.println("Task complete");
    array[0] = 0;
  }

  if(array[0] == "Task2")
  {
      
      for (uint16_t i = 0; i < 399; i += 1) 
      {
        tan(i)
      }
      
    Serial.println("Task [2] complete");
    array[0] = 0;
  }

  if(array[0] == "Task3")
  {
      
      for (uint16_t i = 0; i < 399; i += 1) 
      {
        log(i)
      }
      
    Serial.println("Task [3] complete");
    array[0] = 0;
  }

  if(array[0] == "Task4")
  {
      
      for (uint16_t i = 0; i < 200; i += 1) 
      {
        int32_t n = fft1.read(i);
      }
      
    Serial.println("Task [4] complete");
    array[0] = 0;
  }

  if(array[0] == "Task5")
  {
      
      for (uint16_t i = 0; i < 50; i += 1) 
      {
        fib(i)
      }
      
    Serial.println("Task [5] complete");
    array[0] = 0;
  }
  
  }

  if(array[0] == 0)
  {
    sleepFlag == 1
    
    
  }

  if(sleepFlag == 1)
  {
    asm("wfi")
  }
 

}

void myFunc() {
 
  //Serial.println("START: ");
   
  //buffer[i] = mySPI.popr();
  while ( mySPI.active() ) {
    if (mySPI.available()) {

    //sleepFlag = 0;
    buffer = mySPI.popr();
    
    
   
 
   
   
   
    //mySPI.pushr(size);
 
 
   
    }
    }
    }

int fib(int n) {
    assert(n >= 0);
    if (n > 1) 
        return fib(n - 1) + fib(n - 2);
    else if (n == 1)
        return 1;
    else 
        return 0;
}


    

    
 
