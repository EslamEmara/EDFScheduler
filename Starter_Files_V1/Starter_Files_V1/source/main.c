/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

#define QUEUE_SIZE				3
#define QUEUE_ITEM_SIZE		1				/*IN BYTES*/

#define MESSAGE_NOT_SENT	0
#define MESSAGE_SENT			1

#define TASK1_PERIOD			50
#define TASK2_PERIOD			50
#define TASK3_PERIOD			100
#define TASK4_PERIOD			20
#define TASK5_PERIOD			10
#define TASK6_PERIOD			100

#define FALLING_BUTTON1_ID 	(uint8_t)0
#define RISING_BUTTON1_ID 	(uint8_t)1
#define FALLING_BUTTON2_ID 	(uint8_t)2
#define RISING_BUTTON2_ID 	(uint8_t)3
#define PERIODIC_STRING_ID	(uint8_t)4


signed char MessagesByID[5][20] = {{"=RISING BUTTON1="},{"=FALLING BUTTON1="},{"=RISING BUTTON2="},{"=FALLING BUTTON2="},{"PERIODIC"}};
/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

TaskHandle_t Task1Handler = NULL;
TaskHandle_t Task2Handler = NULL;
TaskHandle_t Task3Handler = NULL;
TaskHandle_t Task4Handler = NULL;
TaskHandle_t Task5Handler = NULL;
TaskHandle_t Task6Handler = NULL;

QueueHandle_t xQueue;

volatile int misses = 0;
int TaskIDLE_inTime,TaskIDLE_execTime;
float CPU_LOAD;

 void Button_1_Monitor( void * pvParameters )
 {				
			TickType_t startTime = 0;
			static uint8_t lastState = PIN_IS_LOW;
	 		static uint8_t messageState = MESSAGE_NOT_SENT;
			pinState_t buttonState;
			uint8_t xMessage;
			TickType_t xLastWakeTime;
			vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
     // Initialise the xLastWakeTime variable with the current time.
			xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {
					buttonState	=	GPIO_read(PORT_0,PIN0);

					if(buttonState == PIN_IS_LOW && lastState == PIN_IS_HIGH )
					{
						xMessage = RISING_BUTTON1_ID;
						lastState = PIN_IS_LOW; 
						messageState = MESSAGE_NOT_SENT;
					}
					else if(buttonState == PIN_IS_HIGH && lastState == PIN_IS_LOW )
					{
						xMessage = FALLING_BUTTON1_ID;
						lastState = PIN_IS_HIGH;
						messageState = MESSAGE_NOT_SENT;
					}
					
					if(messageState == MESSAGE_NOT_SENT)
					{
					 if( xQueueSendToBack( xQueue, &xMessage, 10 ) != pdPASS )
						{
						 /* Data could not be sent to the queue even after waiting 10 ticks. */
						}
						messageState = MESSAGE_SENT;
					}
					if(xTaskGetTickCount() - startTime > TASK1_PERIOD){
						misses++;
					}
         vTaskDelayUntil( &xLastWakeTime, TASK1_PERIOD );
				 startTime = xTaskGetTickCount();


     }
 }
   
  void Button_2_Monitor( void * pvParameters )
 {		
			TickType_t startTime = 0;
			static uint8_t lastState = PIN_IS_LOW;
	 		static uint8_t messageState = MESSAGE_NOT_SENT;
			pinState_t buttonState;
			uint8_t xMessage;
			TickType_t xLastWakeTime;
			
			vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
     // Initialise the xLastWakeTime variable with the current time.
			xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {
					buttonState	=	GPIO_read(PORT_0,PIN1);

					if(buttonState == PIN_IS_LOW && lastState == PIN_IS_HIGH )
					{
						xMessage = RISING_BUTTON2_ID;
						lastState = PIN_IS_LOW; 
						messageState = MESSAGE_NOT_SENT;
					}
					else if(buttonState == PIN_IS_HIGH && lastState == PIN_IS_LOW )
					{
						xMessage = FALLING_BUTTON2_ID;
						lastState = PIN_IS_HIGH;
						messageState = MESSAGE_NOT_SENT;
					}
					
					if(messageState == MESSAGE_NOT_SENT)
					{
					 if( xQueueSendToBack( xQueue, &xMessage, 10 ) != pdPASS )
						{
						 /* Data could not be sent to the queue even after waiting 10 ticks. */
						}
						messageState = MESSAGE_SENT;
					}
         // Wait for the next cycle.
					if(xTaskGetTickCount() - startTime > TASK2_PERIOD){
						misses++;
					}
         vTaskDelayUntil( &xLastWakeTime, TASK2_PERIOD );
				 startTime = xTaskGetTickCount();

     }
 }
   
  void Periodic_Transmitter( void * pvParameters )
 {			int startTime = 0;

		TickType_t xLastWakeTime;
		uint8_t xMessage = PERIODIC_STRING_ID;
		vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
     // Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {
			 	 if( xQueueSendToBack( xQueue, &xMessage, 10 ) != pdPASS )
						{
						 /* Data could not be sent to the queue even after waiting 10 ticks. */
						}
					if(xTaskGetTickCount() - startTime > TASK3_PERIOD){
						misses++;
					}
         vTaskDelayUntil( &xLastWakeTime, TASK3_PERIOD );
				 startTime = xTaskGetTickCount();


     }
 }
   
  void Uart_Receiver( void * pvParameters )
 {
			uint8_t xMessage = 0;
			TickType_t xLastWakeTime;
	 			int startTime = 0;

	  vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {
				 if( xQueueReceive( xQueue, &xMessage, portMAX_DELAY ) != pdPASS )
				 {
				 /* Nothing was received from the queue – even after blocking to wait
				 for data to arrive. */
				 }
				 else
				 {
					 startTime = xTaskGetTickCount();
					 xSerialPutChar('\n');
					 vSerialPutString(MessagesByID[xMessage],20);
						if(xTaskGetTickCount() - startTime > TASK4_PERIOD){
							misses++;
						}
				 /* xMessage now contains the received data. */
				 }

         // Wait for the next cycle.
         vTaskDelayUntil( &xLastWakeTime, TASK4_PERIOD );


     }
 }
void Load_1_Simulation( void * pvParameters )
 {
	 		long unsigned int i = 0;
			TickType_t xLastWakeTime;
			int startTime = xTaskGetTickCount();
	 
			vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();
     for( ;; )
     {		
					for(i = 0; i<34500;i++){							/*Exec. time = 5ms*/
						i=i;
					}

					if((xTaskGetTickCount() - startTime) > TASK5_PERIOD){
						misses++;
					}
					
         vTaskDelayUntil( &xLastWakeTime, TASK5_PERIOD );
				 startTime = xTaskGetTickCount();

     }
 }
 
void Load_2_Simulation( void * pvParameters )
 {
	 	 	long unsigned int i = 0;
			int startTime = 0;

			TickType_t xLastWakeTime;
			vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {		
			 		for(i = 0; i<80000;i++){						/*Exec. Time = 12ms*/
						i=i;
					}
					
         // Wait for the next cycle.
					if(xTaskGetTickCount() - startTime > TASK6_PERIOD){
						misses++;
					}
         vTaskDelayUntil( &xLastWakeTime, TASK6_PERIOD );
				 startTime = xTaskGetTickCount();

     }
 }


void vApplicationTickHook( void ){
		GPIO_write(PORT_0,PIN8,PIN_IS_HIGH);
		GPIO_write(PORT_0,PIN8,PIN_IS_LOW);
		//CPU_LOAD = ((float)(Task1_execTime + Task2_execTime + Task3_execTime +Task4_execTime+Task5_execTime+Task6_execTime) / (float)T1TC)*100.00;
		CPU_LOAD = 100 - (((float)TaskIDLE_execTime / T1TC)*100.00) ;
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{

	/* Setup the hardware for use with the Keil demo board. */
		xQueue = xQueueCreate(QUEUE_SIZE,QUEUE_ITEM_SIZE);
		
			prvSetupHardware();
    /* Create Tasks here */
		xTaskPeriodicCreate(Button_1_Monitor,			"Button_1_Monitor",		100,0,1,&Task1Handler,TASK1_PERIOD);
		xTaskPeriodicCreate(Button_2_Monitor,			"Button_2_Monitor",		100,0,1,&Task2Handler,TASK2_PERIOD);
		xTaskPeriodicCreate(Periodic_Transmitter,"Periodic_Transmitter",100,0,1,&Task3Handler,TASK3_PERIOD);
		xTaskPeriodicCreate(Uart_Receiver,				"Uart_Receiver",			100,0,1,&Task4Handler,TASK4_PERIOD);
		xTaskPeriodicCreate(Load_1_Simulation,		"Load_1_Simulation",	100,0,1,&Task5Handler,TASK5_PERIOD);
		xTaskPeriodicCreate(Load_2_Simulation,		"Load_2_Simulation",	100,0,1,&Task6Handler,TASK6_PERIOD);
/*
		xTaskCreate(Button_1_Monitor,			"Button_1_Monitor",		100,0,1,&Task1Handler);
		xTaskCreate(Button_2_Monitor,			"Button_2_Monitor",		100,0,1,&Task2Handler);
		xTaskCreate(Periodic_Transmitter,"Periodic_Transmitter",100,0,1,&Task3Handler);
		xTaskCreate(Uart_Receiver,				"Uart_Receiver",			100,0,1,&Task4Handler);
		xTaskCreate(Load_1_Simulation,		"Load_1_Simulation",	100,0,1,&Task5Handler);
		xTaskCreate(Load_2_Simulation,		"Load_2_Simulation",	100,0,1,&Task6Handler);
*/
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


