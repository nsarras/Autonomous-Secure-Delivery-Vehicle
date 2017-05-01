/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */


#include "tasks.hpp"
#include "examples/examples.hpp"
#include <stdio.h>
#include "sys_config.h"
#include "io.hpp"
#include "storage.hpp"
#include "event_groups.h"
#include "LPC17xx.h"
#include "gpio.hpp"
#include "utilities.h"
#include "printf_lib.h"
#include <string.h> // include the use of strings
#include "i2c2.hpp"
#include "i2c_base.hpp"
#include "_ansi.h"
#include <sys/reent.h>
#include <machine/time.h>// from time file
#include <stdint.h>
#include <stdio.h>

// Library for IR sensors
#include "adc0.h"

// Library for PWMS
#include "lpc_pwm.hpp"


//-------------------------------------------------------------Pixy Camera Code-------------------------------

QueueHandle_t qh = 0;
class object_identifier_task:public scheduler_task
{
public:
	object_identifier_task(uint8_t priority):scheduler_task("object_identify", 2000, priority)
{

}
	bool run(void *p){

		uint16_t full_bytes = 0;

		if (xQueueReceive(qh, &full_bytes, portMAX_DELAY))
		{
			if(full_bytes == 517){

				printf("OBJECT RECOGNIZED... BLUE BOX\n");
			}
			else if(full_bytes == 514){
				printf("OBJECT RECOGNIZED... ORANGE\n");
			}
			else if(full_bytes == 259){
				printf("OBJECT RECOGNIZED... GREEN SCISSORS\n");
			}
			else{
				printf("OBJECT RECOGNIZED... LOOKING FOR OBJECT\n");
			}
		}


		return true;
	}
};


class pixy_UART_task:public scheduler_task
{
public:
	pixy_UART_task(uint8_t priority):scheduler_task("UART", 2000, priority)
{

}
	bool run(void *p)
	{

		int counter = 0;
		while (counter < 15) {
			uint16_t full_bytes[10];
			uint16_t check = Get_Upper_Lower_Bytes();

			if (check == 0xaa55) {
				full_bytes[0] = check;
				for (int i = 1; i < 10; i++) {
					full_bytes[i] = Get_Upper_Lower_Bytes();
				}
				vTaskDelay(1000);

				//xQueueSend(qh, &full_bytes[2], portMAX_DELAY);

				if(full_bytes[2] == 517){
					//xQueueSend(qh, &full_bytes, portMAX_DELAY);
					printf("OBJECT RECOGNIZED... BLUE BOX\n");
				}
				else if(full_bytes[2] == 514){
					printf("OBJECT RECOGNIZED... ORANGE\n");
				}
				else if(full_bytes[2] == 259){
					printf("OBJECT RECOGNIZED... GREEN SCISSORS\n");
				}
				else{
					printf("OBJECT RECOGNIZED... LOOKING FOR OBJECT\n");
				}

				printf("Here is the object information \n");
				printf("Reads full bytes and sync matched!!\n");

				for (int k = 0; k < 10; k++) {
					printf("Full Byte %i = %u \n", k, (unsigned int)full_bytes[k]);

				}
				vTaskDelay(1000);
			}
			counter++;
			printf("Counter = %i", counter);
		}
		return true;
	}

	bool init(void)
	{
		LPC_SC->PCONP |= (1 << 24); // powers on UART pin 2
		LPC_SC->PCLKSEL1 &= (3 << 16); // clear the bit in bits 16 and 17
		LPC_SC->PCLKSEL1 |= (1 << 16); // set the speed of the peripheral clock to clk, calculations become easier

		LPC_PINCON->PINSEL4 &= ~(0xF << 16); // Clear values
		LPC_PINCON->PINSEL4 |= (10 << 16);  // Set values for UART2 Rx/Tx

		uint32_t baud = 19200;
		uint16_t dll = sys_get_cpu_clock() / (16 * baud);
		// uint8_t dll= 48 * 1000 * 1000 /(16 *baud);

		LPC_UART2->LCR = (1 << 7); // set the DLAB register to 1

		LPC_UART2->DLM = (dll >> 8);
		LPC_UART2->DLL = (dll >> 0);

		// LPC_UART2->LCR &= ~(1 << 7);
		LPC_UART2->LCR = 3; // selects word length to 8 bits no stop bits or parity bits
		//Stop bit
		LPC_UART2->LCR &= ~(1<<2);
		return true;
	}

	void uart2_putChar(char out) {
		LPC_UART2->THR = out; // starts sending out the data

		// it takes time for the data to be sent across so we will have to wait for this function to complete
		// we can wait till the transmitter register bit is 1 to tell us if there is any data left
		while (1) {
			if (LPC_UART2->LSR & (1 << 6)) {
				break; // if bit 5 is 1 Transmitting register is empty
			}
		}
	}

	uint16_t Get_Upper_Lower_Bytes() {

		uint16_t upperByte, lowerByte;

		upperByte = uart2_GetUint16_t();
		lowerByte = uart2_GetUint16_t();

		uint16_t FullByte = (upperByte << 8) | lowerByte;

		printf("Full Byte = %u\n", (unsigned int)FullByte);
		return FullByte;
	}

	char uart2_getChar(void) {
		while (1) {
			if (LPC_UART2->LSR & (1 << 0)) {
				break; // if bit 0 is a 1 FIFO is not empty
			}
		}

		char out = LPC_UART2->RBR;
		return out;
	}

	uint16_t uart2_GetUint16_t(void) {
		while (1) {
			if (LPC_UART2->LSR & (1 << 0)) {
				break; // if bit 0 is a 1 FIFO is not empty
			}
		}

		uint16_t out = LPC_UART2->RBR;
		return out;
	}
};

//-------------------------------------------------------------IR Sensor Code-------------------------------


class IR_sensor:public scheduler_task
{
public:
	IR_sensor(uint8_t priority):scheduler_task("IR", 2000, priority)
{

}
	bool run(void *p)
	{
		int adc3 = adc0_get_reading(3); // Read the value of ADC-3
		printf("The adc integer is: %i \n", adc3);
		//if(1500 < adc3 < 1900) printf("There is an object in its way! \n");
		vTaskDelay(3000);
		// int adc4 = adc0_get_reading(4); // Read the value of ADC-4
		// int adc5 = adc0_get_reading(5); // Read the value of ADC-5
		return true;
	}
	bool init(void)
	{
		/*********************************************************
		 * ADC is already initialized before main() is called.
		 * There are 3 ADC pins labeled on the SJ-One board.
		 * You can use one or more channels as illustrated below.
		 */
		printf("Init IR Sensors");
		LPC_PINCON->PINSEL1 |= (1 << 20); // ADC-3 is on P0.26, select this as ADC0.3
		// LPC_PINCON->PINSEL3 |= (3 << 28); // ADC-4 is on P1.30, select this as ADC0.4
		// LPC_PINCON->PINSEL3 |= (3 << 30); // ADC-5 is on P1.31, select this as ADC0.5
		return true;
	}
};

//-------------------------------------------------------------PWM Motor Code-------------------------------


/**
 * You can control up to 6 servos with hardware signals (and more with sw)
 * Each signal is mapped to from P2.0 to P2.5
 */
void motor_control()
{
	/* Use 1Khz PWM.  Each PWM shares the 1st frequency you set */
	PWM motor1(PWM::pwm1, 1000);
	PWM motor2(PWM::pwm2, 0);

	/* Set to 50% motor speed */
	motor1.set(50);
	motor2.set(50);
}
void servo_control()
{
	/* Use 50Hz PWM for servos.  Each PWM will be 50Hz */
	PWM servo1(PWM::pwm1, 50);
	PWM servo2(PWM::pwm2, 0);

	servo1.set(5.0);  ///< Set to left position
	servo2.set(10.0); ///< Set to right position
}

//class PWM_Motor:public scheduler_task
//{
//public:
//	PWM_Motor(uint8_t priority):scheduler_task("PWM_Motor", 2000, priority)
//{
//
//}
//	bool run(void *p)
//	{
//		/* Use 1Khz PWM.  Each PWM shares the 1st frequency you set */
//		PWM motor1(PWM::pwm1, 1000);
//		PWM motor2(PWM::pwm2, 0);
//		/* Set to 50% motor speed */
//		printf("Setting motor speed! \n");
//		motor1.set(50);
//		motor2.set(50);
//		vTaskDelay(3000);
//		motor1.set(0);
//		motor2.set(0);
//		return true;
//	}
//	bool init(void)
//	{
//
//		return true;
//	}
//};

// PWM Turning functions, only including back motors for now

PWM motor1(PWM::pwm2, 1000);
PWM motor2(PWM::pwm2, 1000);

void motor_straight(void)
{
	motor1.set(65);
	motor2.set(65);
	// Left Motor
	LPC_GPIO0->FIOSET = (1 << 0);
	LPC_GPIO0->FIOCLR = (1 << 1);
	// Right Motor
	LPC_GPIO0->FIOSET = (1 << 30);
	LPC_GPIO0->FIOCLR = (1 << 29);
	delay_ms(1000);
	// Set to basic straight or wait here
}
void motor_slight_left(void)
{
	motor1.set(40);
	motor2.set(80);
	// Left Motor
	LPC_GPIO0->FIOSET = (1 << 1);
	LPC_GPIO0->FIOCLR = (1 << 0);
	// Right Motor
	LPC_GPIO0->FIOSET = (1 << 30);
	LPC_GPIO0->FIOCLR = (1 << 29);
	delay_ms(1000);
	// Set to basic straight or wait here
}
void motor_hard_left(void)
{
	motor1.set(10);
	motor2.set(100);
	// Left Motor
	LPC_GPIO0->FIOSET = (1 << 1);
	LPC_GPIO0->FIOCLR = (1 << 0);
	// Right Motor
	LPC_GPIO0->FIOSET = (1 << 30);
	LPC_GPIO0->FIOCLR = (1 << 29);
	delay_ms(1000);
	// Set to basic straight or wait here
}
void motor_slight_right(void)
{
	motor1.set(80);
	motor2.set(40);
	// Left Motor
	LPC_GPIO0->FIOSET = (1 << 1);
	LPC_GPIO0->FIOCLR = (1 << 0);
	// Right Motor
	LPC_GPIO0->FIOSET = (1 << 30);
	LPC_GPIO0->FIOCLR = (1 << 29);
	delay_ms(1000);
	// Set to basic straight or wait here
}
void motor_hard_right(void)
{
	motor1.set(100);
	motor2.set(10);
	// Left Motor
	LPC_GPIO0->FIOSET = (1 << 1);
	LPC_GPIO0->FIOCLR = (1 << 0);
	// Right Motor
	LPC_GPIO0->FIOSET = (1 << 30);
	LPC_GPIO0->FIOCLR = (1 << 29);
	delay_ms(1000);
	// Set to basic straight or wait here
}

//void pwm_testcode(){
//
//
//	printf("Init motor and GPIO\n");
//	int n = 0;
//	PWM motor1(PWM::pwm1, 200);
//	PWM motor2(PWM::pwm2, 1000);
//	motor1.set(65);
//	motor2.set(65);
//	LPC_GPIO0->FIODIR |= (1 << 0);
//	LPC_GPIO0->FIODIR |= (1 << 1);
//	LPC_GPIO0->FIODIR |= (1 << 29);
//	LPC_GPIO0->FIODIR |= (1 << 30);
//	while(1)
//	{
//		n = n + 5;
//		// First Motor Direction Set
//		// Left Motor
//		LPC_GPIO0->FIOSET = (1 << 1);
//		LPC_GPIO0->FIOCLR = (1 << 0);
//		// Right Motor
//		LPC_GPIO0->FIOSET = (1 << 30);
//		LPC_GPIO0->FIOCLR = (1 << 29);
//
//		delay_ms(10000);
//
//		// Second Motor Direction Set
//		// Left Motor
//		LPC_GPIO0->FIOSET = (1 << 0);
//		LPC_GPIO0->FIOCLR = (1 << 1);
//		// Right Motor SEt
//		LPC_GPIO0->FIOSET = (1 << 29);
//		LPC_GPIO0->FIOCLR = (1 << 30);
//		delay_ms(10000);
//
//		//motor1.set(10);
//		//motor2.set(n);
//		//motor1.set(n);
//		//printf("Current Motor Speed %i\n\n", n);
//
//	}

//
//}


// MAIN PWM TEST CODE

//	LPC_GPIO0->FIODIR |= (1 << 0);
//	LPC_GPIO0->FIODIR |= (1 << 1);
//	LPC_GPIO0->FIODIR |= (1 << 29);
//	LPC_GPIO0->FIODIR |= (1 << 30);
//	motor1.set(65);
//	motor2.set(65);
//	LPC_GPIO0->FIODIR |= (1 << 0);
//	LPC_GPIO0->FIODIR |= (1 << 1);
//	LPC_GPIO0->FIODIR |= (1 << 29);
//	LPC_GPIO0->FIODIR |= (1 << 30);
//	delay_ms(3000);

//	motor_straight();
//	delay_ms(3000);

	//motor_hard_left();
	//	delay_ms(3000);
	//
	//	motor_hard_right();
	//	delay_ms(3000);
	//
	//	motor_slight_left();
	//	delay_ms(3000);
	//
	//	motor_slight_right();
	//	delay_ms(3000);








int main(void)
{

	//scheduler_add_task(new PWM_Motor(PRIORITY_HIGH)); // PWM_Motors
	//scheduler_add_task(new IR_sensor(PRIORITY_HIGH)); // IR Sensors
	printf("Start tasks \n");
	scheduler_add_task(new pixy_UART_task(PRIORITY_HIGH));




	// ------- Critical Section of Code Below, do not change ----------------------------------------
	scheduler_add_task(new terminalTask(PRIORITY_HIGH));

	/* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
	scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

	/* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
#if 0
	scheduler_add_task(new periodicSchedulerTask());
#endif

	/* The task for the IR receiver */
	// scheduler_add_task(new remoteTask  (PRIORITY_LOW));

	/* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
	 * task to always be responsive so you can poke around in case something goes wrong.
	 */

	/**
	 * This is a the board demonstration task that can be used to test the board.
	 * This also shows you how to send a wireless packets to other boards.
	 */
#if 0
	scheduler_add_task(new example_io_demo());
#endif

	/**
	 * Change "#if 0" to "#if 1" to enable examples.
	 * Try these examples one at a time.
	 */
#if 0
	scheduler_add_task(new example_task());
	scheduler_add_task(new example_alarm());
	scheduler_add_task(new example_logger_qset());
	scheduler_add_task(new example_nv_vars());
#endif

	/**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
#if 0
	scheduler_add_task(new queue_tx());
	scheduler_add_task(new queue_rx());
#endif

	/**
	 * Another example of shared handles and producer/consumer using a queue.
	 * In this example, producer will produce as fast as the consumer can consume.
	 */
#if 0
	scheduler_add_task(new producer());
	scheduler_add_task(new consumer());
#endif

	/**
	 * If you have RN-XV on your board, you can connect to Wifi using this task.
	 * This does two things for us:
	 *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
	 *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
	 *
	 * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
	 * @code
	 *     // Assuming Wifly is on Uart3
	 *     addCommandChannel(Uart3::getInstance(), false);
	 * @endcode
	 */
#if 0
	Uart3 &u3 = Uart3::getInstance();
	u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
	scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
#endif

	scheduler_start(); ///< This shouldn't return
	return -1;
}
