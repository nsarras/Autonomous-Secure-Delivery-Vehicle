
#include "tasks.hpp"
#include "sys_config.h"
#include "io.hpp"

#include "storage.hpp"
#include "event_groups.h"

#include "examples/examples.hpp"
#include "stdio.h"
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

#include "adc0.h" // analog to digital
/* Read this include file for more info */
#include "lpc_pwm.hpp" // PWM code


/// IDs used for getSharedObject() and addSharedObject()
typedef enum {
	shared_SensorQueueId,
} sharedHandleId_t;

uint16_t CheckSum, Signature, XCenter, YCenter, Width, Height;


#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORD_CC          0xaa56
#define PIXY_START_WORDX            0x55aa

PWM motor1(PWM::pwm3, 1000); // GPIO 0 and 1
PWM motor2(PWM::pwm4, 500); // GPIO 29 and 30

class pixy_UART_task:public scheduler_task
{
public:
	pixy_UART_task(uint8_t priority):scheduler_task("UART", 1000, priority)
{

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
		LPC_UART2->LCR = (1 << 7); // set the DLAB register to 1
		LPC_UART2->DLM = (dll >> 8);
		LPC_UART2->DLL = (dll >> 0);
		LPC_UART2->LCR = 3; // selects word length to 8 bits no stop bits or parity bits
		//Stop bit
		LPC_UART2->LCR &= ~(1 << 2);

		LPC_GPIO0->FIODIR |= (1 << 0); // Moving Motors
		LPC_GPIO0->FIODIR |= (1 << 1);
		LPC_GPIO0->FIODIR |= (1 << 29);
		LPC_GPIO0->FIODIR |= (1 << 30);
		return true;
		}
	bool run(void *p)
	{
		while (1) {
			uint16_t full_bytes[10];
			full_bytes[4] = -1;
			uint16_t check = Get_Upper_Lower_Bytes();

			printf("Checking \n");

			if (check == 0xaa55) {
				full_bytes[0] = check;
				for (int i = 1; i < 10; i++) {
					full_bytes[i] = Get_Upper_Lower_Bytes();
				}


				printf("Here is the object information \n");
				printf("Reads full bytes and sync matched!!\n");

				for (int k = 0; k < 8; k++) {
					printf("Full Byte %i = %u \n", k, (unsigned int)full_bytes[k]);
				}
				// vTaskDelay(3000);
				if(full_bytes[4] >= 0 && full_bytes[4] <= 200){
					//if(full_bytes[4] >= 0 && full_bytes[4] <= 30) motor_hard_right();
					if(full_bytes[4] >= 0 && full_bytes[4] <= 40) motor_slight_right();
					else if(full_bytes[4] >= 40 && full_bytes[4] <= 160) motor_straight();
					else if(full_bytes[4] >= 160 && full_bytes[4] <= 200) motor_slight_left();
					//else if(full_bytes[4] >= 170 && full_bytes[4] <= 200) motor_hard_left();
				}
			}
			else if(full_bytes[4] == -1)
			{
			motor_search();
			}

			// printf("Counter = %i \n", counter);
		}
		return true;

	}
	void motor_stop(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor stop \n");
	}
	void motor_straight(void)
	{
		printf("motor straight \n");
		motor1.set(20);
		motor2.set(20);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741825; // Go straight PWMS (0 and 30)
		delay_ms(1);
		//motor_stop();
	}
	void motor_search(void)
	{
		printf("motor search \n");
		motor1.set(20);
		motor2.set(20);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741826; // Go straight PWMS (0 and 30)
		delay_ms(150);
		motor_stop();
		delay_ms(150);
	}
	void motor_slight_left(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor slight left \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741826; // Go slight_right PWMS (1 and 30)
		delay_ms(200);
		motor_stop();
	}
	void motor_hard_left(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor hard left \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741826; // Go slight_right PWMS (1 and 30)
		delay_ms(500);
		motor_stop();
	}
	void motor_slight_right(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor slight right \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 536870913; // Go hard_left PWMS (0 and 29)
		delay_ms(200);
		motor_stop();
	}
	void motor_hard_right(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor hard right \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 536870913; // Go hard_left PWMS (0 and 29)
		delay_ms(500);
		motor_stop();
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

		// printf("Full Byte = %u\n", (unsigned int)FullByte); // debugging messages

		return FullByte;
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
	char uart2_getChar(void) {
		while (1) {
			if (LPC_UART2->LSR & (1 << 0)) {
				break; // if bit 0 is a 1 FIFO is not empty
			}
		}

		char out = LPC_UART2->RBR;
		return out;
	}
	// PWM Turning functions, only including back motors for now
};



void adc_read(void)
{
    /*********************************************************
     * ADC is already initialized before main() is called.
     * There are 3 ADC pins labeled on the SJ-One board.
     * You can use one or more channels as illustrated below.
     */

    LPC_PINCON->PINSEL1 |= (1 << 20); // ADC-3 is on P0.26, select this as ADC0.3
    LPC_PINCON->PINSEL3 |= (3 << 28); // ADC-4 is on P1.30, select this as ADC0.4
    // LPC_PINCON->PINSEL3 |= (3 << 30); // ADC-5 is on P1.31, select this as ADC0.5

    int adc3 = adc0_get_reading(3); // Read the value of ADC-3
    printf("The adc integer is %i: ", adc3);
    int adc4 = adc0_get_reading(4); // Read the value of ADC-4
    printf("The adc integer is %i: ", adc4);
    // int adc5 = adc0_get_reading(5); // Read the value of ADC-5
}




class IR_sensor:public scheduler_task
{
public:
	IR_sensor(uint8_t priority):scheduler_task("IR", 1000, priority)
{

}
	bool init(void)
	{
		/*********************************************************
		 * ADC is already initialized before main() is called.
		 * There are 3 ADC pins labeled on the SJ-One board.
		 * You can use one or more channels as illustrated below.
		 */
		printf("Init IR Sensors");
		LPC_PINCON->PINSEL1 |= (1 << 20); // ADC-3 is on P0.26, select this as ADC0.3 // Left IR Sensor
		LPC_PINCON->PINSEL3 |= (3 << 28); // ADC-4 is on P1.30, select this as ADC0.4 // Right IR Sensor
		// LPC_PINCON->PINSEL3 |= (3 << 30); // ADC-5 is on P1.31, select this as ADC0.5


		LPC_GPIO0->FIODIR |= (1 << 0); // Moving Motors
		LPC_GPIO0->FIODIR |= (1 << 1);
		LPC_GPIO0->FIODIR |= (1 << 29);
		LPC_GPIO0->FIODIR |= (1 << 30);
		return true;
	}

	bool run(void *p)
	{
		int adc3 = 0; // right IR sensor
		int adc4 = 0; // left IR sensor
		int average = 10;
		for(int i  = 0; i < average; i++)
			{
				adc3 += adc0_get_reading(3); // Read the value of ADC-3
				adc4 += adc0_get_reading(4); // Read the value of ADC-4
			}
		adc3 = adc3 / average;
		adc4 = adc4 / average;

		if(adc3 > 2000 || adc4 > 2000) motor_backwards();
		else if(adc3 > adc4 && adc3 > 800 && adc3 < 2000) motor_hard_right();
		else if(adc4 > adc3 && adc4 > 800 && adc4 < 2000) motor_hard_left();
		else if(adc3 > adc4 && adc3 > 500 && adc3 < 800) motor_slight_right();
		else if(adc4 > adc3 && adc4 > 500 && adc4 < 800) motor_slight_left();
		else if(adc3 < 500 && adc4 < 500) motor_search();
		else motor_search();

		printf("The adc 1 integer is: %i \n", adc3);
		printf("The adc 2 integer is: %i \n", adc4);
		printf("---------------------------\n");

		vTaskDelay(1000);
		return true;
	}

	void motor_stop(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor stop \n");
	}
	void motor_straight(void)
	{
		printf("motor straight \n");
		motor1.set(20);
		motor2.set(20);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741825; // Go straight PWMS (0 and 30)
		delay_ms(1);
		//motor_stop();
	}
	void motor_search(void)
	{
		printf("motor search \n");
		motor1.set(20);
		motor2.set(20);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741826; // Go straight PWMS (0 and 30)
		delay_ms(250);
		motor_stop();

	}
	void motor_backwards(void)
	{
		printf("motor backwards \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 536870914; // Go straight PWMS (1 and 29)
		delay_ms(500);
		motor_stop();
	}
	void motor_slight_left(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor slight left \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741826; // Go slight_right PWMS (1 and 30)
		delay_ms(300);
		motor_stop();
	}
	void motor_hard_left(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor hard left \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 1073741826; // Go slight_right PWMS (1 and 30)
		delay_ms(500);
		motor_stop();
	}
	void motor_slight_right(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor slight right \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 536870913; // Go hard_left PWMS (0 and 29)
		delay_ms(300);
		motor_stop();
	}
	void motor_hard_right(void)
	{
		LPC_GPIO0->FIOPIN &= 0;
		printf("motor hard right \n");
		motor1.set(50);
		motor2.set(50);
		// Left Motor and Right Motor
		LPC_GPIO0->FIOSET = 536870913; // Go hard_left PWMS (0 and 29)
		delay_ms(500);
		motor_stop();
	}

};


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

void motor_stop(void)
{
	LPC_GPIO0->FIOPIN &= 0;
}
void motor_straight(void)
{
	motor1.set(100);
	motor2.set(100);
	// Left Motor and Right Motor
	LPC_GPIO0->FIOSET = 1073741825; // Go straight PWMS (0 and 30)
	delay_ms(700);
	motor_stop();
}
void motor_backwards(void)
{
	motor1.set(100);
	motor2.set(100);
	// Left Motor and Right Motor
	LPC_GPIO0->FIOSET = 536870914; // Go straight PWMS (1 and 29)
	delay_ms(700);
	motor_stop();
}

void motor_slight_left(void)
{
	motor1.set(50);
	motor2.set(50);
	// Left Motor and Right Motor
	LPC_GPIO0->FIOSET = 1073741826; // Go slight_right PWMS (1 and 30)
	delay_ms(150);
	motor_stop();
}
void motor_hard_left(void)
{
	motor1.set(50);
	motor2.set(50);
	// Left Motor and Right Motor
	LPC_GPIO0->FIOSET = 1073741826; // Go slight_right PWMS (1 and 30)
	delay_ms(400);
	motor_stop();
}
void motor_slight_right(void)
{
	motor1.set(50);
	motor2.set(50);
	// Left Motor and Right Motor
	LPC_GPIO0->FIOSET = 536870913; // Go hard_left PWMS (0 and 29)
	delay_ms(150);
	motor_stop();
}
void motor_hard_right(void)
{
	motor1.set(50);
	motor2.set(50);
	// Left Motor and Right Motor
	LPC_GPIO0->FIOSET = 536870913; // Go hard_left PWMS (0 and 29)
	delay_ms(400);
	motor_stop();
}
int main(void)
{
	scheduler_add_task(new IR_sensor(PRIORITY_HIGH)); // IR Sensors
	scheduler_add_task(new pixy_UART_task(PRIORITY_HIGH)); // Pixy Communicates to SJONE board through UART2 pins


	scheduler_add_task(new terminalTask(PRIORITY_HIGH));

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
