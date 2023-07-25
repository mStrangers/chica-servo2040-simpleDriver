/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "servo2040.hpp"

using namespace servo;

/* Input Defines */
#define SPECIAL_BYTE1	0x1b	
#define SPECIAL_BYTE2	0x5b
#define KEY_UP			0x41
#define KEY_RIGHT		0x43
#define KEY_DOWN		0x42
#define KEY_LEFT		0x44

/* PWM Defines */
#define MIN_PULSE_VALUE	500
#define MAX_PULSE_VALUE 2500



/* Create an array of servo pointers */
const int START_PIN = servo2040::SERVO_1; 	// Can be changed to only calibrate a 
const int END_PIN = servo2040::SERVO_18;	// a group of servos
const int NUM_SERVOS = (END_PIN - START_PIN) + 1;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);

/* PWM value storage */
int PWMvalues[NUM_SERVOS];
int ServoDefaultValue[NUM_SERVOS] = {1500,550,2450,1500,550,2450,1500,550,2450,1500,2450,550,1500,2450,550,1500,2450,550};
int ServoMinValue[NUM_SERVOS] = {500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500};
int ServoMaxValue[NUM_SERVOS] = {2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500};
int ServoDirection[NUM_SERVOS] = {1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
int ServoOffset[NUM_SERVOS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* Helper variables */
char inputByte1;
char inputByte2;
char inputByte3;
uint currPWM[NUM_SERVOS];
int currLeg = 1;
#define CURRLEGOFFSET (((currLeg-1)*3)-1)

int main() {
	stdio_init_all();
	
	/* Initialize the servo cluster */
	servos.init();
	
	while (!stdio_usb_connected()); // Wait for VCP/CDC connection;
	
	sleep_ms(2000);
	
	printf(" Press any key to continue...\r\n\r\n");
	getchar();

	printf(" Defaults all servos...\r\n\r\n");
	
	//set defaut pos
	for (auto currServo = START_PIN; currServo < NUM_SERVOS; currServo++) {
		servos.pulse(currServo , ServoDefaultValue[currServo]);
		printf("Default Servo %d..\r\n", currServo + 1 );
	}

	/* Enable all servos (centers all servos) */
	for (auto currServo = START_PIN; currServo < NUM_SERVOS; currServo++) {
		printf(" Enable servo %d...\r\n", currServo + 1);
		servos.enable(currServo);
		sleep_ms(250); 	// Give each servo time to center to avoid
	}					// drawing too much current at once
	printf("\r\n");
	
	printf(	" Each servo will now be calibrated. The controls are as follows:\r\n"
			" Up Arrow:\t Fast Clockwise\r\n"
			" Right Arrow:\t Slow Clockwise\r\n\r\n"
			" Down Arrow:\t Fast Counter-Clockwise\r\n"
			" Left Arrow:\t Slow Counter-Clockwise\r\n\r\n"
			" Type the space bar to save the PWM as the calibrated value and continue.\r\n\r\n");	
	

		
	while (inputByte1 != ' ')
	{
		inputByte1 = 0;
		inputByte2 = 0;
		inputByte3 = 0;

		//get actual servo value 
		printf(" Leg: %d\r\n", currLeg);
		currPWM[CURRLEGOFFSET+1] = servos.pulse(CURRLEGOFFSET+1); //coax
		printf(" Coax: %d\r\n", currPWM[CURRLEGOFFSET+1]);
		currPWM[CURRLEGOFFSET+2] = servos.pulse(CURRLEGOFFSET+2); //femur
		printf(" Femur: %d\r\n", currPWM[CURRLEGOFFSET+2]);
		currPWM[CURRLEGOFFSET+3] = servos.pulse(CURRLEGOFFSET+3); //tibia
		printf(" Tibia: %d\r\n", currPWM[CURRLEGOFFSET+3]);		
	
		/* special KEY_ handling 
		   3 bytes returned: 0x1b, 0x5b, 0x__ */
		inputByte1 = getchar();
		if (inputByte1 == SPECIAL_BYTE1) {
			inputByte2 = getchar(); // Throw away SPECIAL_BYTE2
			inputByte3 = getchar(); // Arrow ID
			switch (inputByte3)
			{
				case KEY_LEFT:
				{
					currLeg -= 1;
					if (currLeg < 1){
						currLeg = 1;
					}
					currPWM[CURRLEGOFFSET+1] = servos.pulse(CURRLEGOFFSET+1); //coax
					currPWM[CURRLEGOFFSET+2] = servos.pulse(CURRLEGOFFSET+2); //femur
					currPWM[CURRLEGOFFSET+3] = servos.pulse(CURRLEGOFFSET+3); //tibia
					break;
				}
				case KEY_RIGHT:
				{
					currLeg += 1;
					if (currLeg > 6){
						currLeg = 6;
					}
					currPWM[CURRLEGOFFSET+1] = servos.pulse(CURRLEGOFFSET+1); //coax
					currPWM[CURRLEGOFFSET+2] = servos.pulse(CURRLEGOFFSET+2); //femur
					currPWM[CURRLEGOFFSET+3] = servos.pulse(CURRLEGOFFSET+3); //tibia
					break;
				}
			default:
				printf("\r\n Oops! An invalid key was pressed, please try again.\r\n\r\n");
				break;
			} // switch (serialInput)
		}
		/* normal character handling */
		else if (inputByte1 != ' ') {
			switch (inputByte1)
			{
				case '1':
				{
					currPWM[CURRLEGOFFSET+1] -= 10;
					break;
				}
				case '4':
				{
					currPWM[CURRLEGOFFSET+1] += 10;
					break;
				}
				case '2':
				{
					currPWM[CURRLEGOFFSET+2] -= 10;
					break;
				}
				case '5':
				{
					currPWM[CURRLEGOFFSET+2] += 10;
					break;
				}
				case '3':
				{
					currPWM[CURRLEGOFFSET+3] -= 10;
					break;
				}
				case '6':
				{
					currPWM[CURRLEGOFFSET+3] += 10;
					break;
				}
				case 'r':
				{
					printf("reset servo position");
					for (auto currServo = START_PIN; currServo < NUM_SERVOS; currServo++) {
						servos.pulse(currServo , ServoDefaultValue[currServo]);
						printf("Default Servo %d..\r\n", currServo + 1 );
					}
					break;
				}
			default:
				printf("\r\n Oops! An invalid key was pressed, please try again.\r\n\r\n");
				break;
			}

		}
			
		for (auto currServo = 1; currServo < 4; currServo++) {
			/* Input Hygiene */
			if (currPWM[CURRLEGOFFSET+currServo] < MIN_PULSE_VALUE){
				currPWM[CURRLEGOFFSET+currServo] = MIN_PULSE_VALUE;
				printf(" D'oh! The minimum PWM in the script is %d\r\n", MIN_PULSE_VALUE);
			}
			else if (currPWM[CURRLEGOFFSET+currServo] > MAX_PULSE_VALUE){
				currPWM[CURRLEGOFFSET+currServo] = MAX_PULSE_VALUE;
				printf(" D'oh! The minimum PWM in the script is %d\r\n", MAX_PULSE_VALUE);
			}
			
			servos.pulse(CURRLEGOFFSET+currServo, currPWM[CURRLEGOFFSET+currServo]);
		}
	} // while (serialInput != ' ')
		
	/* Exiting keyboard input loop */
	inputByte1 = 0;
	inputByte2 = 0;
	inputByte3 = 0;
		

		
	/* Print results */
	printf(" **********************************************************************\r\n");
	printf(	" test is now complete \r\n");
	/* Disable Servos */
	servos.disable_all();
	
	printf("\r\n");
	printf("\r\n");
	printf("\r\n");
	exit(EXIT_SUCCESS);
}
	