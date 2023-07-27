/**
 * Copyright (c) 2023 Eddie Carrera
 * MIT License
 */

#include "main.h"
using namespace plasma;
using namespace servo;

/////////////* Global Variables */////////////
/* Create an array of servo pointers */
const int START_PIN = servo2040::SERVO_1;
const int END_PIN = servo2040::SERVO_18;
const int NUM_SERVOS = (END_PIN - START_PIN) + 1;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);

/* Set up the shared analog inputs */
Analog sen_adc = Analog(servo2040::SHARED_ADC);
Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
						servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);

/* Set up the analog multiplexer, including the pin for controlling pull-up/pull-down */
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2,
						  PIN_UNUSED, servo2040::SHARED_ADC);

/* Create the LED bar, using PIO 1 and State Machine 0 */
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

/* Servo */
int ServoDirection[NUM_SERVOS] = {1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
int ServoOffset[NUM_SERVOS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int ServoDefaultsPosition[NUM_SERVOS] = {1500,550,2450,1500,550,2450,1500,550,2450,1500,2450,550,1500,2450,550,1500,2450,550};
servoMove ServosMove[NUM_SERVOS];

/*Serial buffer*/
cmdPkt curr_cmdPkt;

uint servoEnabled = false;

int main()
{
	/*******************************************************************************
	 * Initializations
	 ******************************************************************************/
	/* Initialize the servo cluster */
	servos.init();
	curr_cmdPkt.count = 0 ;

	/* Initialize analog inputs with pull downs */
	for (auto i = 0u; i < servo2040::NUM_SENSORS; i++)
	{
		mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
	}

	/* Initialize A0,A1,A2 */
	gpio_init_mask(A0_GPIO_MASK | A1_GPIO_MASK | A3_GPIO_MASK);
	gpio_set_dir_masked(A0_GPIO_MASK | A1_GPIO_MASK | A3_GPIO_MASK,
						GPIO_OUTPUT_MASK); // Set output
	gpio_put_masked(A0_GPIO_MASK | A1_GPIO_MASK | A3_GPIO_MASK,
					GPIO_LOW_MASK); // Set LOW

	stdio_init_all();
	/* Wait for VCP/CDC connection */
	led_bar.start();
	while (!stdio_usb_connected()){pendingVCP_ledSequence();}
	led_bar.clear();

	printf(" Defaults all servos...\r\n\r\n");
	
	//set defaut pos
	for (auto currServo = START_PIN; currServo < NUM_SERVOS; currServo++) {
		servos.pulse(currServo , ServoDefaultsPosition[currServo]);
		printf("Default Servo %d..\r\n", currServo + 1 );
	}

	/* Enable all servos (centers all servos) */
	for (auto currServo = START_PIN; currServo < NUM_SERVOS; currServo++) {
		printf(" Enable servo %d...\r\n", currServo + 1);
		servos.enable(currServo);
		sleep_ms(250); 	// Give each servo time to center to avoid
	}					// drawing too much current at once
	printf("\r\n");

	/*******************************************************************************
	 * Application
	 ******************************************************************************/
	while (1)
	{
		/* Monitor and parse serial data */
		parse_task();
		move_servo();

	} // while(1)
}

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
/*******************************************************************************
 * Core Functions
 ******************************************************************************/
void parse_task(void)
{
	
	int start_loop = millis();
	int input;

	input = getchar_timeout_us(GETC_TIMEOUT_US);
	// check character until one or timeout
	// if character check if we've already waiting for <cr>
	// command
	// check if start char '#'
	// 
	while (input != PICO_ERROR_TIMEOUT)
	{
		/***************************** START OF PARSING *************************************/
		
		//*Check if it is an end char*/
		if ( (input == '\r') && (curr_cmdPkt.count > 0) ) 
		{
			/* show values*/
			/*
			printf("Num char '%d'\n", curr_cmdPkt.count); //for debug
			for (uint i = 0; i <= curr_cmdPkt.count; i++)
			{
				printf("%c", curr_cmdPkt.valueBuffer[i]); //for debug
			}
			printf("\n"); //for debug
			*/
			/*process buffer*/
			if (curr_cmdPkt.valueBuffer[0] == '#') //check if it is a command
			{
				//cut buffer in multiple command
				parsedCmd pCmd[17];
				uint numberOfCmd = 0;
				uint numberOfChars = 0;
				for (uint i = 0; i < curr_cmdPkt.count; i++)
				{
					if ((curr_cmdPkt.valueBuffer[i] == '#') || (curr_cmdPkt.valueBuffer[i] == 'Q') || (curr_cmdPkt.valueBuffer[i] == 'A') || (curr_cmdPkt.valueBuffer[i] == 'B') || (curr_cmdPkt.valueBuffer[i] == 'C') || (curr_cmdPkt.valueBuffer[i] == 'D') || (curr_cmdPkt.valueBuffer[i] == 'E') || (curr_cmdPkt.valueBuffer[i] == 'F'))
					{
						numberOfCmd++;
						numberOfChars = 0;
						pCmd[numberOfCmd - 1].valueBuffer[numberOfChars] = curr_cmdPkt.valueBuffer[i];
						pCmd[numberOfCmd - 1].count = numberOfChars+1;

					}
					else
					{
						numberOfChars++;
						pCmd[numberOfCmd - 1].valueBuffer[numberOfChars] = curr_cmdPkt.valueBuffer[i];
						pCmd[numberOfCmd - 1].count = numberOfChars+1;
					}

				} // for (int i = 0; i < curr_cmdPkt.count; i++)
				
				//printf("Num cmd '%d'\n", numberOfCmd); //for debug

				// parse each command
				for (uint i = 0; i < numberOfCmd; i++) //each commands
				{
					/*
					for (uint j = 0; j < pCmd[i].count; j++)
					{
						printf("%c", pCmd[i].valueBuffer[j]); //for debug
					}
					printf("\n"); //for debug
					*/
					//parse the first character to know the command
					switch (pCmd[i].valueBuffer[0])
					{
					case '#':
						{
							//servomove
							//parsing the command
							uint actualChar = 1 ;
							uint ServoNumber = 0;
							uint ServoPosition = 0;
							uint ServoSpeed = 0;
							uint ServoTime = 0 ;
							while ( actualChar < pCmd[i].count ) //start at second character
							{
								//printf("first servo char : %c\n", pCmd[i].valueBuffer[actualChar]); //for debug
								ServoNumber = pCmd[i].valueBuffer[actualChar] - '0'; //convert first character to integer
								actualChar++; //second servo number or next command
								if ( (pCmd[i].valueBuffer[actualChar] != ' ') && (pCmd[i].valueBuffer[actualChar] != 'P') )
								{
									//add the second one and convert to integer
									//printf("second servo char : %c\n", pCmd[i].valueBuffer[actualChar]); //for debug
									ServoNumber = ServoNumber * 10 + (pCmd[i].valueBuffer[actualChar] - '0');
									actualChar++; // move to next character
									
								}
								//printf("Servo Number Int : %d\n", ServoNumber); // for debugging
								ServoNumber= ServoNumber - 1 ;
								// only 2 char servo max 
								if (pCmd[i].valueBuffer[actualChar] == ' ')
								{
									actualChar++; // skip space if one
								}
								//next will be PWM value
								if ((pCmd[i].valueBuffer[actualChar] == 'P') && (actualChar < pCmd[i].count))
								{
									actualChar++; // skip P
									while ( (pCmd[i].valueBuffer[actualChar] != ' ') && (pCmd[i].valueBuffer[actualChar] != 'S') && (pCmd[i].valueBuffer[actualChar] != 'T') && (actualChar < pCmd[i].count) ) 
									{
										//printf("servo PWM char : %c \n", pCmd[i].valueBuffer[actualChar]); //for debug
										ServoPosition = ServoPosition * 10 + ( pCmd[i].valueBuffer[actualChar] - '0' ); //convert character to integer
										//printf("temp PWM Int : %d \n", ServoPosition); // for debugging
										actualChar++; //second servo number or space
									}
									//printf("Servo PWM Int : %d\n", ServoPosition); // for debugging
									// constrain position
									if (ServoPosition > 2500) 
									{
										ServoPosition = 2500;
									} else if (ServoPosition < 500) 
									{
										ServoPosition = 500;
									}
								} else 
								{
									printf("Syntax error\n");
									break;
								}
								
								if (pCmd[i].valueBuffer[actualChar] == ' ')
								{
									actualChar++; // skip space if one
								}

								//Next will be Time Or Speed 
								if ((pCmd[i].valueBuffer[actualChar] == 'S') && (actualChar < pCmd[i].count))
								{
									actualChar++; // skip S
									while ( (pCmd[i].valueBuffer[actualChar] != ' ') && (pCmd[i].valueBuffer[actualChar] != 'T') && (actualChar < pCmd[i].count) ) 
									{
										//printf("servo Speed char : %c \n", pCmd[i].valueBuffer[actualChar]); //for debug
										ServoSpeed = ServoSpeed * 10 + ( pCmd[i].valueBuffer[actualChar] - '0' ); //convert character to integer
										//printf("temp Speed Int : %d \n", ServoSpeed); // for debugging
										actualChar++; //second servo number or space
									}
									//printf("Servo Speed Int : %d\n", ServoSpeed); // for debugging
									//convert to time 
									ServoTime = speedToTime(servos.pulse(ServoNumber), ServoPosition, ServoSpeed);
									//printf("Servo Actual Pos Int : %d\n", int(servos.pulse(ServoNumber))); // for debugging
									//printf("Servo Time Int : %d\n", ServoSpeed); // for debugging

								} else if ((pCmd[i].valueBuffer[actualChar] == 'T') && (actualChar < pCmd[i].count))
								{
									actualChar++; // skip T
									while ( (pCmd[i].valueBuffer[actualChar] != ' ')  && (actualChar < pCmd[i].count) ) 
									{
										//printf("servo Time char : %c \n", pCmd[i].valueBuffer[actualChar]); //for debug
										ServoTime = ServoTime * 10 + ( pCmd[i].valueBuffer[actualChar] - '0' ); //convert character to integer
										//printf("temp Time Int : %d \n", ServoTime); // for debugging
										actualChar++; //second servo number or space
									}
									//printf("Servo Time Int : %d\n", ServoTime); // for debugging
								} else 
								{
									ServoTime = 0;
								}
								
								if (pCmd[i].valueBuffer[actualChar] == ' ')
								{
									actualChar++; // skip space if one
								}
								break;

							} // while ( actualChar < pCmd[i].count )
							//update servos structure
							//to-do reverse servos that need it.
							ServosMove[ServoNumber].IsMoving = true;
							ServosMove[ServoNumber].StartPos = servos.pulse(ServoNumber);
							ServosMove[ServoNumber].EndPos = ServoPosition;
							ServosMove[ServoNumber].MoveStartTime = millis();
							ServosMove[ServoNumber].Time = ServoTime;
							printf("Moving Servo : %d from position %d to position %d Starttime: %lu Endtime: %d\n", ServoNumber, ServosMove[ServoNumber].StartPos,ServosMove[ServoNumber].EndPos,ServosMove[ServoNumber].MoveStartTime,ServosMove[ServoNumber].Time);
							break;
						}					
					case 'Q':
						{
							// query
							break;
						}
					default:
						break;
					} 
				
				
				}
			}
			else
			{
				printf("Not a command\n"); //for debug
			}
			
		/*flush the buffer after parsing*/
		curr_cmdPkt.count = 0;
		}
		
		else if ((input != '\r') && (curr_cmdPkt.count < MAX_COUNT_VALUE)) 
		/*add character to the buffer*/
		{
			//printf("Types character %c \n", input); //for debug
			curr_cmdPkt.valueBuffer[curr_cmdPkt.count] = input;
			curr_cmdPkt.count++;

		}
		
		/*Get next char if we have time*/	
		if ( ( millis() - start_loop ) < MAX_PARSE_TIME ) 
		{
			input = getchar_timeout_us(GETC_TIMEOUT_US);
		}
		else //break the parse loop for servos loop execution
		{
			break;
		}
	} // while (input != PICO_ERROR_TIMEOUT)
	
}
/*******************************************************************************
 ******************************************************************************/
/*******************************************************************************
 * Moving servos Functions
 ******************************************************************************/
void move_servo(void)
{
	//printf("Servo Time Int : %d\n", ServoTime); // for debugging
	//printf("Move Servos \n"); // for debugging
	for (auto currServo = START_PIN; currServo < NUM_SERVOS; currServo++) 
	{
		if (ServosMove[currServo].IsMoving) 
		{
			ulong Progress = millis() - ServosMove[currServo].MoveStartTime;
			uint nextPWM ; 
			//uint nextPWM = map(Progress, 0 ,ServosMove[currServo].Time,ServosMove[currServo].StartPos,ServosMove[currServo].EndPos);
			/*long map(long x, long in_min, long in_max, long out_min, long out_max) {
  					return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;}*/
			//nextPWM = (Progress)*(ServosMove[currServo].EndPos - ServosMove[currServo].StartPos) / (ServosMove[currServo].Time) + ServosMove[currServo].StartPos;
			nextPWM = map(Progress, 0 ,ServosMove[currServo].Time,ServosMove[currServo].StartPos,ServosMove[currServo].EndPos);
			printf("Moving Servo : %d To Pwm : %d\n", currServo, nextPWM);
			// constrain position
			if (nextPWM > 2500) 
			{
				nextPWM = 2500;
			} else if (nextPWM < 500) 
			{
				nextPWM = 500;
			}
			servos.pulse(currServo,nextPWM);
			if ( nextPWM == ServosMove[currServo].EndPos)
			{
				ServosMove[currServo].IsMoving = false;
			}
		}
	}
}

// return reversed servo value
uint reverseServoValue(uint Value)
{
	uint reversedValue = (1500-Value)+1500 ;
	return  reversedValue;
}

// calculate time to move for a given speed
uint speedToTime(uint startPos, uint endPos, uint speed)
{
	int travel = startPos - endPos ;
	uint absoluteMove = abs( travel );
	uint TimeInms = ( absoluteMove / speed ) * 1000;
	return TimeInms;   
}

//map function from https://www.arduino.cc/reference/en/language/functions/math/
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  					return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;}

/*******************************************************************************
 * VCP/Parsing Support Functions
 ******************************************************************************/
uint cmdPin_to_hardwarePin(cmdPins cmdPin)
{
	return RP_hardwarePins_table[cmdPin];
}
/*******************************************************************************
 ******************************************************************************/
void vcp_transmit(uint *txbuff, uint size)
{
	for (uint byte = 0; byte < size; byte++)
	{
		putchar_raw(txbuff[byte]);
	}
}

/*******************************************************************************
 * LED Support Functions
 ******************************************************************************/
void pendingVCP_ledSequence(void)
{
	static float offset = 0.0;
	const uint updates = 50;

	offset += 0.005;

	// Update all the LEDs
	for (auto i = 0u; i < servo2040::NUM_LEDS; i++)
	{
		float hue = (float)i / (float)servo2040::NUM_LEDS;
		led_bar.set_hsv(i, hue + offset, 1.0f, BRIGHTNESS);
	}

	sleep_ms(1000 / updates);
}

/*******************************************************************************
 * Sensing Support Functions
 ******************************************************************************/
float read_current(void)
{
	mux.select(servo2040::CURRENT_SENSE_ADDR);
	return (cur_adc.read_current());
}
/*******************************************************************************
 ******************************************************************************/
float read_voltage(void)
{
	mux.select(servo2040::VOLTAGE_SENSE_ADDR);
	return (vol_adc.read_voltage());
}
/*******************************************************************************
 ******************************************************************************/
float read_analogPin(uint sensorAddress)
{
	mux.select(sensorAddress);
	return (sen_adc.read_voltage());
}