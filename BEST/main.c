#pragma config(UART_Usage, UART1, uartUserControl, baudRate1200, IOPins, None, None)
#pragma config(Motor,  port2,           leftDrive,     tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port3,           rightDrive,    tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port4,           elevator,      tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port5,           lever1,        tmotorServoStandard, openLoop)
#pragma config(Motor,  port6,           lever2,        tmotorServoStandard, openLoop)
#pragma config(Motor,  port7,           dropper,       tmotorServoStandard, openLoop)
#pragma config(Motor,  port8,           IR,            tmotorServoStandard, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// custom baud rate set code

typedef unsigned long  uint32_t;
typedef unsigned short uint16_t;

typedef struct
{
  uint16_t SR;
  uint16_t RESERVED0;
  uint16_t DR;
  uint16_t RESERVED1;
  uint16_t BRR;
  uint16_t RESERVED2;
  uint16_t CR1;
  uint16_t RESERVED3;
  uint16_t CR2;
  uint16_t RESERVED4;
  uint16_t CR3;
  uint16_t RESERVED5;
  uint16_t GTPR;
  uint16_t RESERVED6;
} USART_TypeDef;

/* Peripheral memory map */
#define PERIPH_BASE           ((unsigned long)0x40000000)
#define APB1PERIPH_BASE       PERIPH_BASE
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define USART2                ((USART_TypeDef *) USART2_BASE)
#define USART3                ((USART_TypeDef *) USART3_BASE)

void setBaud( const TUARTs nPort, int baudRate ) {
    uint32_t tmpreg = 0x00, apbclock = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    /* pclk1 - 36MHz */
    apbclock = 36000000;

    /* Determine the integer part */
    integerdivider = ((0x19 * apbclock) / (0x04 * (baudRate)));
    tmpreg = (integerdivider / 0x64) << 0x04;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (0x64 * (tmpreg >> 0x04));
    tmpreg |= ((((fractionaldivider * 0x10) + 0x32) / 0x64)) & 0x0F;

    /* Write to USART BRR */
    USART_TypeDef *uart = USART2;
    if( nPort == UART2 ) {
      uart = USART3;
    }
    uart->BRR = (uint16_t)tmpreg;
}

//Trackers
bool reversed = false;
bool down = false;
bool driving = true;

//Lever Values
int forward_full = 127;
int neutral = 0;
int back_full = -127;

//IR Codes
int drive_low = 0x99;
int drive_med = 0xA5;
int drive_high = 0xC3;
int rotate_low = 0x69;
int rotate_med = 0x96;
int rotate_high = 0x0F;
int drive = 0x33;
int rotate = 0x5A;
int lift = 0x3C;

task main()
{
	setBaud(UART1, 600);
	motor[IR] = 127;
	while(true) {
		//Robot control
		if(driving) {
			//Driving
			if(!reversed) {
				if(abs(vexRT[Ch3]) > 20) {
					motor[leftDrive] = vexRT[Ch3];
				}
				else {
					motor[leftDrive] = 0;
				}
				if(abs(vexRT[Ch2]) > 20) {
					if(vexRT[Btn7D] == 1) {
						motor[leftDrive] = vexRT[Ch2];
						motor[rightDrive] = vexRT[Ch2] * 0.9;
					}
					else {
						motor[rightDrive] = vexRT[Ch2];
					}
				}
				else {
					motor[rightDrive] = 0;
				}
			}
			else {
				if(abs(vexRT[Ch3]) > 20) {
					motor[rightDrive] = -vexRT[Ch3];
				}
				else {
					motor[rightDrive] = 0;
				}
				if(abs(vexRT[Ch2]) > 20) {
					if(vexRT[Btn7D] == 1) {
						motor[leftDrive] = -vexRT[Ch2];
						motor[rightDrive] = -vexRT[Ch2] * 0.9;
					}
					else {
						motor[leftDrive] = -vexRT[Ch2];
					}
				}
				else {
					motor[leftDrive] = 0;
				}
			}
			if(vexRT[Btn6D] == 1) {
				while (vexRT[Btn6D] == 1) {}
				reversed = !reversed;
			}

		  //Wheel Droppers
		  if(vexRT[Btn7R] == 1) {
				motor[dropper] = 85;
			}
			else if(vexRT[Btn7L] == 1) {
		  	motor[dropper] = -85;
		  }
		  else {
		  	motor[dropper] = 0;
		  }

		  //Elevator
		  if(vexRT[Btn5U] == 1) {
				motor[elevator] = 100;
			}
			else if(vexRT[Btn5D] == 1) {
				motor[elevator] = -100;
			}
			else {
				motor[elevator] = 0;
			}

		  //IR Stick
			if(vexRT[Btn6U] == 1) {
				while(vexRT[Btn6U] == 1) {}
				if(down) {
					motor[IR] = 127;
				}
				else {
					motor[IR] = -85;
				}
				down = !down;
			}

			//Squeaky Mode
			if(vexRT[Btn8R] == 1) {
				while(vexRT[Btn8R] == 1) {}
				driving = false;
			}
		}

		//Squeaky Control
		else {
			//Driving
			if(abs(vexRT[Ch2]) > 20) {
				sendChar(UART1, drive);
				wait(0.2);
				motor[lever1] = vexRT[Ch2];
				motor[lever2] = vexRT[Ch2];
			}

			//Rotation
			else if(abs(vexRT[Ch4]) > 20) {
				sendChar(UART1, rotate);
				wait(0.2);
				motor[lever1] = vexRT[Ch4];
				motor[lever2] = vexRT[Ch4];
			}

			//Lift
			else if(vexRT[Btn5U] == 1) {
				sendChar(UART1, lift);
				wait(0.2);
				motor[lever1] = back_full;
				motor[lever2] = back_full;
			}
			else if(vexRT[Btn6U] == 1) {
				sendChar(UART1, lift);
				wait(0.2);
				motor[lever1] = forward_full;
				motor[lever2] = forward_full;
			}
			else {
				motor[lever1] = neutral;
				motor[lever2] = neutral;
				sendChar(UART1, drive_high);
				sendChar(UART1, rotate_high);
				wait(0.2);
			}

			//Driving Mode
			if(vexRT[Btn8R] == 1) {
				while(vexRT[Btn8R] == 1) {}
				driving = true;
			}
		}
	}
}
