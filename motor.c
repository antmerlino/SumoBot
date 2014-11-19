/*
 * motor.c
 *
 *  Created on: Mar 18, 2014
 *      Author: Anthony Merlino and Brad Ebinger
 */

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/pwm.h>
#include <driverlib/sysctl.h>

#include "system.h"
#include "subsys.h"
#include "motor.h"


static inline calc_cycles(uint32_t us) {return (PWM_TIMER_FREQ/1000000.0)*us;}
static inline calc_usec(uint32_t cycles) {return (1000000.0/PWM_TIMER_FREQ)*cycles;}

version_t MOTOR_VERSION;

motor_state_t motors[4];

// Callback function definition
void MotorLogCallback(char * cmd);

void MotorsInit(void) {

	uint8_t pin_mask;

	// Allow register access to PWM1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	// Allow register access to GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Configure PF0, PF1, PF2, and PF3 as PWM outputs
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	GPIOPinConfigure(GPIO_PF3_M1PWM7);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	/* PF0 requires unlocking before configuration */
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
	GPIOPinConfigure(GPIO_PF0_M1PWM4);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_M;

	// Configure the direction and standby pins as outputs
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);

	// Set the clock to be
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	// Setup Front Left Motor
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, calc_cycles(MOTOR_PERIOD));
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);
	pin_mask = 1 << (0x0000000F & PWM_OUT_6);
	PWMOutputState(PWM1_BASE, pin_mask, 0);

	// Setup Front Right Motor
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, calc_cycles(MOTOR_PERIOD));
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);
	pin_mask = 1 << (0x0000000F & PWM_OUT_7);
	PWMOutputState(PWM1_BASE, pin_mask, 0);

	// Setup Back Left Motor
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, calc_cycles(MOTOR_PERIOD));
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 0);
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	pin_mask = 1 << (0x0000000F & PWM_OUT_4);
	PWMOutputState(PWM1_BASE, pin_mask, 0);

	// Setup Back Right Motor
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, calc_cycles(MOTOR_PERIOD));
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 0);
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	pin_mask = 1 << (0x0000000F & PWM_OUT_5);
	PWMOutputState(PWM1_BASE, pin_mask, 0);

	// Add debugging option
	MOTOR_VERSION.word = 0x14110800LU;
	SubsystemInit(MOTOR, MESSAGE, "MOTOR", MOTOR_VERSION);
	RegisterCallback(MOTOR, MotorLogCallback);
}

void MotorsEnableFront(){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1, ON);
}

void MotorsEnableBack(){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, ON);
}

void MotorsDisableFront(){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1, OFF);
}

void MotorsDisableBack(){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, OFF);
}

void MotorLogCallback(char * cmd) {
//	LogMsg(MOTOR, MESSAGE, "CMD Received: %s", cmd);
	switch(*cmd) {
	case 'y':
		MotorsEnableFront();
		MotorsEnableBack();
		break;
	case 'n':
		MotorsDisableFront();
		MotorsDisableBack();
		break;
	}
}

void MotorsUpdate(){
	uint8_t pin_mask;
	uint32_t cycles;

	// Update Front Left Motor
	switch(motors[FRONTLEFT_MOTOR].direction){
		case CW:
			// Set the pins so that the direction is Clockwise
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ON);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, OFF);
			break;
		case CCW:
			// Set the pins so that the direction is Counter-Clockwise
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, OFF);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, ON);
			break;
		case BRAKE:
			// Set the pins so that we brake
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ON);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, ON);
			break;
		case STOP:
			// Set the pins so we stop
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, OFF);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, OFF);
			break;
	}

	cycles = calc_cycles(MOTOR_PERIOD*motors[FRONTLEFT_MOTOR].duty_tenths_perc/1000);
	pin_mask = 1 << (0x0000000F & PWM_OUT_6);
	if(cycles <= 0){
		PWMOutputState(PWM1_BASE, pin_mask, 0);
	}
	else{
		PWMOutputState(PWM1_BASE, pin_mask, 1);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, cycles);
	}

	// Update Front Right Motor
	switch(motors[FRONTRIGHT_MOTOR].direction){
		case CW:
			// Set the pins so that the direction is Clockwise
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, ON);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, OFF);
			break;
		case CCW:
			// Set the pins so that the direction is Counter-Clockwise
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, OFF);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, ON);
			break;
		case BRAKE:
			// Set the pins so that we brake
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, ON);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, ON);
			break;
		case STOP:
			// Set the pins so we stop
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, OFF);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, OFF);
			break;
	}

	cycles = calc_cycles(MOTOR_PERIOD*motors[FRONTRIGHT_MOTOR].duty_tenths_perc/1000);
	pin_mask = 1 << (0x0000000F & PWM_OUT_7);
	if(cycles <= 0){
		PWMOutputState(PWM1_BASE, pin_mask, 0);
	}
	else{
		PWMOutputState(PWM1_BASE, pin_mask, 1);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, cycles);
	}

	// Update Back Left Motor
	switch(motors[BACKLEFT_MOTOR].direction){
		case CW:
			// Set the pins so that the direction is Clockwise
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, ON);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, OFF);
			break;
		case CCW:
			// Set the pins so that the direction is Counter-Clockwise
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, OFF);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ON);
			break;
		case BRAKE:
			// Set the pins so that we brake
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, ON);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ON);
			break;
		case STOP:
			// Set the pins so we stop
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, OFF);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, OFF);
			break;
	}

	cycles = calc_cycles(MOTOR_PERIOD*motors[BACKLEFT_MOTOR].duty_tenths_perc/1000);
	pin_mask = 1 << (0x0000000F & PWM_OUT_4);
	if(cycles <= 0){
		PWMOutputState(PWM1_BASE, pin_mask, 0);
	}
	else{
		PWMOutputState(PWM1_BASE, pin_mask, 1);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, cycles);
	}

	// Update Back Right Motor
	switch(motors[BACKRIGHT_MOTOR].direction){
		case CW:
			// Set the pins so that the direction is Clockwise
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, ON);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, OFF);
			break;
		case CCW:
			// Set the pins so that the direction is Counter-Clockwise
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, OFF);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, ON);
			break;
		case BRAKE:
			// Set the pins so that we brake
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, ON);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, ON);
			break;
		case STOP:
			// Set the pins so we stop
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, OFF);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, OFF);
			break;
	}

	cycles = calc_cycles(MOTOR_PERIOD*motors[BACKRIGHT_MOTOR].duty_tenths_perc/1000);
	pin_mask = 1 << (0x0000000F & PWM_OUT_5);
	if(cycles <= 0){
		PWMOutputState(PWM1_BASE, pin_mask, 0);
	}
	else{
		PWMOutputState(PWM1_BASE, pin_mask, 1);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, cycles);
	}

}
