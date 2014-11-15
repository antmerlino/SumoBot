/**
 * @file motor.h
 *
 * @defgroup motor Motor Subsystem
 *
 *  Created on: Mar 15, 2014
 *      @author: Bradley Ebinger, Anthony Merlino
 *
 * @version 2014.11.08
 * @{
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <driverlib/pwm.h>


#define OFF 0
#define ON (~0)

/**
 * PWM Clock Divider
 */
#define PWM_CLK_DIV 64

/**
 * Frequency of PWM timer in Hz
 */
#define PWM_TIMER_FREQ (PERIPHERAL_CLOCK/PWM_CLK_DIV)

/**
 * Period of motor PWM signals
 * NOTE: About 52000 us is the high limit for period!
 */
#define MOTOR_PERIOD 10000
/**
 * Enumeration providing an index to each motor
 */
typedef enum  {
	FRONTLEFT_MOTOR = 0,
	FRONTRIGHT_MOTOR,
	BACKLEFT_MOTOR,
	BACKRIGHT_MOTOR
} motor_index_t;

/**
 * Enumeration for the different states of motor direction
 */
typedef enum  {
	CW,
	CCW,
	BRAKE,
	STOP
} motor_direction_t;

/**
 * Holds PWM information for each motor
 *
 *  - Direction
 *  - Module Generator
 */
typedef struct {
	motor_direction_t direction;
	uint16_t duty_tenths_perc;
} motor_state_t;

extern motor_state_t motors[4];

/**
 * Initialize all pins and register with system using GPIO libraries.
 * NOTE: If a new motor is added to the pwm_motors list, please make
 * 		 it be initialized in this function.
 */
void MotorsInit(void);

void MotorsEnableFront();
void MotorsEnableBack();
void MotorsDisableFront();
void MotorsDisableBack();

void MotorsUpdate();

#endif /* MOTOR_H_ */
/** @}*/
