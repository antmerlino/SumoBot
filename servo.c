#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"

#include "system.h"
#include "subsys.h"
#include "servo.h"

version_t SERVO_VERSION;

uint16_t ui16Period; // Change from 32 bit to 16 bit.

// Callback function definition
void ServoLogCallback(char * cmd);

// Init software PWM system
void ServoInit(){

	uint8_t pin_mask;

	// Enable pwm modules
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// Configure servo pwm module
	GPIOPinConfigure(servo_config.pwm_module);

	// Enable servo pwm output port/pin
	GPIOPinTypePWM(servo_config.gpio_port, servo_config.gpio_pin);

	uint32_t ui32PWMClock = (SysCtlClockGet() / PWM_DIV); // get the current pwmClock in Hz (Currently 40000000/64 = 625 KHz)
	ui16Period = (ui32PWMClock / PWM_FREQ); // value loaded into the register (Currently 12500) ~20 ms period
	PWMGenConfigure(servo_config.pwm_base, servo_config.pwm_gen, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(servo_config.pwm_base, servo_config.pwm_gen, ui16Period);
	ServoSetPosition(170);
	PWMGenEnable(servo_config.pwm_base, servo_config.pwm_gen);
	pin_mask = 1 << (0x0000000F & servo_config.pwm_pin);
	PWMOutputState(servo_config.pwm_base, pin_mask, true);

	// Add debugging option
	SERVO_VERSION.word = 0x14110800LU;
	SubsystemInit(SERVO, MESSAGE, "SERVO", SERVO_VERSION);
	RegisterCallback(SERVO, ServoLogCallback);
}

void ServoLogCallback(char * cmd) {
	//	LogMsg(MOTOR, MESSAGE, "CMD Received: %s", cmd);
	switch(*cmd){
	case '0':
		ServoSetPosition(170);
		break;
	case '1':
		ServoSetPosition(27);
		break;
	default:
		break;
	}
}

// Set the position of the servo attached to channel
void ServoSetPosition(double position){
	// Scale the position
	position = position * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)/MAX_POSITION + MIN_PULSE_WIDTH;

	if(position > MAX_PULSE_WIDTH){
		position = MAX_PULSE_WIDTH;
	}else if(position < MIN_PULSE_WIDTH){
		position = MIN_PULSE_WIDTH;
	}
	PWMPulseWidthSet(servo_config.pwm_base, servo_config.pwm_pin, position);

}


