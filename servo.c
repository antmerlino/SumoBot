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
void servo_init(){
	// Set up PWM clock
	// set to SYSCLK/64 = 625KHz
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	// Enable pwm modules
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// Enable servo pwm output port/pin
	GPIOPinTypePWM(servo_config.gpio_port, servo_config.gpio_pin);

	// Configure servo pwm module
	GPIOPinConfigure(servo_config.pwm_module);


	uint32_t ui32PWMClock = (SysCtlClockGet() / PWM_DIV); // get the current pwmClock in Hz (Currently 40000000/64 = 625 KHz)
	ui16Period = (ui32PWMClock / PWM_FREQ); // value loaded into the register (Currently 12500) ~20 ms period
	PWMGenConfigure(servo_config.pwm_base, servo_config.pwm_gen, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(servo_config.pwm_base, servo_config.pwm_gen, ui16Period);

	PWMPulseWidthSet(servo_config.pwm_base, servo_config.pwm_pin, ui16Period * .05); // ~1 ms - 0 degrees
	PWMOutputState(servo_config.pwm_base, (1 << (0xF & servo_config.pwm_pin)), true);
	PWMGenEnable(servo_config.pwm_base, servo_config.pwm_gen);

	// Add debugging option
	SERVO_VERSION.word = 0x14110800LU;
	SubsystemInit(SERVO, MESSAGE, "SERVO", SERVO_VERSION);
	RegisterCallback(SERVO, ServoLogCallback);
}

void ServoLogCallback(char * cmd) {
	//	LogMsg(MOTOR, MESSAGE, "CMD Received: %s", cmd);
	switch(*cmd){
	case '0':
		servo_set_position(0);
		break;
	case '1':
		servo_set_position(90);
		break;
	case '2':
		servo_set_position(180);
		break;
	default:
		break;
	}
}

// Set the position of the servo attached to channel
void servo_set_position(uint32_t position){
	// Scale the position
	position = position * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)/MAX_POSITION + MIN_PULSE_WIDTH;

	if(position > MAX_PULSE_WIDTH){
		position = MAX_PULSE_WIDTH;
	}else if(position < MIN_PULSE_WIDTH){
		position = MIN_PULSE_WIDTH;
	}
	PWMPulseWidthSet(servo_config.pwm_base, servo_config.pwm_pin, position);

}


