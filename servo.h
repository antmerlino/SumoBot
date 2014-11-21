#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_pwm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <driverlib/pin_map.h>

#include <driverlib/sysctl.h>

#define PWM_FREQ 50 // refresh rate 20 ms
#define PWM_DIV 64

#define MIN_PULSE_WIDTH	450
#define MAX_PULSE_WIDTH	1450
#define MAX_POSITION	180

#define SPAN_VALUE (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)

typedef struct{
	uint8_t gpio_port;
	uint8_t gpio_pin;
	uint32_t pwm_module;
	uint32_t pwm_base;
	uint8_t pwm_gen;
	uint8_t pwm_pin;
	uint16_t duty;
} servo_config_t;

static servo_config_t servo_config = {
	// Configure the Servo settings
	GPIO_PORTB_BASE,
	GPIO_PIN_6,
	GPIO_PB6_M0PWM0,
	PWM1_BASE,
	PWM_GEN_2,
	PWM_OUT_7,
	1200 // Arbitrary number between 450 and 1450
};

// Init software PWM system
void servo_init(void);

// Set the position of the servo attached to channel
void servo_set_position(uint32_t pos);

#endif
