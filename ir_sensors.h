#ifndef _IR_SENSORS_H
#define _IR_SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>

#include <driverlib/gpio.h>
#include <driverlib/debug.h>
#include <driverlib/adc.h>
#include <driverlib/sysctl.h>

#define IR_LONGRANGE_SENSORS 5
#define IR_LONGRANGE_ENABLE_PINS 2
#define IR_SHORTRANGE_SENSORS 2


typedef enum enemy_location_t {
	NONE = 0,
	FRONT,
	LEFT,
	RIGHT,
	BACK
} enemy_state_t;

// Long range IR config structure
typedef struct {
	uint32_t analog_gpio_port;
	uint8_t analog_gpio_pin;
	uint32_t enable_gpio_port;
	uint8_t enable_gpio_pin;
} ir_longrange_config_t;


// Long range SHARP IR sensors configuration setup.
// NOTE: any IRs that use the enable pin must be listed first!
static ir_longrange_config_t ir_longrange[IR_LONGRANGE_SENSORS] = {
	{// Front left (PD2)
		GPIO_PORTD_BASE,
		GPIO_PIN_2,
		GPIO_PORTD_BASE,	// enable pin
		GPIO_PIN_7
	},
	{// Front right (PE4)
		GPIO_PORTE_BASE,
		GPIO_PIN_4,
		GPIO_PORTD_BASE,	// enable pin
		GPIO_PIN_4
	},
	{// Left (PE1)
		GPIO_PORTE_BASE,
		GPIO_PIN_1,
		0,
		0
	},
	{// Right (PB5)
		GPIO_PORTB_BASE,
		GPIO_PIN_5,
		0,
		0
	},
	{// Back (PE0)
		GPIO_PORTE_BASE,
		GPIO_PIN_0,
		0,
		0
	}
};


// Long range IR data storage
typedef union{

	uint32_t adc_ir_long_data[8];

	struct{
		uint32_t front_left;
		uint32_t front_right;
		uint32_t left;
		uint32_t right;
		uint32_t back;
		uint32_t filler1;
		uint32_t filler2;
		uint32_t filler3;
	};
} ir_longrange_data_t;

// Short range IR config structure
typedef struct {
	uint32_t analog_gpio_port;
	uint8_t analog_gpio_pin;
	uint32_t enable_gpio_port;
	uint8_t enable_gpio_pin;
} ir_shortrange_config_t;

// Short range IR configuration setup
static ir_shortrange_config_t ir_shortrange[IR_SHORTRANGE_SENSORS] = {
	{// Front left (PD1)
		GPIO_PORTD_BASE,
		GPIO_PIN_1,
		GPIO_PORTA_BASE,	// enable pin
		GPIO_PIN_0
	},
	{// Front right (PB4)
		GPIO_PORTB_BASE,
		GPIO_PIN_4,
		GPIO_PORTA_BASE,	// enable pin
		GPIO_PIN_0
	}
};

// Short range IR data storage
typedef union{

	uint32_t adc_ir_short_data[4];

	struct{
		uint32_t front_left;
		uint32_t front_right;
		uint32_t filler1;
		uint32_t filler2;
	};
} ir_shortrange_data_t;

void ir_init(void);
void ir_poll_long(ir_longrange_data_t *ir_longrange_data);
void ir_poll_short(ir_shortrange_data_t * ir_shortrange_data);
void SumoSetEnemyState(enemy_state_t);
enemy_state_t SumoGetEnemyState(void);
void update_ir(ir_longrange_data_t *ir_longrange_data);

#endif
