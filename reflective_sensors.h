#ifndef _REFLECTIVE_SENSORS_H
#define _REFLECTIVE_SENSORS_H

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

#define NUM_REFLECTIVE_SENSORS 3
#define REFLECTIVE_REF_LOW  500
#define REFLECTIVE_REF_HIGH 600

typedef union{

	uint32_t adc_reflective_data[4];

	struct{
		uint32_t left;
		uint32_t center;
		uint32_t right;
		uint32_t filler1;
	};
} reflective_data_t;

typedef struct{
	uint32_t analog_gpio_port;
	uint8_t analog_gpio_pin;
} reflective_adc_config_t;

static reflective_adc_config_t reflective[NUM_REFLECTIVE_SENSORS] = {
	{// Left Reflective (PD3)
		GPIO_PORTD_BASE,
		GPIO_PIN_3
	},
	{// Center Reflective (PD0)
		GPIO_PORTD_BASE,
		GPIO_PIN_0
	},
	{// Right Reflective (PE5)
		GPIO_PORTE_BASE,
		GPIO_PIN_5
	},
};

void ReflectiveInit(void);
void ReflectiveISR(void);

#endif
