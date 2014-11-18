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

#include "ir_sensors.h"
#include "subsys.h"
#include "sumo.h"

version_t IR_VERSION;

enemy_state_t enemyState;

const char const * enemyState2String[]={"NONE", "FRONT", "LEFT", "RIGHT", "BACK"};


void ir_init(){

	// Init ADC system peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	int i;

	// Set up the long range IR pins for ADC
	for(i = 0; i < IR_LONGRANGE_SENSORS; i++){
		GPIOPinTypeADC(ir_longrange[i].analog_gpio_port, ir_longrange[i].analog_gpio_pin);
	}

	// Set up the enable pins for the SHARP sensors
	for(i = 0; i < IR_LONGRANGE_ENABLE_PINS; i++){
		GPIOPinTypeGPIOOutput(ir_longrange[i].enable_gpio_port, ir_longrange[i].enable_gpio_pin);
	}

	// Set up the short range IR pins for ADC
	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
		GPIOPinTypeADC(ir_shortrange[i].analog_gpio_port, ir_shortrange[i].analog_gpio_pin);
	}

	// Set up the enable pins for the short range IR LEDs
	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
		GPIOPinTypeGPIOOutput(ir_shortrange[i].enable_gpio_port, ir_shortrange[i].enable_gpio_pin);
	}

	// Configure the sequencer
	ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 3);
	ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 2);

	// Configure the steps in the ADC for long range
	ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH5);	// PD2 - front left long range IR
	ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADC_CTL_CH9);	// PE4 - front right long range IR
	ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADC_CTL_CH2); // PE1 - left long range IR
	ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADC_CTL_CH11);// PB5 - right long range
	ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);	// PE0 - back long range IR

	// Configure the steps in the ADC for short range
	ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH6);	// PD1 - front left close range IR
	ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH10 | ADC_CTL_IE | ADC_CTL_END);	// PB4 - front right close range IR

	// Enable the ADC Sequence
	ADCSequenceEnable(ADC1_BASE, 0);
	ADCSequenceEnable(ADC1_BASE, 2);

	IR_VERSION.word = 0x14110800LU;
	SubsystemInit(IR, MESSAGE, "IR", IR_VERSION);
}

void ir_poll_long(ir_longrange_data_t *ir_longrange_data){
	// IRs are on by default with SHARP sensors

	// Clear the flag
	ADCIntClear(ADC1_BASE, 0);

	// Trigger the ADC
	ADCProcessorTrigger(ADC1_BASE, 0);

	// Allow ADC to complete conversion
	while(!ADCIntStatus(ADC1_BASE, 0, false));

	// Clear the interrupt
	ADCIntClear(ADC1_BASE, 0);

	// Copy data from the ADC0 sample sequencer 0 to Buffer ir_data
	ADCSequenceDataGet(ADC1_BASE, 0, (ir_longrange_data->adc_ir_long_data));

//	LogMsg(IR, MESSAGE, "IR data: %d\t%d\t%d\t%d\t%d\t", ir_longrange_data->adc_ir_long_data[0],
//				ir_longrange_data->adc_ir_long_data[1], ir_longrange_data->adc_ir_long_data[2],
//				ir_longrange_data->adc_ir_long_data[3], ir_longrange_data->adc_ir_long_data[4]);

}

void ir_poll_short(ir_shortrange_data_t *ir_shortrange_data){
	// Turn on the short range IRs
//	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
//		GPIOPinWrite(ir_shortrange[i].enable_gpio_port, ir_shortrange[i].enable_gpio_pin, 1);
//	}
//
//	// Turn off the long range sensors
//	// NOTE: setting the enable pin turns the IR off
//	for(i = 0; i < IR_LONGRANGE_ENABLE_PINS; i++){
//		GPIOPinWrite(ir_longrange[i].enable_gpio_port, ir_longrange[i].enable_gpio_pin, 1);
//	}

	// Clear the flag
	ADCIntClear(ADC1_BASE, 2);

	// Trigger the ADC
	ADCProcessorTrigger(ADC1_BASE, 2);

	// Allow ADC to complete conversion
	while(!ADCIntStatus(ADC1_BASE, 2, false));

	// Clear the interrupt
	ADCIntClear(ADC1_BASE, 2);

	// Copy data from the ADC0 sample sequencer 0 to Buffer ir_data
	ADCSequenceDataGet(ADC1_BASE, 2, (ir_shortrange_data->adc_ir_short_data));

	// Turn off the short range IRs
//	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
//		GPIOPinWrite(ir_shortrange[i].enable_gpio_port, ir_shortrange[i].enable_gpio_pin, 0);
//	}
//
//	// Turn on the long range sensors
//	// NOTE: clearing the enable pin turns the IR on
//	for(i = 0; i < IR_LONGRANGE_ENABLE_PINS; i++){
//		GPIOPinWrite(ir_longrange[i].enable_gpio_port, ir_longrange[i].enable_gpio_pin, 0);
//	}
}

void EnemySetState(enemy_state_t newstate){
	if(enemyState != newstate){
		LogMsg(IR, MESSAGE, "Enemy State Updated: %s", enemyState2String[newstate]);
	}
	enemyState = newstate;
}

enemy_state_t EnemyGetState(void){
	return enemyState;
}

void update_ir(ir_longrange_data_t *ir_longrange_data){
	// Poll the large IRs, determine if the short IRs are needed, set state
	LogMsg(IR, MESSAGE, "IR_UPDATE");
	ir_poll_long(ir_longrange_data);
	uint32_t temp = 0;
	uint8_t dir = 10;
	uint16_t diff = 0;
	uint16_t IR_THRESHOLDS[5] = {1200, 1200, 1100, 800, 1100};
	int i;
	for(i=0; i<IR_LONGRANGE_SENSORS; i++){
		if(ir_longrange_data->adc_ir_long_data[i] > temp && ir_longrange_data->adc_ir_long_data[i] > IR_THRESHOLDS[i]){
			if(ir_longrange_data->adc_ir_long_data[i]-IR_THRESHOLDS[i] > diff){
				diff = ir_longrange_data->adc_ir_long_data[i]-IR_THRESHOLDS[i];
				temp = ir_longrange_data->adc_ir_long_data[i];
				dir = i;
			}
		}
	}
	switch(dir){
		case 0:
		case 1:
			EnemySetState(FRONT);
//			SumoSetState(ATTACK);
			break;
		case 2:
			EnemySetState(LEFT);
//			SumoSetState(SEARCH);
			break;
		case 3:
			EnemySetState(RIGHT);
//			SumoSetState(SEARCH);
			break;
		case 4:
			EnemySetState(BACK);
//			SumoSetState(SEARCH);
			break;
		default:
			EnemySetState(NONE);
//			SumoSetState(SEARCH);
			break;
	}
//	DelayMs(200);
}
