#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>

#include <driverlib/gpio.h>
#include <driverlib/debug.h>
#include <driverlib/adc.h>
#include <driverlib/sysctl.h>

#include "ir_sensors.h"
#include "subsys.h"
#include "sumo.h"

version_t IR_VERSION;

ir_shortrange_data_t ir_shortrange_data;
ir_longrange_data_t ir_longrange_data;

uint8_t debuglong = 0;
uint8_t debugshort = 0;
uint8_t debugdir = 0;
uint16_t THRESHOLD = 1400;

// Callback function definition
void IR_LogCallback(char * cmd);

void IR_Init(){

	// Init ADC system peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	int i;

	// Set up the long range IR pins for ADC
	for(i = 0; i < IR_LONGRANGE_SENSORS; i++){
		GPIOPinTypeADC(ir_longrange[i].analog_gpio_port, ir_longrange[i].analog_gpio_pin);
	}

	/* PF0 requires unlocking before configuration */
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
	// Set up the enable pins for the SHARP sensors
	for(i = 0; i < IR_LONGRANGE_ENABLE_PINS; i++){
		GPIOPinTypeGPIOOutput(ir_longrange[i].enable_gpio_port, ir_longrange[i].enable_gpio_pin);
	}
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_M;

	// Set up the short range IR pins for ADC
	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
		GPIOPinTypeADC(ir_shortrange[i].analog_gpio_port, ir_shortrange[i].analog_gpio_pin);
	}

	// Set up the enable pins for the short range IR LEDs
	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
		GPIOPinTypeGPIOOutput(ir_shortrange[i].enable_gpio_port, ir_shortrange[i].enable_gpio_pin);
	}

	GPIOPinWrite(ir_longrange[1].enable_gpio_port, ir_longrange[0].enable_gpio_pin +  ir_longrange[1].enable_gpio_pin, ~(0));

	// Configure the sequencer
	ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 3);
	ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 2);

	ADCHardwareOversampleConfigure(ADC1_BASE, 64);

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
	RegisterCallback(IR, IR_LogCallback);
}

void IR_LogCallback(char * cmd) {
	//	LogMsg(MOTOR, MESSAGE, "CMD Received: %s", cmd);
	switch(*cmd) {
	case 'l':
		debuglong ^= 1;
		break;
	case 's':
		debugshort ^= 1;
		break;
	case 'd':
		debugdir ^= 1;
		break;
	case '1':
		THRESHOLD = 1000;
		break;
	case '2':
		THRESHOLD = 1050;
		break;
	case '3':
		THRESHOLD = 1100;
		break;
	case '4':
		THRESHOLD = 1150;
		break;
	case '5':
		THRESHOLD = 1200;
		break;
	case '6':
		THRESHOLD = 1250;
		break;
	case '7':
		THRESHOLD = 1300;
		break;
	case '8':
		THRESHOLD = 1350;
		break;
	case '9':
		THRESHOLD = 1400;
		break;
	case '0':
		THRESHOLD = 1450;
		break;
	case '-':
		THRESHOLD = 1500;
		break;
	case '=':
		THRESHOLD = 1550;
		break;
	default:
		break;

	}
}

void IR_PollLong(void){
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
	ADCSequenceDataGet(ADC1_BASE, 0, (ir_longrange_data.adc_ir_long_data));

	if(debuglong){
		LogMsg(IR, MESSAGE, "IR Long data: %d\t%d\t%d\t%d\t%d\t", ir_longrange_data.adc_ir_long_data[0],
				ir_longrange_data.adc_ir_long_data[1], ir_longrange_data.adc_ir_long_data[2],
				ir_longrange_data.adc_ir_long_data[3], ir_longrange_data.adc_ir_long_data[4]);
		DelayMs(100);
	}
}

void IR_PollShort(void){
	// Turn on the short range IRs
	int i =0;
	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
		GPIOPinWrite(ir_shortrange[i].enable_gpio_port, ir_shortrange[i].enable_gpio_pin, 1);
	}

	// Turn off the long range sensors
	// NOTE: setting the enable pin turns the IR off
	for(i = 0; i < IR_LONGRANGE_ENABLE_PINS; i++){
		GPIOPinWrite(ir_longrange[i].enable_gpio_port, ir_longrange[i].enable_gpio_pin, 1);
	}

	// Clear the flag
	ADCIntClear(ADC1_BASE, 2);

	// Trigger the ADC
	ADCProcessorTrigger(ADC1_BASE, 2);

	// Allow ADC to complete conversion
	while(!ADCIntStatus(ADC1_BASE, 2, false));

	// Clear the interrupt
	ADCIntClear(ADC1_BASE, 2);

	// Copy data from the ADC0 sample sequencer 0 to Buffer ir_data
	ADCSequenceDataGet(ADC1_BASE, 2, (ir_shortrange_data.adc_ir_short_data));

	// Turn off the short range IRs
	for(i = 0; i < IR_SHORTRANGE_SENSORS; i++){
		GPIOPinWrite(ir_shortrange[i].enable_gpio_port, ir_shortrange[i].enable_gpio_pin, 0);
	}

	// Turn on the long range sensors
	// NOTE: clearing the enable pin turns the IR on
	for(i = 0; i < IR_LONGRANGE_ENABLE_PINS; i++){
		GPIOPinWrite(ir_longrange[i].enable_gpio_port, ir_longrange[i].enable_gpio_pin, 0);
	}

	if(debugshort){
		LogMsg(IR, MESSAGE, "IR Short data: %d\t%d", ir_shortrange_data.adc_ir_short_data[0], ir_shortrange_data.adc_ir_short_data[1]);
		DelayMs(100);
	}
}

int16_t IR_GetFrontDiff(void){
	return (ir_longrange_data.front_left - ir_longrange_data.front_right);
}

void IR_Update(void){
	uint32_t temp = 0;
	uint8_t dir = 10;
	static uint8_t prev_dir = 10;
	int16_t long_diff = 0;
	uint16_t front_long_avg = 0;
	uint16_t short_diff = 0;
	bool poll_long = true;
	uint16_t IR_LONG_THRESHOLD[5] = {1400, 1400, 1200, 1200, 1200}; // we need to come up with another solution
	uint16_t IR_SHORT_THRESHOLD[2] = {500, 500}; // Check these values

	//	// poll the small IRs and determine if the large ones are needed or if the enemy is in front of us
	//	IR_PollShort();
	//	if(ir_shortrange_data->adc_ir_short_data[0] > IR_SHORT_THRESHOLD[0] || ir_shortrange_data->adc_ir_short_data[1] > IR_SHORT_THRESHOLD[1]){
	//		short_diff = ir_shortrange_data->adc_ir_short_data[0] - ir_shortrange_data->adc_ir_short_data[1];
	//		if(abs(short_diff < 500)){	// if both IRs are triggered
	//			poll_long = false;	// no need to use the long IRs
	//			dir = 5; // Directly in front of you
	//		}else if(ir_shortrange_data->adc_ir_short_data[0] > IR_SHORT_THRESHOLD[0]){ // if only the left IR was triggered
	//			poll_long = false;	// no need to use the long IRs
	//			dir = 0; // In front but to the left
	//		}else if(ir_shortrange_data->adc_ir_short_data[1] > IR_SHORT_THRESHOLD[1]){	// if only the right IR was triggered
	//			poll_long = false;	// no need to use the long IRs
	//			dir = 1; // In front but to the right
	//		}
	//	}
	if(poll_long){
		IR_PollLong();
		int i;
		for(i = 0; i < IR_LONGRANGE_SENSORS; i++){
			if(ir_longrange_data.adc_ir_long_data[i] > IR_LONG_THRESHOLD[i] && ir_longrange_data.adc_ir_long_data[i] > temp){
				// Do something here to determine where the enemy is with changing IR values.
				//if(ir_longrange_data->adc_ir_long_data[i]-IR_THRESHOLDS[i] > diff){
				//diff = ir_longrange_data->adc_ir_long_data[i]-IR_THRESHOLDS[i];
				temp = ir_longrange_data.adc_ir_long_data[i];
				dir = i;
				//}
			}
		}
		if(dir == 0 || dir == 1){
			if(ir_longrange_data.adc_ir_long_data[!dir] > IR_LONG_THRESHOLD[!dir]){
				dir = 5;
			}
//			// If one of the front IRs are triggered, is the enemy directly in front of you or off at an angle
//			long_diff = ir_longrange_data.left - ir_longrange_data.right;
//			front_long_avg = (ir_longrange_data.left - ir_longrange_data.right)/2;
//			if(abs(long_diff < 500)){
//				dir = 5; // Directly in front of you
//			}
		}
	}
	if(debugdir){
		LogMsg(IR, MESSAGE, "IR dir: %d", dir);
	}

	if(dir == 0 && prev_dir == 2){
		dir = 2;
	}

	if(dir == 1 && prev_dir == 3){
		dir = 3;
	}

	prev_dir = dir;

	if(SumoGetState() != IDLE && SumoGetState() != REVERSE && SumoGetState() != REVERSE_LEFT && SumoGetState() != REVERSE_RIGHT){
		switch(dir){
		case 0:
			SumoSetState(FRONT_LEFT); // to the front left
			break;
		case 1:
			SumoSetState(FRONT_RIGHT);	// to the front right
			break;
		case 2:
			SumoSetState(TURN_LEFT);	// to the left
			break;
		case 3:
			SumoSetState(TURN_RIGHT);	// to the right
			break;
		case 4:
			SumoSetState(TURN_LEFT);	// behind you
			break;
		case 5:
			SumoSetState(FORWARD);	// Directly in front
			break;
		default:
			//		SumoSetState(SEARCH);	// can't see anything, continue
			break;
		}
	}
}
