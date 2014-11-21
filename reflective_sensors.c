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
#include <driverlib/interrupt.h>

#include "reflective_sensors.h"
#include "timing.h"
#include "system.h"
#include "subsys.h"
#include "sumo.h"

#define COMPARATOR_0 1
#define COMPARATOR_1 2
#define COMPARATOR_2 4

version_t REFLECT_VERSION;

uint8_t DEBUG = 0;

// Callback function definition
void REFLECTIVE_LogCallback(char * cmd);

void ReflectiveInit(){
	// Init ADC system peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_3);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);

	//Enable register access to ADC0
	SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC0 );

	//Set up the ADC sequencer
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_ALWAYS, 1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH7|ADC_CTL_CMP0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH8|ADC_CTL_CMP1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH4 | ADC_CTL_CMP2 | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);

	//Set up comparator 0
	ADCComparatorConfigure(ADC0_BASE, 0, ADC_COMP_TRIG_NONE | ADC_COMP_INT_LOW_HONCE );
	ADCComparatorRegionSet(ADC0_BASE, 0, REFLECTIVE_REF_LOW, REFLECTIVE_REF_HIGH);
	ADCComparatorReset(ADC0_BASE, 0, true, true);
	ADCComparatorIntEnable(ADC0_BASE, 1);

	//Set up comparator 1
	ADCComparatorConfigure(ADC0_BASE, 1, ADC_COMP_TRIG_NONE | ADC_COMP_INT_LOW_HONCE );
	ADCComparatorRegionSet(ADC0_BASE, 1, REFLECTIVE_REF_LOW, REFLECTIVE_REF_HIGH);
	ADCComparatorReset(ADC0_BASE, 1, true, true);
	ADCComparatorIntEnable(ADC0_BASE, 1);

	//Set up comparator 2
	ADCComparatorConfigure(ADC0_BASE, 2, ADC_COMP_TRIG_NONE | ADC_COMP_INT_LOW_HONCE );
	ADCComparatorRegionSet(ADC0_BASE, 2, REFLECTIVE_REF_LOW, REFLECTIVE_REF_HIGH);
	ADCComparatorReset(ADC0_BASE, 2, true, true);
	ADCComparatorIntEnable(ADC0_BASE, 1);

	ADCIntRegister(ADC0_BASE, 1, ReflectiveISR);
	ADCIntEnable(ADC0_BASE, 1);

	REFLECT_VERSION.word = 0x14110800LU;
	SubsystemInit(REFLECT, MESSAGE, "REFLECT", REFLECT_VERSION);
	RegisterCallback(IR, REFLECTIVE_LogCallback);
}

void REFLECTIVE_LogCallback(char * cmd) {
	//	LogMsg(MOTOR, MESSAGE, "CMD Received: %s", cmd);
	switch(*cmd) {
	case 'd':
		DEBUG ^= 1;
		break;
	default:
		break;
	}
}

void ReflectiveISR(void) {
	//Determine which comparator caused the interrupt
	uint32_t comparatorStatus = ADCComparatorIntStatus(ADC0_BASE);

	// Clear the interrupt
	ADCComparatorIntClear(ADC0_BASE, 0x0F); // Clear the interrupt

	if(SumoGetState() != IDLE && SumoGetState() != REVERSE && SumoGetState() != REVERSE_LEFT && SumoGetState() != REVERSE_RIGHT){
		switch(comparatorStatus){
		case COMPARATOR_0:
			if(DEBUG){
				LogMsg(REFLECT, MESSAGE, "Left Sensor Detected Edge");
			}
			if(SumoGetState() != IDLE){
				SumoSetState(REVERSE_RIGHT);
			}
			break;
		case COMPARATOR_1:
			if(DEBUG){
				LogMsg(REFLECT, MESSAGE, "Center Sensor Detected Edge");
			}
			if(SumoGetState() != IDLE){
				SumoSetState(REVERSE);
			}
			break;
		case COMPARATOR_2:
			if(DEBUG){
				LogMsg(REFLECT, MESSAGE, "Right Sensor Detected Edge");
			}
			if(SumoGetState() != IDLE){
				SumoSetState(REVERSE_LEFT);
			}
			break;
		}
	}

	//		if(SumoGetState() != IDLE){
	//			SumoSetState(REVERSE);
	//		}
}

//// Do we really need this? Why not just leave the IRs on and have the interrupt thrown when we see white?
////	Possibly call this function when we know we are pushing an enemy to tell us to disregard the reflective sensors is they see white
//void ReflectivePoll(reflective_data_t *reflective_data){
//	// Clear the flag
//	ADCIntClear(ADC0_BASE, 1);
//
//	// Trigger the ADC, sequencer 1
//	ADCProcessorTrigger(ADC0_BASE, 1);
//
//	// Allow ADC to complete conversion
//	while(!ADCIntStatus(ADC0_BASE, 1, false));
//
//	// Clear the interrupt
//	ADCIntClear(ADC0_BASE, 1);
//
//	// Copy data from the ADC1 sample sequencer 1 to Buffer reflective_data
//	ADCSequenceDataGet(ADC0_BASE, 1, (reflective_data->adc_reflective_data));
//}
