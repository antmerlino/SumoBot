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

int i;

void ReflectiveInit(){
	// Init ADC system peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_3);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);

	// Configure the sequencer ADC1, sequencer 1, priority 3
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_ALWAYS, 3);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH7 | ADC_CTL_CMP0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH7 | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);

	// Configure the steps in the ADC to sample PD3, PD0, PE5
//	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH4 | ADC_CTL_CMP0);	// PD3 - left reflective sensor
//	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH7 | ADC_CTL_CMP1);	// PD0 - center reflective sensor
//	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH8 | ADC_CTL_CMP2);	// PE5 - right reflective sensor
//	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH7 | ADC_CTL_END);	// must sample the last channel twice when using comparators

	// Configure the comparator
	ADCComparatorConfigure(ADC0_BASE, 0, ADC_COMP_TRIG_NONE | ADC_COMP_INT_HIGH_HONCE); // Double check this configuration
//	ADCComparatorConfigure(ADC0_BASE, 1, ADC_COMP_TRIG_NONE | ADC_COMP_INT_HIGH_HONCE); // Double check this configuration
//	ADCComparatorConfigure(ADC0_BASE, 2, ADC_COMP_TRIG_NONE | ADC_COMP_INT_HIGH_HONCE); // Double check this configuration

	ADCComparatorRegionSet(ADC0_BASE, 0, REFLECTIVE_REF_LOW, REFLECTIVE_REF_HIGH); // Need to test and calibrate these Reference points
//	ADCComparatorRegionSet(ADC0_BASE, 1, REFLECTIVE_REF_LOW, REFLECTIVE_REF_HIGH); // Need to test and calibrate these Reference points
//	ADCComparatorRegionSet(ADC0_BASE, 2, REFLECTIVE_REF_LOW, REFLECTIVE_REF_HIGH); // Need to test and calibrate these Reference points

	ADCComparatorReset(ADC0_BASE, 0, true, true);
//	ADCComparatorReset(ADC0_BASE, 1, true, true);
//	ADCComparatorReset(ADC0_BASE, 2, true, true);

	ADCComparatorIntEnable(ADC0_BASE, 1);

	// Register the interrupt handler for the ADC interrupt on ADC1, seq. 1
	ADCIntRegister(ADC0_BASE, 1, ReflectiveISR);

	// Enable sample sequencer 2 interrupt enable
	ADCIntEnable(ADC0_BASE, 1);
}

void ReflectiveISR(void){
	// Clear the interrupt
	ADCComparatorIntClear(ADC0_BASE, 0x0F); // Clear the interrupt
	// stop everything!
	// Turn around
	// Start over (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) ^ 1)
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2); // Turn on
	DelayMs(200);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00); // Turn off
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
