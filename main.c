#include <stdbool.h>
#include <stdint.h>

#include "driverlib/sysctl.h"

#include "timing.h"
#include "task.h"
#include "subsys.h"
#include "system.h"

#include "motor.h"
#include "reflective_sensors.h"
#include "ir_sensors.h"

version_t SUMO_VERSION;

#define TURN_AROUND_TIME 500
#define SEARCH_TURN_TIME 350

enum state {
	IDLE = 0,
	SEARCH,
	ATTACK,
	TURN_AROUND
} state;

//enum search_state {
//	IDLE = 0;
//	CW,
//	CCW,
//
//
//} search_state;

void Switch2Idle(void);
void PollStartButton(void);

int main(void)
{

	tint_t Search_StartTime = 0;
	tint_t turnTime = 0;
	uint8_t searchDir = 0;

	// Setup the clock to be 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);

	SUMO_VERSION.word = 0x14110800LU;

	ReflectiveInit();
	ir_init();
	TimerInit();
	TaskInit();
	SystemInit();
	MotorsInit();

	// Setup Start Button as Input
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

	SubsystemInit(SUMO, MESSAGE, "SUMO", SUMO_VERSION);

	LogMsg(SUMO, MESSAGE, "System Initialized!");

	state = IDLE;

//	MotorsEnableFront();
//	MotorsEnableBack();

	MotorsDisableFront();
	MotorsDisableBack();

	IntMasterEnable();

	while(1){
		switch (state){
			case IDLE:
				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 0;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = 0;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 0;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 0;
				break;
			case SEARCH:
//				// If true, we haven't started searching yet
//				if(searchDir == CW){
//					Search_StartTime = TimeNow();
//					searchDir++;
//					motors[FRONTLEFT_MOTOR].direction = CW;
//					motors[BACKLEFT_MOTOR].direction = CW;
//					motors[FRONTRIGHT_MOTOR].direction = CW;
//					motors[BACKRIGHT_MOTOR].direction = CW;
//				}
//				// Switch Directions
//				if(TimeSince(Search_StartTime) > SEARCH_TURN_TIME){
//					Search_StartTime = TimeNow();
//					// If search count is greater than 1, go back to idle
//					if(searchDir > CCW){
//						searchDir = CW;
//						state = IDLE;
//						break;
//					}
//					searchDir++;
//					motors[FRONTLEFT_MOTOR].direction ^= 1;
//					motors[BACKLEFT_MOTOR].direction ^= 1;
//					motors[FRONTRIGHT_MOTOR].direction ^= 1;
//					motors[BACKRIGHT_MOTOR].direction ^= 1;
//				}
//				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 200;
//				motors[BACKLEFT_MOTOR].duty_tenths_perc = 200;
//				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 200;
//				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 200;
//				break;
			case ATTACK:
				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 200;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = 200;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 200;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 200;

				motors[FRONTLEFT_MOTOR].direction = CCW;
				motors[BACKLEFT_MOTOR].direction = CCW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;

				break;
			case TURN_AROUND:
//				turnTime = TimeNow();
				motors[FRONTLEFT_MOTOR].direction = CW;
				motors[BACKLEFT_MOTOR].direction = CW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;

				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 200;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = 200;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 200;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 200;

				if(TimeSince(turnTime) > TURN_AROUND_TIME){
					state = IDLE;
				}
				break;
		}

		PollStartButton();
		SystemTick();
//		IR_Update();
		MotorsUpdate();
	}
}


void PollStartButton(void){
	if(!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2)){
		DelayMs(5000);
		state = SEARCH;
	}
}


void Switch2Idle(void){
	state = IDLE;
}
