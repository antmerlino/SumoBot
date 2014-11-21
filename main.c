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
#include "sumo.h"
#include "servo.h"

version_t SUMO_VERSION;

#define TURN_AROUND_TIME 350
#define REVERSE_TIME 275
#define SEARCH_TURN_TIME 350
#define TURN_TIMEOUT 300
#define IR_TIMEOUT 5

#define SPEED 600
#define TURN_SPEED 400
#define ATTACK_SPEED 750

const char const * state2String[]={"Idle", "Search", "Forward" ,"Attack", "Reverse", "Turn Around", "Turn Left", "Turn Right", "Front Left", "Front Right", "Reverse Left", "Reverse Right"};

sumo_state_t state;
sumo_state_t prev_state;

tint_t moveTimer = 0;

void Switch2Idle(void);
void PollStartButton(void);

tint_t Search_StartTime = 0;
tint_t Scan_StartTime = 0;

int main(void)
{
	int16_t frontDiff;

	// Setup the clock to be 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);

	SUMO_VERSION.word = 0x14110800LU;

	TimerInit();
	TaskInit();
	SystemInit();
	MotorsInit();
	ServoInit();
	ReflectiveInit();
	IR_Init();

	// Setup Start Button as Input
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

	SubsystemInit(SUMO, MESSAGE, "SUMO", SUMO_VERSION);

	LogMsg(SUMO, MESSAGE, "System Initialized!");

	LogMsg(SUMO, MESSAGE, "Reset Register: %d", SysCtlResetCauseGet());
	SysCtlResetCauseClear(0xFFFF);

	SumoSetState(IDLE);

	TaskScheduleAdd(IR_Update, TASK_MEDIUM_PRIORITY, 2000, 150);

	MotorsEnableFront();
	MotorsEnableBack();

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
			// If true, we haven't started searching yet
			//			if(searchDir == CW){
			//				Search_StartTime = TimeNow();
			//				searchDir++;
			//				motors[FRONTLEFT_MOTOR].direction = CW;
			//				motors[BACKLEFT_MOTOR].direction = CW;
			//				motors[FRONTRIGHT_MOTOR].direction = CW;
			//				motors[BACKRIGHT_MOTOR].direction = CW;
			//			}
			//			// Switch Directions
			//			if(TimeSince(Search_StartTime) > SEARCH_TURN_TIME){
			//				Search_StartTime = TimeNow();
			//				// If search count is greater than 1, go back to idle
			//				if(searchDir > CCW){
			//					searchDir = CW;
			//					state = IDLE
			//					break;
			//				}
			//				searchDir++;
			//				motors[FRONTLEFT_MOTOR].direction ^= 1;
			//				motors[BACKLEFT_MOTOR].direction ^= 1;
			//				motors[FRONTRIGHT_MOTOR].direction ^= 1;
			//				motors[BACKRIGHT_MOTOR].direction ^= 1;
			//			}
			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
			break;
		case FORWARD:
			motors[FRONTLEFT_MOTOR].direction = CCW;
			motors[BACKLEFT_MOTOR].direction = CCW;
			motors[FRONTRIGHT_MOTOR].direction = CW;
			motors[BACKRIGHT_MOTOR].direction = CW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
			break;
		case ATTACK:
			motors[FRONTLEFT_MOTOR].direction = CCW;
			motors[BACKLEFT_MOTOR].direction = CCW;
			motors[FRONTRIGHT_MOTOR].direction = CW;
			motors[BACKRIGHT_MOTOR].direction = CW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = ATTACK_SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = ATTACK_SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = ATTACK_SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = ATTACK_SPEED;
			break;
		case REVERSE:
			motors[FRONTLEFT_MOTOR].direction = CW;
			motors[BACKLEFT_MOTOR].direction = CW;
			motors[FRONTRIGHT_MOTOR].direction = CCW;
			motors[BACKRIGHT_MOTOR].direction = CCW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = 1.25*SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = 1.25*SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 1.25*SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = 1.25*SPEED;

			if(TimeSince(moveTimer) > REVERSE_TIME){
				SumoSetState(TURN_AROUND);
			}
			break;
		case TURN_AROUND:

			switch(prev_state){

			case REVERSE_RIGHT:
				motors[FRONTLEFT_MOTOR].direction = CW;
				motors[BACKLEFT_MOTOR].direction = CW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;
				break;
			case REVERSE_LEFT:
				motors[FRONTLEFT_MOTOR].direction = CCW;
				motors[BACKLEFT_MOTOR].direction = CCW;
				motors[FRONTRIGHT_MOTOR].direction = CCW;
				motors[BACKRIGHT_MOTOR].direction = CCW;
				break;
			default:
				motors[FRONTLEFT_MOTOR].direction = CW;
				motors[BACKLEFT_MOTOR].direction = CW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;
				break;
			}

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;

			if(TimeSince(moveTimer) > TURN_AROUND_TIME){
				SumoSetState(FORWARD);
			}
			break;

//			switch(prev_state){
//				case REVERSE_RIGHT:
//					motors[FRONTLEFT_MOTOR].direction = CW;
//					motors[BACKLEFT_MOTOR].direction = CW;
//					motors[FRONTRIGHT_MOTOR].direction = CCW;
//					motors[BACKRIGHT_MOTOR].direction = CCW;
//
//					motors[FRONTLEFT_MOTOR].duty_tenths_perc = 1.5*SPEED;
//					motors[BACKLEFT_MOTOR].duty_tenths_perc = 1.5*SPEED;
//					motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 0.5*SPEED;
//					motors[BACKRIGHT_MOTOR].duty_tenths_perc = 0.5*SPEED;
//					//what is the point of this if it is the same as TURN_LEFT?
//					if(TimeSince(moveTimer) > TURN_AROUND_TIME){
//						SumoSetState(FORWARD);
//					}
//					break;
//				default:
//					motors[FRONTLEFT_MOTOR].direction = CW;
//					motors[BACKLEFT_MOTOR].direction = CW;
//					motors[FRONTRIGHT_MOTOR].direction = CCW;
//					motors[BACKRIGHT_MOTOR].direction = CCW;
//
//					motors[FRONTLEFT_MOTOR].duty_tenths_perc = 0.5*SPEED;
//					motors[BACKLEFT_MOTOR].duty_tenths_perc = 0.5*SPEED;
//					motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 1.5*SPEED;
//					motors[BACKRIGHT_MOTOR].duty_tenths_perc = 1.5*SPEED;
//					if(TimeSince(moveTimer) > TURN_AROUND_TIME){
//						SumoSetState(FORWARD);
//					}
//					break;
//				}
			case TURN_LEFT:
				motors[FRONTLEFT_MOTOR].direction = CW;
				motors[BACKLEFT_MOTOR].direction = CW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;

				motors[FRONTLEFT_MOTOR].duty_tenths_perc = TURN_SPEED;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = TURN_SPEED;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = TURN_SPEED;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = TURN_SPEED;

				if(TimeSince(moveTimer) > TURN_TIMEOUT){
					SumoSetState(FORWARD);
				}
				break;
			case TURN_RIGHT:
				motors[FRONTLEFT_MOTOR].direction = CCW;
				motors[BACKLEFT_MOTOR].direction = CCW;
				motors[FRONTRIGHT_MOTOR].direction = CCW;
				motors[BACKRIGHT_MOTOR].direction = CCW;

				motors[FRONTLEFT_MOTOR].duty_tenths_perc = TURN_SPEED;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = TURN_SPEED;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = TURN_SPEED;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = TURN_SPEED;
				if(TimeSince(moveTimer) > TURN_TIMEOUT){
					SumoSetState(FORWARD);
				}
				break;
			case FRONT_LEFT:
				motors[FRONTLEFT_MOTOR].direction = CCW;
				motors[BACKLEFT_MOTOR].direction = CCW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;

				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 0.75*SPEED;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = 0.75*SPEED;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 1.25*SPEED;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 1.25*SPEED;
				break;
			case FRONT_RIGHT:
				motors[FRONTLEFT_MOTOR].direction = CCW;
				motors[BACKLEFT_MOTOR].direction = CCW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;

				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 1.25*SPEED;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = 1.25*SPEED;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 0.75*SPEED;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 0.75*SPEED;
				break;
			case REVERSE_LEFT:
				motors[FRONTLEFT_MOTOR].direction = CW;
				motors[BACKLEFT_MOTOR].direction = CW;
				motors[FRONTRIGHT_MOTOR].direction = CCW;
				motors[BACKRIGHT_MOTOR].direction = CCW;

				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 0.5*SPEED;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = 0.5*SPEED;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 1.5*SPEED;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 1.5*SPEED;
				if(TimeSince(moveTimer) > REVERSE_TIME){
					SumoSetState(TURN_AROUND);
				}
				break;
			case REVERSE_RIGHT:
				motors[FRONTLEFT_MOTOR].direction = CW;
				motors[BACKLEFT_MOTOR].direction = CW;
				motors[FRONTRIGHT_MOTOR].direction = CCW;
				motors[BACKRIGHT_MOTOR].direction = CCW;

				motors[FRONTLEFT_MOTOR].duty_tenths_perc = 1.5*SPEED;
				motors[BACKLEFT_MOTOR].duty_tenths_perc = 1.5*SPEED;
				motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 0.5*SPEED;
				motors[BACKRIGHT_MOTOR].duty_tenths_perc = 0.5*SPEED;

				if(TimeSince(moveTimer) > REVERSE_TIME){
					SumoSetState(TURN_AROUND);
				}
				break;
		}// switch
		PollStartButton();
		SystemTick();
		MotorsUpdate();
	}
}


void PollStartButton(void){
	static tint_t lastBtnPress = 0;
	if(!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) && TimeSince(lastBtnPress) > 2000){
		lastBtnPress = TimeNow();
		if(SumoGetState() == IDLE){
			LogMsg(SUMO, MESSAGE, "Start Button Pressed.");
			DelayMs(5000);
			ServoSetPosition(26);
			SumoSetState(FORWARD);
			Scan_StartTime = TimeNow();
		} else {
			SumoSetState(IDLE);
			ServoSetPosition(170);
		}
	}
}

void SumoSetState(sumo_state_t newState){
	if(state != newState){
		prev_state = state;
		state = newState;
		LogMsg(SUMO, MESSAGE, "State Updated: %s", state2String[state]);

		motors[FRONTLEFT_MOTOR].direction = BRAKE;
		motors[BACKLEFT_MOTOR].direction = BRAKE;
		motors[FRONTRIGHT_MOTOR].direction = BRAKE;
		motors[BACKRIGHT_MOTOR].direction = BRAKE;
		MotorsUpdate();
		moveTimer = TimeNow();
	}
}

sumo_state_t SumoGetState(void){
	return state;
}
