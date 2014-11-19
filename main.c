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

version_t SUMO_VERSION;

#define TURN_AROUND_TIME 300
#define REVERSE_TIME 250
#define SEARCH_TURN_TIME 350
#define IR_TIMEOUT 50

#define SPEED 400

const char const * state2String[]={"Idle", "Search", "Attack", "Reverse", "Turn Around", "Turn Left", "Turn Right", "Front Right", "Front Left", "Reverse Left", "Reverse Right"};

sumo_state_t state;

tint_t moveTimer = 0;

void Switch2Idle(void);
void PollStartButton(void);

ir_shortrange_data_t ir_shortrange_data;
ir_longrange_data_t ir_longrange_data;
tint_t Search_StartTime = 0;

int main(void)
{
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
	LogMsg(SUMO, MESSAGE, "Reset Register: %d", SysCtlResetCauseGet());
	SumoSetState(IDLE);
	SysCtlResetCauseClear(0xFFFF);

	MotorsEnableFront();
	MotorsEnableBack();

	//	MotorsDisableFront();
	//	MotorsDisableBack();

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
			if(searchDir == CW){
//					Search_StartTime = TimeNow();
//					searchDir++;
				motors[FRONTLEFT_MOTOR].direction = CW;
				motors[BACKLEFT_MOTOR].direction = CW;
				motors[FRONTRIGHT_MOTOR].direction = CW;
				motors[BACKRIGHT_MOTOR].direction = CW;
			}
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
			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
			break;
		case ATTACK:
			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;

			motors[FRONTLEFT_MOTOR].direction = CCW;
			motors[BACKLEFT_MOTOR].direction = CCW;
			motors[FRONTRIGHT_MOTOR].direction = CW;
			motors[BACKRIGHT_MOTOR].direction = CW;

			break;
		case REVERSE:
			motors[FRONTLEFT_MOTOR].direction = CW;
			motors[BACKLEFT_MOTOR].direction = CW;
			motors[FRONTRIGHT_MOTOR].direction = CCW;
			motors[BACKRIGHT_MOTOR].direction = CCW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;

			if(TimeSince(moveTimer) > REVERSE_TIME){
				SumoSetState(TURN_AROUND);
			}

			break;
		case TURN_AROUND:
			motors[FRONTLEFT_MOTOR].direction = CW;
			motors[BACKLEFT_MOTOR].direction = CW;
			motors[FRONTRIGHT_MOTOR].direction = CW;
			motors[BACKRIGHT_MOTOR].direction = CW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
//what is the point of this if it is the same as TURN_LEFT?
			if(TimeSince(moveTimer) > TURN_AROUND_TIME){
				SumoSetState(ATTACK);
			}
			break;
		case TURN_LEFT:
			motors[FRONTLEFT_MOTOR].direction = CW;
			motors[BACKLEFT_MOTOR].direction = CW;
			motors[FRONTRIGHT_MOTOR].direction = CW;
			motors[BACKRIGHT_MOTOR].direction = CW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
			break;
		case TURN_RIGHT:
			motors[FRONTLEFT_MOTOR].direction = CCW;
			motors[BACKLEFT_MOTOR].direction = CCW;
			motors[FRONTRIGHT_MOTOR].direction = CCW;
			motors[BACKRIGHT_MOTOR].direction = CCW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
			break;
		case FRONT_LEFT:
			motors[FRONTLEFT_MOTOR].direction = CCW;
			motors[BACKLEFT_MOTOR].direction = CCW;
			motors[FRONTRIGHT_MOTOR].direction = CW;
			motors[BACKRIGHT_MOTOR].direction = CW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
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
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
			break;
		case REVERSE_LEFT:
			motors[FRONTLEFT_MOTOR].direction = CW;
			motors[BACKLEFT_MOTOR].direction = CW;
			motors[FRONTRIGHT_MOTOR].direction = CCW;
			motors[BACKRIGHT_MOTOR].direction = CCW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = 1.25*SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = 1.25*SPEED;
			break;
		case REVERSE_RIGHT:
			motors[FRONTLEFT_MOTOR].direction = CW;
			motors[BACKLEFT_MOTOR].direction = CW;
			motors[FRONTRIGHT_MOTOR].direction = CCW;
			motors[BACKRIGHT_MOTOR].direction = CCW;

			motors[FRONTLEFT_MOTOR].duty_tenths_perc = 1.25*SPEED;
			motors[BACKLEFT_MOTOR].duty_tenths_perc = 1.25*SPEED;
			motors[FRONTRIGHT_MOTOR].duty_tenths_perc = SPEED;
			motors[BACKRIGHT_MOTOR].duty_tenths_perc = SPEED;
			break;
		}// switch
		PollStartButton();
		SystemTick();
		MotorsUpdate();
		if(state != REVERSE && state != REVERSE_LEFT && state != REVERSE_RIGHT && state != TURN_AROUND && state != IDLE){
			if(TimeSince(Search_StartTime) > IR_TIMEOUT){
				Search_StartTime = TimeNow();
				update_ir(&ir_longrange_data);
//					ir_poll_long(&ir_longrange_data);
			}
		}
	}// while
}


void PollStartButton(void){
	if(!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) && (SumoGetState() == IDLE)){
		DelayMs(5000);
		SumoSetState(ATTACK);
		Search_StartTime = TimeNow();
	}
}

void SumoSetState(sumo_state_t newState){
	if(state != newState){
		state = newState;
		LogMsg(SUMO, MESSAGE, "State Updated: %s", state2String[state]);

		switch (state){
		case TURN_AROUND:
			motors[FRONTLEFT_MOTOR].direction = BRAKE;
			motors[BACKLEFT_MOTOR].direction = BRAKE;
			motors[FRONTRIGHT_MOTOR].direction = BRAKE;
			motors[BACKRIGHT_MOTOR].direction = BRAKE;
			MotorsUpdate();
			moveTimer = TimeNow();
			break;
		case REVERSE:
			motors[FRONTLEFT_MOTOR].direction = BRAKE;
			motors[BACKLEFT_MOTOR].direction = BRAKE;
			motors[FRONTRIGHT_MOTOR].direction = BRAKE;
			motors[BACKRIGHT_MOTOR].direction = BRAKE;
			MotorsUpdate();
			moveTimer = TimeNow();
			break;
		}
	}
}

sumo_state_t SumoGetState(void){
	return state;
}
