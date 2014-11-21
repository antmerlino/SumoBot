/*
 * sumoh.h
 *
 *  Created on: Nov 15, 2014
 *      Author: Anthony
 */

#ifndef SUMO_H_
#define SUMO_H_

typedef enum sumo_state_t {
	IDLE = 0,
	SEARCH,
	FORWARD,
	ATTACK,
	REVERSE,
	TURN_AROUND,
	TURN_LEFT,
	TURN_RIGHT,
	FRONT_LEFT,
	FRONT_RIGHT,
	REVERSE_LEFT,
	REVERSE_RIGHT
} sumo_state_t;


void SumoSetState(sumo_state_t newState);
sumo_state_t SumoGetState(void);



#endif /* SUMO_H_ */
