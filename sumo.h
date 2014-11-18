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
	ATTACK,
	REVERSE,
	TURN_AROUND,
	TURN_LEFT,
	TURN_RIGHT
} sumo_state_t;


void SumoSetState(sumo_state_t newState);
sumo_state_t SumoGetState(void);



#endif /* SUMO_H_ */
