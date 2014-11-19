/*
 * system.h
 *
 *  Created on: Mar 26, 2014
 *      Author: Anthony
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

#define PERIPHERAL_CLOCK SysCtlClockGet()
#define SYSTEM_CLOCK SysCtlClockGet()

// subsytem enumeration needed for the subsys module
enum sys_index {
    SYSTEM = 0,
    // ADD SUBSYSTEMS BELOW //
    SUMO,
    REFLECT,
    IR,
    MOTOR,
    // ADD SUBSYSTEMS ABOVE //
    UNKNOWN
};

#define SUBSYS_USE_UART4
#define USE_UART4
#define UART4_BAUD 115200

#define LOG_BUF &tx4

#endif /* SYSTEM_H_ */
