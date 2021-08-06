#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <inttypes.h>
#include "can.h"

// error number
extern volatile uint8_t errcode;

// state
enum canbus_state {ACTIVE, 
				   BUS_PASSIVE_ON, BUS_PASSIVE_OFF, 
				   BUS_OFF_ON, BUS_OFF_OFF};

extern enum canbus_state g_state;

extern void panic(void);

#endif  // GLOBALS_H_
