#ifndef GPIO_H_
#define GPIO_H_

#include <avr/io.h>
#include "defs.h"
#include "debounce.h"

extern void gpio_setup(void);

extern void led1_on(void);
extern void led1_off(void);
extern void led1_toggle(void);
extern void led2_on(void);
extern void led2_off(void);
extern void led2_toggle(void);
extern int is_led2_on(void);

extern debounce_pin_t joystick_click;

#endif	// GPIO_H_
