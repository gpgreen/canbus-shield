#ifndef GPIO_H_
#define GPIO_H_

#include "defs.h"

extern void gpio_setup(void);

extern void led1_on(void);
extern void led1_off(void);
extern void led1_toggle(void);
extern void led2_on(void);
extern void led2_off(void);
extern void led2_toggle(void);
extern int is_led2_on(void);

#endif	// GPIO_H_
