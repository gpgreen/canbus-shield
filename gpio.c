#include <stdint.h>
#include <avr/io.h>

#include "gpio.h"

void gpio_setup(void)
{
	// setup the two leds
	DDRD |= _BV(PD7);
	DDRB |= _BV(PB0);
	
	// MCP2515 hardware

	// CAN_CS, output
	DDR_CANCS |= _BV(P_CANCS);
	PORT_CANCS |= _BV(P_CANCS);

}

/*-----------------------------------------------------------------------*/

void
led1_on(void)
{
	PORTB |= _BV(PB0);
}
void
led1_off(void)
{
	PORTB &= ~_BV(PB0);
}
void
led1_toggle(void)
{
	PORTB ^= _BV(PB0);
}

/*-----------------------------------------------------------------------*/

void
led2_on(void)
{
	PORTD |= _BV(PD7);
}

void
led2_off(void)
{
	PORTD &= ~_BV(PD7);
}
void
led2_toggle(void)
{
	PORTD ^= _BV(PD7);
}

