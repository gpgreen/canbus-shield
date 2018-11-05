/*-------------------------------------------------------------------------
 * canbus-shield
 *
 * Shield attached to arduino diecimila with atmega168
 * This is to drive the sparkfun canbus-shield sold by sk pang
 * the board also contains a sd card, and joystick
 *
 * outputs messages via USB that are received by the can controller
 * can also be placed in loopback mode to test can software
 *
 * The can controller is an mcp2515, with an mcp2551 transceiver
 *
 * Commands can be sent via serial port. They are:
 *
 * D000100 - turn loopback mode off
 *
 * D000101 - turn loopback mode on
 *
 * t0800400000000 - send an IDS broadcast message
 *
 * -------------------------------------------------------------------------
 */

#include "defs.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "globals.h"
#include "mcp2515.h"
#include "canserial.h"

// set serial port to stdio
static FILE uart_ostr = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE uart_istr = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

// the state
enum canbus_state g_state;

// 2 Hz timer flag
volatile uint8_t g_timer2_set;

// 80 Hz timer flag
volatile uint8_t g_timer80_set;

// global error code
volatile uint8_t errcode;

volatile uint32_t jiffies;

// the can stack initialization struct
can_init_t can_settings;

/* struct for mcp2515 device specific data */
struct mcp2515_dev g_mcp2515_dev = {
	// settings structure
	.settings = &can_settings,
	// interrupt control register, controlling edge detection
	.int_dir_reg = &EICRA,
	// mask for interrupt control register for edge detection, falling edge
	.int_dir_mask = _BV(ISC01),
	// interrupt control register, enable/disable
	.int_en_reg = &EIMSK,
	// mask for interrupt control register
	.int_en_mask = _BV(INT0),
	// GPIO port interrupt is on
	.port = &PORTD,
	// GPIO port direction control register
	.ddr_port = &DDRD,
	// GPIO pin mask
	.port_pin = _BV(2),
};

/*-----------------------------------------------------------------------*/

// handles can errors
static void canbus_can_error(struct can_device* dev, const can_error_t* err)
{
	printf_P(PSTR("E%02x%02x%02x\n"), (uint8_t)err->error_code,
			 (uint8_t)err->dev_buffer,
			 (uint8_t)err->dev_code);
	if (err->error_code == CAN_BUS_OFF)
		g_state = BUS_OFF_ON;
	else if (err->error_code == CAN_BUS_PASSIVE)
		g_state = BUS_PASSIVE_ON;
}

// the can device
struct can_device candev = {
    .priv_dev = &g_mcp2515_dev,
    .devno = 1,
    .handle_error_fn = canbus_can_error,
};

// canserial initialization struct
struct can_serial g_can_serial = {
    .candev = &candev,
	.can_log_recv_message = log_recv,
	.can_log_send_message = log_send,
	.can_log_serial_command = log_canserial_cmd,
	.can_log_failed_serial_command = log_failed_cmd,
	.can_log_device_command = log_dev_cmd,
	.can_log_failed_device_command = log_failed_cmd,
	.can_send_message = can_send_message,
	.can_device_command = can_device_command,
    .can_handle_error = canbus_can_error,
};

/*-----------------------------------------------------------------------*/

// this function is called if CAN doesn't work, otherwise
// use the failed fn below
// blinks at 2Hz to show offline
void
offline(void)
{
	printf("Offline: %d\n", errcode);
	uint8_t on = -1;
	led1_on();
	while (1) {
        // 10 hz timer
        if (g_timer2_set)
        {
            g_timer2_set = 0;
			if (on)
				led1_off();
			else
				led1_on();
			on = on? 0 : -1;
		}
	}
}

/*-----------------------------------------------------------------------*/

/* a fifo to hold received bytes on serial port */
struct fifo 
{
	uint8_t idx_w;
	uint8_t idx_r;
	uint8_t count;
	uint8_t buf[256];
};

static struct fifo s_fifo;

/* how many bytes available */
static uint8_t fifo_count(void)
{
	return s_fifo.count;
}

/* put a byte on the fifo */
static void fifo_put(uint8_t byte)
{
	if(s_fifo.count>=256) {
		s_fifo.count = 0;
		s_fifo.idx_w = s_fifo.idx_r = 0;
	}
	s_fifo.buf[s_fifo.idx_w++] = byte;
	s_fifo.count++;
	if(s_fifo.idx_w>=256)
		s_fifo.idx_w = 0;
}

/* get a byte from the fifo */
static uint8_t fifo_get(void)
{
	uint8_t byte = s_fifo.buf[s_fifo.idx_r++];
	s_fifo.count--;
	if(s_fifo.idx_r>=256)
		s_fifo.idx_r=0;
	return byte;
}

/*-----------------------------------------------------------------------*/

static void
handle_uart(void)
{
	// check for uart error
	uint8_t err = uart_error();
	if(err) {
		printf_P(PSTR("uart error:%x\n"), err);
		return;
	}
	
	// check for input
	uint8_t bytes = uart_test();

	// get input, put it in the fifo
	for(int i=0; i<bytes; ++i)
		fifo_put(uart_getchar(stdin));

	if(!fifo_count())
		return;
	
#ifdef CANSERIALDEBUG
	printf("new bytes:%d count:%d\n", bytes, fifo_count());
#endif
	
	// save the read state of the fifo, so we can adjust for how many
	// bytes are parsed
	uint8_t oldr = s_fifo.idx_r;
	uint8_t oldc = s_fifo.count;

	// parse the bytes in the fifo
	uint8_t consumed = canserial_parse_input_buffer(&g_can_serial, fifo_count, fifo_get);
#ifdef CANSERIALDEBUG
	printf("parsed:%d\n", consumed);
#endif
	// adjust the read state of the fifo for how many bytes actually consumed
	s_fifo.idx_r = oldr + consumed;
	s_fifo.count = oldc - consumed;
	if(s_fifo.idx_r>=256)
		s_fifo.idx_r -= 256;

#ifdef CANSERIALDEBUG
	printf("r:%d w:%d c:%d\n", s_fifo.idx_r, s_fifo.idx_w, s_fifo.count);
#endif
}

/*-----------------------------------------------------------------------*/

void
ioinit(void)
{
	gpio_setup();

	timer_init();
	
    // setup the 80,2 Hz timer
    // WGM12 = 1, CTC, OCR1A
    // CS11 = 1, fosc / 8
    TCCR1B = _BV(CS11);
    // set interrupt to happen at 80Hz
	OCR1A = (F_CPU / 80 / 8);
	// set interrupt to happen at 1000Hz
	OCR1B = (F_CPU / 1000 / 8);
    // set OC interrupt 1A and 1B
    TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);
	// setup the serial hardware
	uart_init();

	puts_P(PSTR("Canbus Shield"));
	printf_P(PSTR("Hardware: %d Software: %d\n-------------------------\n"),
			 HARDWARE_REVISION, SOFTWARE_REVISION);
	
    // spi needs to be setup first
	if (spi_init(2) == SPI_FAILED)
		offline();

	puts_P(PSTR("spi initialized."));

	// setup the can initialization struct
	can_settings.speed_setting = CAN_250KBPS;
	can_settings.loopback_on = -1;      /* loopback on */
	can_settings.tx_wait_ms = 20;

	errcode = can_init(&can_settings, &candev);
	if (errcode != CAN_OK)
		offline();

	puts_P(PSTR("can initialized."));
	
	// self test the CAN stack
	errcode = can_self_test(&candev);
	if (errcode != CAN_OK)
		offline();

	puts_P(PSTR("can self-test complete.\nioinit complete."));
}

/*-----------------------------------------------------------------------*/

int
main(void)
{
	g_state = ACTIVE;
	
	// stdout is the uart
	stdout = &uart_ostr;
	// stdin is the uart
	stdin = &uart_istr;
	// stderr is the uart
	stderr = &uart_ostr;
	
	ioinit();
	sei();

	puts_P(PSTR("Starting main loop.\n"));

    while(1)
    {
		// 80 Hz timer
		if (g_timer80_set)
		{
			g_timer80_set = 0;
		}

        // 2 Hz timer
        if (g_timer2_set)
        {
			static int count = 0;
            g_timer2_set = 0;
			if (g_state == ACTIVE && ++count >= 10) {
				uint8_t tx,rx;
				can_error_counts(&candev, &tx, &rx);
                // print the error counts if either are not 0
                if(tx !=0 || rx != 0)
                    printf_P(PSTR("can errors tx:%u rx:%u\n"), tx, rx);
				count = 0;
			}
        }
		
		// clear the interrupt flags on the device
		int status;
		if(can_handle_interrupt(&candev, &status) == CAN_INTERRUPT) {

			can_msg_t incoming;
			uint8_t status = CAN_OK;
			for (int i=0; i<4 && status == CAN_OK; ++i) {
				status = can_read_message(&candev, &incoming);
				if (status == CAN_OK)
					// do something with any incoming msgs
					canserial_handle_recv_message(&g_can_serial, &incoming);
			}
        }

		// check for input
		handle_uart();
    }
    return 0;
}

//-----------------------------------------------------------------------
//
// Timer compare output 1A interrupt
//
//-----------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
    g_timer80_set = 1;
	static uint8_t count = 0;
	if (++count >= 40) {
		g_timer2_set = 1;
		count = 0;
		if (g_state < BUS_PASSIVE_ON)
			return;
		// error state
		if (g_state == BUS_OFF_ON) {
			led1_on();
			led2_on();
			g_state = BUS_OFF_OFF;
		} else if (g_state == BUS_OFF_OFF) {
			led1_off();
			led2_off();
			g_state = BUS_OFF_ON;
		}
		else if (g_state == BUS_PASSIVE_ON) {
			led1_on();
			g_state = BUS_PASSIVE_OFF;
		} else if (g_state == BUS_PASSIVE_OFF) {
			led1_off();
			g_state = BUS_PASSIVE_ON;
		}
	}
}

//-----------------------------------------------------------------------
//
// Timer compare output 1A interrupt
//
//-----------------------------------------------------------------------
ISR(TIMER1_COMPB_vect)
{
	jiffies++;
}

/*-----------------------------------------------------------------------*/

