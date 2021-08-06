/*-------------------------------------------------------------------------
 * canbus-shield
 *
 * Shield attached to arduino uno with atmega328p
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
 * -------------------------------------------------------------------------
 */

#include "defs.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "globals.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "timer1.h"
#include "mcp2515.h"
#include "fifo.h"
#include "canserial.h"

// io
static FILE uartstream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar,
                                           _FDEV_SETUP_RW);

// uart fifo buffers
uint8_t tx_fifo_buffer[TX_FIFO_SIZE];
uint8_t rx_fifo_buffer[RX_FIFO_SIZE];

// the state
enum canbus_state g_state;

// 2 Hz timer flag
volatile uint8_t g_timer2hz_set;

// 80 Hz timer flag
volatile uint8_t g_timer80hz_set;

/*-----------------------------------------------------------------------*/

void timer1_compareA(void)
{
    g_timer80hz_set = 1;
    
    static int count = 0;
    if (++count == 40)
    {
        g_timer2hz_set = 1;
        count = 0;
        static int on = 0;
        if (on)
        {
            led1_off();
            on = 0;
        } else {
            led1_on();
            on = 1;
        }
    }
}

static timer1_init_t timer1_settings = {
    .scale = CLK8,
    // compare A triggers at 80Hz
    .compareA_cb = timer1_compareA,
    .compareA_val = (F_CPU / 80 / 8),
    .compareB_cb = 0,
    .compareB_val = 0,
};

/*-----------------------------------------------------------------------*/

// the can stack initialization struct
can_init_t can_settings = {
    .speed_setting = CAN_250KBPS,
    .loopback_on = 0,
    .tx_wait_ms = 20,
};

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
    .init_fn = mcp2515_init,
    .reinit_fn = mcp2515_reinit,
    .self_test_fn = mcp2515_self_test,
    .check_receive_fn = mcp2515_check_receive,
    .free_send_buffer_fn = mcp2515_get_next_free_tx_buf,
    .write_msg_fn = mcp2515_write_msg,
    .read_msg_fn = mcp2515_read_msg,
    .device_command = 0,
    .handle_int_fn = mcp2515_handle_interrupt,
    .handle_error_fn = canbus_can_error,
    .error_counts = mcp2515_error_counts,
    .clear_tx_buffers = mcp2515_clear_tx_buffers,
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

// panic the firmware
// blinks 4 times at 10 hz then a long pause then repeat
// the led is normally part of SPI (SCK)
void
panic(void)
{
    uint8_t count = 0;
    uint8_t major_count = 0;
    uint8_t minor_count = 0;
    // turn off spi, so led will work
    SPCR = 0;
    DDR_SPI |= _BV(P_SCK);
    led1_on();
	while (1) {
		if (g_timer2hz_set)
		{
            if (++count == 4) {
                count = 0;
                if (major_count == 0) {
                    led1_toggle();
                    if (++minor_count == 7)
                        major_count++;
                } else if (major_count++ == 4) {
                    major_count = 0;
                    minor_count = 0;
                    led1_on();
                }
            }
            g_timer2hz_set = 0;
		}
	}
}

/*-----------------------------------------------------------------------*/

/* fifo for parsing input buffer */
#define FIFO_BUFSIZE                    255
uint8_t fifo_buf[FIFO_BUFSIZE];
static struct fifo s_fifo;

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
    {
        if (fifo_count(&s_fifo) == FIFO_BUFSIZE)
            break;
		fifo_put_unsafe(&s_fifo, uart_getchar(stdin));
    }

	if(!fifo_count(&s_fifo))
		return;
	
#ifdef CANSERIALDEBUG
	printf("new bytes:%d count:%d\n", bytes, fifo_count(&s_fifo));
#endif
	
	// save the read state of the fifo, so we can adjust for how many
	// bytes are parsed
	uint8_t oldr = s_fifo.idx_r;
	uint8_t oldc = s_fifo.count;

	// parse the bytes in the fifo
	uint8_t consumed = canserial_parse_input_buffer(&g_can_serial,
                                                    &s_fifo);
#ifdef CANSERIALDEBUG
	printf("parsed:%d\n", consumed);
#endif
	// adjust the read state of the fifo for how many bytes actually consumed
	s_fifo.idx_r = oldr + consumed;
	s_fifo.count = oldc - consumed;
	if (s_fifo.idx_r >= FIFO_BUFSIZE)
		s_fifo.idx_r -= FIFO_BUFSIZE;

#ifdef CANSERIALDEBUG
	printf("r:%d w:%d c:%d\n", s_fifo.idx_r, s_fifo.idx_w, s_fifo.count);
#endif
}

/*-----------------------------------------------------------------------*/

void
ioinit(void)
{
    led1_on();
    
	gpio_setup();

	timer_init();

    timer1_init(&timer1_settings);
    
    // initialize the parsing fifo
    fifo_init(&s_fifo, FIFO_BUFSIZE, fifo_buf);
    
	// setup the serial hardware
	uart_init(TX_FIFO_SIZE, tx_fifo_buffer,
              RX_FIFO_SIZE, rx_fifo_buffer);
    sei();

	printf_P(PSTR("\nCanbus Shield\n"));
	printf_P(PSTR("Hardware: %d Software: %d.%d\n-------------------------\n"),
             HARDWARE_REVISION, APP_VERSION_MAJOR, APP_VERSION_MINOR);

    
    // spi needs to be setup first
	if (spi_init(2) == SPI_FAILED)
		panic();

	printf_P(PSTR("spi initialized\n"));

	// setup can
	int errcode = can_init(&can_settings, &candev);
	if (errcode != CAN_OK)
        panic();

	printf_P(PSTR("can initialized\n"));
	
	// self test the CAN stack
	errcode = can_self_test(&candev);
	if (errcode != CAN_OK)
		panic();

	printf_P(PSTR("can self-test complete.\nioinit complete\n"));
    led1_off();
}

/*-----------------------------------------------------------------------*/

int
main(void)
{
	g_state = ACTIVE;
	
    stdout = &uartstream;
    stderr = &uartstream;
    stdin = &uartstream;

	ioinit();

	printf_P(PSTR("Starting main loop\n"));

    while(1)
    {
		// 80 Hz timer
		if (g_timer80hz_set)
		{
			g_timer80hz_set = 0;
		}

        // 2 Hz timer
        if (g_timer2hz_set)
        {
			static int count = 0;
            g_timer2hz_set = 0;
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
