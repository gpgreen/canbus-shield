/*
 * Define registers and pins
 * AtMega168
 * arduino diecimila board
 * with SK Pang CAN-BUS shield
 */
#ifndef DEFS_H_
#define DEFS_H_

/*-----------------------------------------------------------------------*/

/*
 * Hardware and software revisions
 */
#define HARDWARE_REVISION 1
#define SOFTWARE_REVISION 1

/*-----------------------------------------------------------------------*/

/* functions available */
#define HAVE_UART_DEVICE                1

/* size of fifo buffers */
#define FIFO_SIZE                       64

/* define baud rate for serial comm */
#define BAUD                            115200

/*-----------------------------------------------------------------------*/

/* DEBUGGING */

/*#define MCPDEBUG (1)*/
/*#define CANSERIALDEBUG (1)*/
/*#define CANDEBUG (1)*/

/*-----------------------------------------------------------------------*/
#define MCP2515FILTER                   (1)
/* MCP clock */
#define MCP2515_16MHZ                   (1)

/* FIFO lengths */
#define MCP_RECVBUFLEN                  10
#define MCP_ERRBUFLEN                   20

/* MCP interrupt vector */
#define MCP2515_INT_VECT                INT0_vect

/* CS pin tied to arduino 10 (PB2)*/
#define DDR_CANCS                       DDRB
#define PORT_CANCS                      PORTB
#define P_CANCS                         2

/* SPI pins tied to arduino 11(MOSI), 12(MISO), 13(SCK)*/
#define DDR_SPI                         DDRB
#define PORT_SPI                        PORTB
#define P_MOSI                          3
#define P_MISO                          4
#define P_SCK                           5

/*-----------------------------------------------------------------------*/

#endif  // DEFS_H_
