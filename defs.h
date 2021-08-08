/*
 * Define registers and pins
 * AtMega328p
 * arduino uno board
 * with SK Pang CAN-BUS shield
 */
#ifndef DEFS_H_
#define DEFS_H_

/*-----------------------------------------------------------------------*/

/*
 * Hardware and software revisions
 */
#define HARDWARE_REVISION               1
#define APP_VERSION_MAJOR               0
#define APP_VERSION_MINOR               2

/*-----------------------------------------------------------------------*/

/* uart */
/* size of fifo buffers */
#define TX_FIFO_SIZE                    128
#define RX_FIFO_SIZE                    128

/* define baud rate for serial comm */
#define BAUD                            230400

/*-----------------------------------------------------------------------*/

/* DEBUGGING */

/*#define MCPDEBUG                        1*/
/*#define CANSERIALDEBUG                  1*/
/*#define CANDEBUG                        1*/
/*#define NMEA2000DEBUG                   1*/

/*-----------------------------------------------------------------------*/
// use timestamps in CAN
#define CAN_TIMESTAMP                   1

#define MCP2515FILTER                   1
/* MCP2515 clock */
#define MCP2515_16MHZ                   1

#define NMEA2000_PREFERRED_SOURCE_ADDRESS 5

/* nmea2000 map size */
#define NMEA2000_ADDR_MAP_SZ            10

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

/* joystick pin definitions */
#define DDR_JOYSTICK_CLICK              DDRC
#define PORT_JOYSTICK_CLICK             PORTC
#define PIN_JOYSTICK_CLICK              PINC
#define P_JOYSTICK_CLICK                4

/*-----------------------------------------------------------------------*/

#endif  // DEFS_H_
