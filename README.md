# canbus-shield
Firmware project for Arduino + CAN bus shield. The CAN bus shield is
from sk pang. It has a command language, and sends CAN messages
to/from the bus via USB serial port.

## Commands

Turn loopback on:
```
D000101
```
Turn loopback off:
```
D000100
```

Set baudrate (baudrate is 'bb')
```
D0101bb
```

Set timeout (mm-milliseconds hi byte nn-miliseconds low byte)
```
D0202mmnn
```

## Messages

the messages are in slcan linux serial can driver format

<type> <id> <dlc> <data>*

Extended frames (29 bit) are defined by capital characters in the type.
RTR frames are defined as 'r' types - normal frames have 't' type:
t => 11 bit data frame
r => 11 bit RTR frame
T => 29 bit data frame
R => 29 bit RTR frame

The <id> is 3 (standard) or 8 (extended) bytes in ASCII Hex (base64).
The <dlc> is a one byte ASCII number ('0' - '8')
The <data> section has up to 8 ASCII Hex bytes as defined by the <dlc>

Examples:

```
t1230 : can_id 0x123, can_dlc 0, no data
```
```
t4563112233 : can_id 0x456, can_dlc 3, data 0x11 0x22 0x33
```
```
T12ABCDEF2AA55 : extended can_id 0x12ABCDEF, can_dlc 2, data 0xAA 0x55
```
```
r1230 : can_id 0x123, can_dlc 0, no data, remote transmission request
```

## Errors

first byte is error code
second byte device buffer
third byte device code

CAN error warning
```
E090000
```

CAN bus passive
```
E010001
```
