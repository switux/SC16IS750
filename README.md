# SC16IS750
This is a barebone library to use the SC16IS750 UART bridge.

It supports:
  - Basic serial communication ( I tested it with a NEO-8M GPS module )
  - GPIO pins ( read, write, interrupts )

By default, aside the baud rate that must be given to begin(), it sets a 8N1 line policy.

I read that most (if not all) modules lack a wire between in nIRQ pin of the board and that of the chip (mine lacks the pcb track). You must solder a wire between the two, this is easily done by using the end of the 1kOhm pull-up resistor that faces the chip if you are not willing to attach the wire directly to the chip's pin.

