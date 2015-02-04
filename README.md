# create
A library for running an irobot create from a raspberry pi.

# To find out what port your usb to serial cable converter is on
dmesg | grep tty
# To get port by device id.
/dev/serial/by-id/usb-9710_7715-if00-port0
# Set baud rate for a serial port.
stty -F /dev/ttyUSB0 57600
# View current settings for serial port.
stty -F /dev/ttyUSB0 -a

# To dump bytes in human readable format
od -t u1
