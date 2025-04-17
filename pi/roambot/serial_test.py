######################################################################
## Instructions
######################################################################

##### SETUP #####
# ls /dev/tty*
# sudo nano /boot/firmware/config.txt
# enable_uart=1
# dtoverlay=disable-bt
# sudo reboot

######################################################################
## Imports
######################################################################

import serial
import time

######################################################################
## Main Program
######################################################################

# Open serial port (adjust tty name and baudrate as needed)
ser = serial.Serial('/dev/ttyS0', 9600)  # or /dev/ttyAMA0 or /dev/ttyUSB0

# Give the connection a second to settle
time.sleep(1)

# Send data
ser.write(b'Hello, world!\n')

# Close the port
ser.close()
