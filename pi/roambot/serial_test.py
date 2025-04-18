######################################################################
## Instructions
######################################################################

##### SETUP #####
# sudo apt update
# sudo apt install python3-venv -y
# cd ~/roambot
# python3 -m venv myenv
# source ~/roambot/myenv/bin/activate
# pip install pyserial

##### TO RUN #####
# sudo ~/roambot/myenv/bin/python3 ~/roambot/serial_test.py

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
