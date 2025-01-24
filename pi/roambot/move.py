# SaberTooth Controller 2x25 v2.0 (Mode: 011100) [0 is up, 1 is down]
# Switches [1:2] = Mode (01: Simple Serial)
# Switches [3] = Lithium Cutoff (1: Enabled)
# Switches [4:5] = Baudrate (10: 9600)
# Switches [6] = Slave Mode (0: Disabled)
# 5V TTL Logic

import serial
import time

# All Stop = 0
# Full Left Forward = 1
# Full Left Stop    = 64
# Full Left Reverse = 127


# Full Right Forward = 128
# Full Right Stop    = 192 
# Full Right Reverse = 255

def send(port, val):
    time.sleep(1)
    port.write(bytes([val]))
    print(f"Sent: {val}")
    port.flush()

def main():
    # Set up the serial port
    port = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

    # send(port, 0)
    # send(port, 1)
    # send(port, 5)
    # send(port, 10)
    # send(port, 15)
    # send(port, 20)
    # send(port, 30)
    # send(port, 40)
    # send(port, 50)
    # send(port, 60)
    # send(port, 64)
    # send(port, 70)
    # send(port, 80)
    # send(port, 90)
    # send(port, 100)
    # send(port, 110)
    # send(port, 127)
    # send(port, 0)


    send(port, 0)
    send(port, 110)
    send(port, 112)
    send(port, 114)
    send(port, 116)
    send(port, 118)
    send(port, 120)
    send(port, 122)
    send(port, 124)
    send(port, 127)
    send(port, 0)

    port.close()

main()
