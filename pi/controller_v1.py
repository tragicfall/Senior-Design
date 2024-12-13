# SaberTooth Controller 2x25 v2.0 (Mode: 011101) [0 is up, 1 is down]
# Switches [1:2] = Mode (01: Simple Serial)
# Switches [3] = Lithium Cutoff (1: Enabled)
# Switches [4:5] = Baudrate (10: 9600)
# Switches [6] = Slave Mode (1: Enabled)
# 5V TTL Logic

import curses
import threading
import time
import serial
import RPi.GPIO as GPIO

# Function to send power levels to the serial port
def send_to_serial(port, power_levels, status):
    while True:
        if status[0]:
            # Mask the power levels if the all_wheel_status is OFF
            command_arr = [0, 0, 0, 0]
            command_arr[0] = power_levels[0] & status[1]
            command_arr[1] = power_levels[1] & status[1]
            command_arr[2] = power_levels[2] & status[1]
            command_arr[3] = power_levels[3] & status[1]

            # Set the GPIO pin 2 to HIGH to enable the front (left and right) motors
            time.sleep(0.001)
            GPIO.output(3, GPIO.LOW)
            GPIO.output(2, GPIO.HIGH)
            time.sleep(0.001)

            # Send the power levels to the serial port
            port.write(bytes([command_arr[0]]))
            port.flush()
            port.write(bytes([command_arr[1]]))
            port.flush()
            time.sleep(0.001)

            # Set the GPIO pin 3 to HIGH to enable the front (left and right) motors
            GPIO.output(2, GPIO.LOW)
            GPIO.output(3, GPIO.HIGH)
            time.sleep(0.001)

            # Send the power levels to the serial port
            port.write(bytes([command_arr[2]]))
            port.flush()
            port.write(bytes([command_arr[3]]))
            port.flush()
            time.sleep(0.001)

# Function to display the menu
def display_menu(stdscr, current_row, power_levels, status):
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    # Define the menu items (remove "ALL Wheel Status")
    menu = ["Front Left", "Front Right", "Back Left", "Back Right", "Exit"]

    # Calculate the maximum width for the menu item text
    max_menu_length = max(len(item) for item in menu)

    # Display the overall wheel status at the top
    uart_str = "ON" if status[0] else "OFF"
    status_str = "ON" if status[1] else "OFF"
    stdscr.addstr(0, 0, f"UART Status: {uart_str}")
    stdscr.addstr(1, 0, f"ALL Wheel Status: {status_str}")

    for idx, row in enumerate(menu):
        x = w // 2 - max_menu_length // 2 - 15  # Center the menu
        y = h // 2 - len(menu) // 2 + idx
        
        if idx == current_row:
            stdscr.attron(curses.color_pair(1))

        # For the first 4 options, display the power level
        if idx < 4:
            stdscr.addstr(y, x, f"{row:<15} [Power: {power_levels[idx]:<3}]")  # Aligning the power level
        else:
            stdscr.addstr(y, x, row)  # 'Exit' option has no power level

        if idx == current_row:
            stdscr.attroff(curses.color_pair(1))

    stdscr.refresh()

def main(stdscr):
    # Set up the GPIO pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT) # GPIO pin 2 for the front motors
    GPIO.setup(3, GPIO.OUT) # GPIO pin 3 for the back motors

    # Initialize curses
    curses.curs_set(0)
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)

    # Wheel power levels (starting from 0)
    power_levels = [64, 192, 64, 192]  # Front Left, Front Right, Back Left, Back Right
    status = [0b0, 0b00000000]  # Initial status for all wheels

    current_row = 0
    max_row = 4  # Total menu items (now including only 4 wheels and 'Exit')

    # Set up the serial port
    port = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

    # Start the serial sending thread
    threading.Thread(target=send_to_serial, args=(port, power_levels, status), daemon=True).start()

    while True:
        display_menu(stdscr, current_row, power_levels, status)
        key = stdscr.getch()

        # Navigate through menu with up/down keys
        if key == curses.KEY_UP and current_row > 0:
            current_row -= 1
        elif key == curses.KEY_DOWN and current_row < max_row:
            current_row += 1

        # Adjust power levels with left/right keys if a wheel is selected
        elif key == curses.KEY_LEFT:
            # if between (0, 127)
            if current_row in [0,2] and power_levels[current_row] > 1:
                power_levels[current_row] -= 1
            elif current_row in [1,3] and power_levels[current_row] > 128:
                power_levels[current_row] -= 1
        elif key == curses.KEY_RIGHT:
            if current_row in [0,2] and power_levels[current_row] < 127:
                power_levels[current_row] += 1
            elif current_row in [1,3] and power_levels[current_row] < 255:
                power_levels[current_row] += 1


        # Toggle On/Off status for all wheels with the Spacebar
        elif key == 32:  # Spacebar key code
            status[1] = status[1] ^ 0b11111111  # Toggle all bits

        # Exit the program if 'Exit' is selected or 'Enter' is pressed on Exit
        elif key == curses.KEY_ENTER or key in [10, 13]:
            if current_row == 4:  # 'Exit' option
                break
            else:
                # Toggle the status of the UART
                status[0] = status[0] ^ 0b1

        # Refresh after each input
        stdscr.refresh()

    # Close the serial port when exiting
    port.close()

# Run the curses wrapper
curses.wrapper(main)
