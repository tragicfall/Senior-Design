# SaberTooth Controller 2x25 v2.0 (Mode: 011100) [0 is up, 1 is down]
# Switches [1:2] = Mode (01: Simple Serial)
# Switches [3] = Lithium Cutoff (1: Enabled)
# Switches [4:5] = Baudrate (10: 9600)
# Switches [6] = Slave Mode (0: Disabled)
# 5V TTL Logic

import curses
import serial
import RPi.GPIO as GPIO

# Function to send power level to the serial port
def send_to_serial(port, power_level):
    command = f"{chr(power_level)}"  # Send the power level
    port.write(bytes([power_level]))
    port.flush()

# Function to display the menu
def display_menu(stdscr, power_level, selected_item):
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    # Define the menu items
    menu = ["Send Motor Speed", "Exit"]

    # Display the current motor speed at the top
    stdscr.addstr(0, 0, f"Data: {power_level}")

    for idx, row in enumerate(menu):
        x = w // 2 - len(row) // 2  # Center the menu
        y = h // 2 - len(menu) // 2 + idx
        if idx == selected_item:  # Highlight the selected item
            stdscr.addstr(y, x, row, curses.A_REVERSE)
        else:
            stdscr.addstr(y, x, row)

    stdscr.refresh()

def main(stdscr):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT) 
    GPIO.output(2, GPIO.HIGH)
    # GPIO pin 2 for the front motors
    # Initialize curses
    curses.curs_set(0)

    # Initialize motor speed (starting from 0)
    power_level = 0  # Motor speed (0-255)

    # Set up the serial port
    port = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

    selected_item = 0  # Start with the first menu item

    while True:
        display_menu(stdscr, power_level, selected_item)
        key = stdscr.getch()

        # Adjust power level with left/right keys
        if key == curses.KEY_LEFT:
            if power_level > 0:
                power_level -= 1
        elif key == curses.KEY_RIGHT:
            if power_level < 255:
                power_level += 1

        # Navigate the menu with up/down keys
        elif key == curses.KEY_UP:
            selected_item = (selected_item - 1) % 2  # Wrap around
        elif key == curses.KEY_DOWN:
            selected_item = (selected_item + 1) % 2  # Wrap around

        # Send the command when Enter is pressed on "Send Motor Speed"
        elif key in [curses.KEY_ENTER, 10, 13] and selected_item == 0:
            send_to_serial(port, power_level)

        # Send stop command when Space is pressed
        elif key == 32:  # Spacebar key code
            send_to_serial(port, 0)

        # Exit the program when Enter is pressed on "Exit"
        elif key in [curses.KEY_ENTER, 10, 13] and selected_item == 1:
            break  # Exit the loop if Exit is selected

    # Close the serial port when exiting
    port.close()

# Run the curses wrapper
curses.wrapper(main)
