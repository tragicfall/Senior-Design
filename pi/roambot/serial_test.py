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
# pip install pygame

##### TO RUN #####
# sudo ~/roambot/myenv/bin/python3 ~/roambot/serial_test.py

######################################################################
## Imports
######################################################################

import serial
import time
import pygame

######################################################################
## GPIO Pin Functions
######################################################################

def setup_serial():
    return serial.Serial('/dev/ttyUSB1', 115200)

def clean_serial(ser):
    ser.close()

def set_gpio(ser, front_left, front_right, back_left, back_right):
    ser.write(bytes([front_left, front_right, back_left, back_right]))

######################################################################
## Main Program
######################################################################

# Setup GPIO lines
ser = setup_serial()

# Initialize pygame
pygame.init()

# Set up the display
width, height = 450, 400
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("GPIO Test")

# Define colors
black = (0, 0, 0)
white = (255, 255, 255)
gray = (160, 160, 160)
green = (0, 255, 0)

# Positions for the squares
square_size = 40
square_positions = {
    'W': (200, 150),  # top
    'A': (150, 200),  # left
    'S': (200, 200),  # bottom
    'D': (250, 200),  # right
}

# Define font for rendering text
font = pygame.font.SysFont('Arial', 26)

# Render letters and place them in the middle of each square
letter_texts = {}
for key, pos in square_positions.items():
    text_surface = font.render(key, True, gray)
    letter_texts[key] = text_surface

# Main loop
running = True
while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get the state of the keys
    keys = pygame.key.get_pressed()

    # Clear the screen
    screen.fill(black)

    # Draw squares
    pygame.draw.rect(screen, green if keys[pygame.K_w] else white, (square_positions['W'][0], square_positions['W'][1], square_size, square_size))
    pygame.draw.rect(screen, green if keys[pygame.K_a] else white, (square_positions['A'][0], square_positions['A'][1], square_size, square_size))
    pygame.draw.rect(screen, green if keys[pygame.K_s] else white, (square_positions['S'][0], square_positions['S'][1], square_size, square_size))
    pygame.draw.rect(screen, green if keys[pygame.K_d] else white, (square_positions['D'][0], square_positions['D'][1], square_size, square_size))

    # Blit the letters (always draw them)
    for key, pos in square_positions.items():
        screen.blit(letter_texts[key], (pos[0] + square_size // 4, pos[1] + square_size // 4))

    keys_pressed = pygame.key.get_pressed()
    keys_count = sum(keys_pressed)

    if keys_count == 1:
        # Only one key pressed, check which one
        if keys[pygame.K_w]:
            front_left = 9; front_right = 9; back_left = 8; back_right = 8;
        elif keys[pygame.K_a]:
            front_left = -20; front_right = 20; back_left = -20; back_right = 24;
        elif keys[pygame.K_s]:
            front_left = -8; front_right = -8; back_left = -9; back_right = -9;
        elif keys[pygame.K_d]:
            front_left = 20; front_right = -20; back_left = 20; back_right = -20;
        else:
            front_left = 0; front_right = 0; back_left = 0; back_right = 0;
    else:
        # More than one key pressed, reset all GPIOs
        front_left = 0; front_right = 0; back_left = 0; back_right = 0;

    # Update GPIO lines based on key presses
    set_gpio(ser, 64 + front_left, 192 - front_right, 64 + back_left, 192 - back_right)

    # Update the display
    pygame.display.flip()

    # Sleep
    time.sleep(0.01)  # Sleep for 10ms

# Clean up GPIO lines
clean_serial(ser)

# Quit pygame
pygame.quit()
