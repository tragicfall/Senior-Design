######################################################################
## Instructions
######################################################################

##### SETUP #####
# sudo apt update
# sudo apt install python3-venv -y
# sudo gpiodetect
# cd ~/roambot
# python3 -m venv myenv
# source ~/roambot/myenv/bin/activate
# pip install gpiod
# pip install pygame

##### TO RUN #####
# sudo ~/roambot/myenv/bin/python3 ~/roambot/gpio_test.py

######################################################################
## Imports
######################################################################

import gpiod
import time
import pygame

from gpiod.line import Direction, Value

######################################################################
## GPIO Pin Functions
######################################################################

def setup_gpio():
    chip = gpiod.Chip('/dev/gpiochip4')
    lines = chip.request_lines(consumer="gpio_test", config={
        5: gpiod.LineSettings(
            direction=Direction.OUTPUT, output_value=Value.INACTIVE
        ),
        6: gpiod.LineSettings(
            direction=Direction.OUTPUT, output_value=Value.INACTIVE
        ),
        13: gpiod.LineSettings(
            direction=Direction.OUTPUT, output_value=Value.INACTIVE
        ),
        19: gpiod.LineSettings(
            direction=Direction.OUTPUT, output_value=Value.INACTIVE
        )
    })
    return chip, lines

def clean_gpio(chip):
    chip.close()

def set_gpio(lines, line_num, value):
    if value == 0:
        lines.set_value(line_num, Value.INACTIVE)
    else:
        lines.set_value(line_num, Value.ACTIVE)

######################################################################
## Main Program
######################################################################

# Setup GPIO lines
chip, lines = setup_gpio()

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
            GPIO_UP = True; GPIO_LEFT = False; GPIO_DOWN = False; GPIO_RIGHT = False;
        elif keys[pygame.K_a]:
            GPIO_UP = False; GPIO_LEFT = True; GPIO_DOWN = False; GPIO_RIGHT = False;
        elif keys[pygame.K_s]:
            GPIO_UP = False; GPIO_LEFT = False; GPIO_DOWN = True; GPIO_RIGHT = False;
        elif keys[pygame.K_d]:
            GPIO_UP = False; GPIO_LEFT = False; GPIO_DOWN = False; GPIO_RIGHT = True;
        else:
            GPIO_UP = False; GPIO_LEFT = False; GPIO_DOWN = False; GPIO_RIGHT = False;
    else:
        # More than one key pressed, reset all GPIOs
        GPIO_UP = False; GPIO_LEFT = False; GPIO_DOWN = False; GPIO_RIGHT = False;

    # Update GPIO lines based on key presses
    set_gpio(lines, 5, GPIO_UP)
    set_gpio(lines, 6, GPIO_LEFT)
    set_gpio(lines, 13, GPIO_DOWN)
    set_gpio(lines, 19, GPIO_RIGHT)

    # Update the display
    pygame.display.flip()

    # Debug Information
    # print(f"GPIO_UP: {GPIO_UP}, GPIO_LEFT: {GPIO_LEFT}, GPIO_DOWN: {GPIO_DOWN}, GPIO_RIGHT: {GPIO_RIGHT}")

# Clean up GPIO lines
clean_gpio(chip)

# Quit pygame
pygame.quit()

