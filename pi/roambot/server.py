# SaberTooth Controller 2x25 v2.0 (Mode: 011100) [0 is up, 1 is down]
# Switches [1:2] = Mode (01: Simple Serial)
# Switches [3] = Lithium Cutoff (1: Enabled)
# Switches [4:5] = Baudrate (10: 9600)
# Switches [6] = Slave Mode (0: Disabled)
# 5V TTL Logic


import asyncio
import websockets
import gpiod


chip = None
GPIO_05_U = None
GPIO_06_L = None
GPIO_13_D = None
GPIO_19_R = None


mapping = {
    "US": [1, 0, 0, 0],
    "LS": [0, 1, 0, 0],
    "DS": [0, 0, 1, 0],
    "RS": [0, 0, 0, 1],
    "SS": [0, 0, 0, 0]
}


# Function to setup gpio line
def setup_gpio_line(line_num):
    line = chip.get_line(line_num)
    line.request(consumer="server.py", type=gpiod.LINE_REQ_DIR_OUT)
    line.set_value(0)
    return line


# Function to send power levels to the serial port
def send_to_serial(power_levels):
    GPIO_05_U.set_value(power_levels[0])
    GPIO_06_L.set_value(power_levels[1])
    GPIO_13_D.set_value(power_levels[2])
    GPIO_19_R.set_value(power_levels[3])


# WebSocket handler
async def handler(websocket):
    print("Client connected...")
    message_old = "SS"
    message_new = "SS"
    try:
        while True:
            message_old = message_new
            message_new = await websocket.recv()
            power_levels = mapping[message_new]
            print(f"Received: {message_new}")
            if message_new != message_old:
                send_to_serial(power_levels)
            
    except websockets.ConnectionClosed:
        print("Client disconnected...")


# WebSocket server start
async def start():
    server = await websockets.serve(handler, "0.0.0.0", 64912)
    print("WebSocket server is running on ws://0.0.0.0:64912")
    await asyncio.Future()  # Keeps the server running indefinitely


# Main function
def main():
    global chip
    global GPIO_05_U
    global GPIO_06_L
    global GPIO_13_D
    global GPIO_19_R

    # Initialize GPIO lines to the requested lines list
    chip = gpiod.Chip('gpiochip0')
    GPIO_05_U = setup_gpio_line(5)
    GPIO_06_L = setup_gpio_line(6)
    GPIO_13_D = setup_gpio_line(13)
    GPIO_19_R = setup_gpio_line(19)

    try:
        asyncio.run(start())

    except KeyboardInterrupt:
        print("\nServer stopped by user...")

    finally:
        # Cleanup the GPIO lines
        GPIO_05_U.set_value(0)
        GPIO_06_L.set_value(0)
        GPIO_13_D.set_value(0)
        GPIO_19_R.set_value(0)
        GPIO_05_U.release()
        GPIO_06_L.release()
        GPIO_13_D.release()
        GPIO_19_R.release()
        chip.close()


# run the main function
main()