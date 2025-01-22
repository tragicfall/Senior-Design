# server.py
import asyncio
import websockets
import RPi.GPIO as GPIO

# Wheel power levels for each direction (Front Left, Front Right, Back Left, Back Right)
# Stop (64, 192, 64, 192)

UP_STRAIGHT    = [84, 212, 84, 212] # All wheels move forward
LEFT_STRAIGHT  = [44, 212, 44, 212] # Left wheels move backward, right wheels move forward
DOWN_STRAIGHT  = [44, 172, 44, 172] # All wheels move backward
RIGHT_STRAIGHT = [84, 172, 84, 172] # Left wheels move forward, right wheels move backward

UP_LEFT        = [74, 212, 74, 212] # Front left wheel moves slower
UP_RIGHT       = [84, 202, 84, 202] # Front right wheel moves slower
DOWN_LEFT      = [54, 172, 54, 172] # Back left wheel moves slower
DOWN_RIGHT     = [64, 162, 64, 162] # Back right wheel moves slower

STOP_STOP      = [64, 192, 64, 192] # Stop all wheels

mapping = {
    "US": UP_STRAIGHT,
    "LS": LEFT_STRAIGHT,
    "DS": DOWN_STRAIGHT,
    "RS": RIGHT_STRAIGHT,
    "UL": UP_LEFT,
    "UR": UP_RIGHT,
    "DL": DOWN_LEFT,
    "DR": DOWN_RIGHT,
    "SS": STOP_STOP
}

# Define the handler function for incoming WebSocket connections
async def handler(websocket):
    print("Client connected...")
    try:
        while True:
            message = await websocket.recv()
            print(mapping[message])
    except websockets.ConnectionClosed:
        print("Client disconnected...")

# Start function to run the server
async def start():
    server = await websockets.serve(handler, "0.0.0.0", 64912)
    print("WebSocket server is running on ws://0.0.0.0:64912")
    await asyncio.Future()  # Keeps the server running indefinitely

# main
def main():
    # Set up the GPIO pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT) # GPIO pin 2 for the front motors
    GPIO.setup(3, GPIO.OUT) # GPIO pin 3 for the back motors

    try:
        asyncio.run(start())
    except KeyboardInterrupt:
        print("\nServer has been stopped...")

# Run the main function
main()