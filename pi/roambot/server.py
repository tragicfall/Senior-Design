import asyncio
import websockets
import serial
import RPi.GPIO as GPIO

# Wheel power levels for each direction (Front Left, Front Right, Back Left, Back Right)
FL_REVERSE_FAST = 44
FL_REVERSE_SLOW = 54
FL_FULL_STOP     = 64
FL_FORWARD_SLOW  = 74
FL_FORWARD_FAST  = 84

FR_REVERSE_FAST = 172
FR_REVERSE_SLOW = 182
FR_FULL_STOP     = 192
FR_FORWARD_SLOW  = 202
FR_FORWARD_FAST  = 212

BL_REVERSE_FAST = 44
BL_REVERSE_SLOW = 54
BL_FULL_STOP     = 64
BL_FORWARD_SLOW  = 74
BL_FORWARD_FAST  = 84

BR_REVERSE_FAST = 172
BR_REVERSE_SLOW = 182
BR_FULL_STOP     = 192
BR_FORWARD_SLOW  = 202
BR_FORWARD_FAST  = 212

# Direction mappings
UP_STRAIGHT    = [FL_FORWARD_FAST, FR_FORWARD_FAST, BL_FORWARD_FAST, BR_FORWARD_FAST]
LEFT_STRAIGHT  = [FL_REVERSE_FAST, FR_FORWARD_FAST, BL_REVERSE_FAST, BR_FORWARD_FAST]
DOWN_STRAIGHT  = [FL_REVERSE_FAST, FR_REVERSE_FAST, BL_REVERSE_FAST, BR_REVERSE_FAST]
RIGHT_STRAIGHT = [FL_FORWARD_FAST, FR_REVERSE_FAST, BL_FORWARD_FAST, BR_REVERSE_FAST]

UP_LEFT        = [FL_FORWARD_SLOW, FR_FORWARD_FAST, BL_FORWARD_SLOW, BR_FORWARD_FAST]
UP_RIGHT       = [FL_FORWARD_FAST, FR_FORWARD_SLOW, BL_FORWARD_FAST, BR_FORWARD_SLOW]
DOWN_LEFT      = [FL_REVERSE_SLOW, FR_REVERSE_FAST, BL_REVERSE_SLOW, BR_REVERSE_FAST]
DOWN_RIGHT     = [FL_REVERSE_FAST, FR_REVERSE_SLOW, BL_REVERSE_FAST, BR_REVERSE_SLOW]

STOP_STOP      = [FL_FULL_STOP, FR_FULL_STOP, BL_FULL_STOP, BR_FULL_STOP]

mapping = {
    "US": UP_STRAIGHT,
    "LS": LEFT_STRAIGHT,
    "DS": DOWN_STRAIGHT,
    "RS": RIGHT_STRAIGHT,
    "UL": UP_LEFT,
    "UR": UP_RIGHT,
    "DL": DOWN_LEFT,
    "DR": DOWN_RIGHT,
    "SS": STOP_STOP,
}


# Function to send power levels to the serial port
def send_to_serial(port, power_levels):
    GPIO.output(3, GPIO.LOW)
    GPIO.output(2, GPIO.HIGH)

    port.write(bytes([power_levels[0]]))
    port.flush()
    port.write(bytes([power_levels[1]]))
    port.flush()

    GPIO.output(2, GPIO.LOW)
    GPIO.output(3, GPIO.HIGH)

    port.write(bytes([power_levels[2]]))
    port.flush()
    port.write(bytes([power_levels[3]]))
    port.flush()


# WebSocket handler
async def handler(websocket):
    print("Client connected...")
    port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
    message_old = "SS"
    message_new = "SS"
    try:
        while True:
            message_old = message_new
            message_new = await websocket.recv()
            power_levels = mapping[message_new]
            print(f"Received: {message_new}")
            if message_new != message_old:
                send_to_serial(port, power_levels)
            
    except websockets.ConnectionClosed:
        print("Client disconnected...")


# WebSocket server start
async def start():
    server = await websockets.serve(handler, "0.0.0.0", 64912)
    print("WebSocket server is running on ws://0.0.0.0:64912")
    await asyncio.Future()  # Keeps the server running indefinitely


# Main function
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT)  # GPIO pin 2 for the front motors
    GPIO.setup(3, GPIO.OUT)  # GPIO pin 3 for the back motors

    port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)  # Initialize serial port
    power_levels = STOP_STOP

    try:
        asyncio.run(start())
    except KeyboardInterrupt:
        print("\nServer stopped by user...")
    finally:
        port.close()
        GPIO.cleanup()

# run the main function
main()