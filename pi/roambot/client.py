import pygame
import time
import asyncio
import websockets

# Initialize pygame
pygame.init()

# Create a screen (required for Pygame to run properly, even if you're not using it for graphics)
screen = pygame.display.set_mode((640, 480))

# Initialize the message to be sent
message = "Stop"

async def send_message():
    uri = "ws://192.168.1.92:64912"  # Change 'localhost' to the server's IP if needed
    websocket = None
    try:
        websocket = await websockets.connect(uri)
        print(f"Connected to server at {uri}")
        while True:
            # Send the message to the server every 1 second
            await asyncio.sleep(0.1)
            print(f"Sending: {message}")
            await websocket.send(message)

    except Exception as e:
        print(f"Connection failed...{e}")

    finally:
        if websocket:
            await websocket.close()
            print("Connection closed...")

async def poll_keys():
    global message
    while True:
        count = 0

        # Check for events (e.g., quit event)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        # Get the current state of all keys (True means the key is pressed)
        keys = pygame.key.get_pressed()
        count += keys[ord('w')]
        count += keys[ord('a')]
        count += keys[ord('s')]
        count += keys[ord('d')]

        # Print out the pressed keys
        if count == 1:
            if keys[ord('w')]:
                message = "US"
            elif keys[ord('a')]:
                message = "LS"
            elif keys[ord('s')]:
                message = "DS"
            elif keys[ord('d')]:
                message = "RS"
            else:
                message = "SS"
        elif count == 2:
            if keys[ord('w')] and keys[ord('a')]:
                message = "UL"
            elif keys[ord('w')] and keys[ord('d')]:
                message = "UR"
            elif keys[ord('s')] and keys[ord('a')]:
                message = "DL"
            elif keys[ord('s')] and keys[ord('d')]:
                message = "DR"
            else:
                message = "SS"
        else:
            message = "SS"

        # Delay to limit CPU usage
        await asyncio.sleep(0.1)

async def main():
    # Run both tasks concurrently
    await asyncio.gather(
        poll_keys(),  # Run the polling function in a separate thread
        send_message()  # Run the websocket message sending task
    )

# Run the main function
asyncio.run(main())
