import asyncio
import websockets

async def client():
    uri = "ws://127.0.0.1:64912"  # Change 'localhost' to the server's IP if needed
    websocket = None
    try:
        websocket = await websockets.connect(uri)
        print(f"Connected to server at {uri}")
        while True:
            # Get the user's message
            message = input("Enter message to send to server (or 'exit' to quit): ")
            
            if message.lower() == 'exit':  # Exit if the user types 'exit'
                print("Closing connection...")
                break
            
            # Send the message to the server
            print(f"Sending: {message}")
            await websocket.send(message)
    
    except Exception as e:
        print(f"Connection failed...{e}")

    finally:
        if websocket:
            await websocket.close()
            print("Connection closed...")

# Run the client
asyncio.run(client())
