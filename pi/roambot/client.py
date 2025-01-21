# client.py
import asyncio
import websockets

async def client():
    uri = "ws://192.168.1.92:64912"  # Change 'localhost' to the server's IP if needed
    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to server at {uri}")
            
            # Send a message to the server
            message = "Hello, Server!"
            print(f"Sending: {message}")
            await websocket.send(message)
            
            # Wait for the server's response
            response = await websocket.recv()
            print(f"Received from server: {response}")
    except Exception as e:
        print(f"Connection failed: {e}")

# Run the client
if __name__ == "__main__":
    asyncio.run(client())
