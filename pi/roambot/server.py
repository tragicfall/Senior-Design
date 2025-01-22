# server.py
import asyncio
import websockets

# Define the handler function for incoming WebSocket connections
async def handler(websocket):
    print("Client connected...")
    try:
        while True:
            message = await websocket.recv()
            print(f"Received: {message}")
    except websockets.ConnectionClosed:
        print("Client disconnected...")

# Main function to start the server
async def main():
    server = await websockets.serve(handler, "0.0.0.0", 64912)
    print("WebSocket server is running on ws://0.0.0.0:64912")
    await asyncio.Future()  # Keeps the server running indefinitely

# Run the server
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("\nServer has been stopped...")

