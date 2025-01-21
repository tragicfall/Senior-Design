# server.py
import asyncio
import websockets

# Define the handler function for incoming WebSocket connections
async def handler(websocket, path):
    print(f"Client connected from {path}")
    try:
        async for message in websocket:
            print(f"Received: {message}")
            response = f"Echo: {message}"
            await websocket.send(response)
            print(f"Sent: {response}")
    except websockets.ConnectionClosed:
        print("Client disconnected")

# Main function to start the server
async def main():
    server = await websockets.serve(handler, "0.0.0.0", 64912)
    print("WebSocket server is running on ws://0.0.0.0:64912")
    await asyncio.Future()  # Keeps the server running indefinitely

# Run the server
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")
