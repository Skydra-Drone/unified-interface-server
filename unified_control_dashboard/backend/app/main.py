from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import threading
import uvicorn
import asyncio
import json
from .ros_bridge import RosBridge

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global ROS Bridge
ros_node = RosBridge()

# Connection Manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

manager = ConnectionManager()

@app.on_event("startup")
async def startup_event():
    # Start ROS node in a separate thread
    threading.Thread(target=ros_node.run_ros_spin, daemon=True).start()

@app.get("/")
async def root():
    return {"message": "Unified Control Backend is Running"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # Send ROS state to UI
            state = ros_node.get_state()
            await websocket.send_json(state)
            
            # Receive commands from UI
            data = await websocket.receive_text()
            cmd = json.loads(data)
            
            if "trigger_mission" in cmd:
                ros_node.publish_mission_trigger(cmd["trigger_mission"])
                
            await asyncio.sleep(0.1) # 10Hz update rate
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
