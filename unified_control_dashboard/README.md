# Unified Control Dashboard 🛸

> **"The Brain" of the Trinity Drone System.**
<img width="1917" height="857" alt="image" src="https://github.com/user-attachments/assets/ed117833-2640-4746-a690-285d6c03196e" />


##  The Trinity Architecture

This repository is **Part 1** of a 3-part distributed system designed for autonomous search and delivery operations.

| Component | Device | Responsibilities | Communication |
| :--- | :--- | :--- | :--- |
| **1. Unified Control** (This Repo) | **Windows Laptop** | **Standard C2 Interface.** UI, Mission Authorization, KML Upload, System Status. | **WebSocket / ROS Bridge** |
| **2. Perception Node** | Scout Drone (Jetson) | **The Eyes.** AI Inspection, YOLOv8 Object Detection, Camera Stream. | **ROS Native** |
| **3. Scout Node** | Scout Drone (Flight Controller) | **The Hands.** Flight Logic, Lawnmower Search Pattern, Collision Avoidance. | **MAVLink / ROS** |

###  How It Connects (The Bridge)
Since the **Unified Control Laptop** runs on **Windows** (which doesn't natively support ROS 1 Noetic), we use a **ROS Bridge** architecture:

1.  **Linux/WSL Side**: Runs `roscore` and `rosbridge_server`. This exposes the ROS network as a WebSocket on port `9090`.
2.  **Windows Side (Backend)**: Uses `roslibpy` to connect to `ws://localhost:9090`. It acts as a bridge, translating UI Commands -> ROS Topics.
3.  **Frontend**: Connects to the Backend via WebSockets (`ws://localhost:8000/ws`) to get real-time telemetry.

```mermaid
graph TD
    UI[🖥️ React Frontend] <-- "WebSocket (8000)" --> BE[🐍 Python Backend]
    BE <-- "roslibpy (9090)" --> BRIDGE[🌉 ROS Bridge Server (WSL)]
    BRIDGE <-- "ROS Topics" --> DRONE[🛸 Scout Drone]
```

---

##  Getting Started

### Prerequisites
*   **Node.js** (for Frontend)
*   **Python 3.10+** (for Backend)
*   **ROS Noetic** (on a Linux machine or WSL instance)

### 1. Start the ROS Bridge (Linux/WSL)
You must have the ROS bridge running to enable communication.
```bash
sudo apt-get install ros-noetic-rosbridge-suite
source ~/catkin_ws/devel/setup.bash  # IMPORTANT: Source your workspace!
roslaunch rosbridge_server rosbridge_websocket.launch
```

### 2. Start the Backend (Windows)
The "Brain" that bridges React to ROS.
```bash
cd backend
# Setup (First Time)
./setup.sh 

# Run
source venv/Scripts/activate
py -m app.main
```
*Server runs at `http://localhost:8000`*

### 3. Start the Frontend (Windows)
The "Hacker Terminal" UI.
```bash
cd frontend
npm install
npm run dev
```
*Dashboard runs at `http://localhost:5173`*

---

##  Features
*   **Hacker Terminal UI**: Minimalist, high-contrast aesthetics for field operations.
*   **KML Mission Upload**: Upload standard Google Earth KML files to define search boundaries.
*   **Real-time Telemetry**: Live target counting and status monitoring.
*   **Platform Independent**: The Dashboard runs on Windows, Mac, or Linux, decoupling the Operator from the Robot Operating System.

---

##  Tech Stack
*   **Frontend**: React, Vite, Tailwind CSS, Lucide Icons.
*   **Backend**: Python, FastAPI, roslibpy (ROS Bridge Client).
*   **Communication**: WebSockets, JSON.

---


