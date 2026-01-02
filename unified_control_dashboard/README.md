# Unified Control Dashboard

A modern, centralized Command & Control (C2) dashboard for the Scout & Delivery Drone system.

## 🏗️ Architecture
- **Frontend**: React (Vite) + Tailwind CSS + Lucide Icons.
- **Backend**: Python FastAPI (Async) + ROS Noetic Bridge.
- **Communication**: WebSockets (UI <-> Backend) and ROS Topics (Backend <-> System).

## 🚀 Getting Started

### 1. Prerequisites
- **Node.js**: Required for the frontend.
- **Python 3**: Required for FastAPI.
- **ROS Noetic**: Required for the bridge.

### 2. Setup Backend
Navigate to `backend`:
```bash
cd backend
# Install dependencies
pip install -r requirements.txt
# Run the Server (Ensure roscore is running!)
python3 -m app.main
```
The server runs at `http://localhost:8000`.

### 3. Setup Frontend
Navigate to `frontend`:
```bash
cd frontend
# Install dependencies
npm install
# Run the Dashboard
npm run dev
```
The dashboard runs at `http://localhost:5173`.

## 📡 Features
- **Real-time Telemetry**: Views `targets_found` and connection status live via WebSockets.
- **Mission Trigger**: detailed "Start Mission" button publishes directly to `/mission/command`.
