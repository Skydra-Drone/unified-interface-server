import React, { useState, useEffect, useRef } from 'react';
import { Target, Radio, Plane, AlertCircle } from 'lucide-react';

function App() {
    const [socketUrl] = useState('ws://localhost:8000/ws');
    const [rosState, setRosState] = useState({
        status: "DISCONNECTED",
        targets_found: 0,
        drone_status: "UNKNOWN"
    });
    const [connected, setConnected] = useState(false);
    const ws = useRef(null);

    useEffect(() => {
        ws.current = new WebSocket(socketUrl);

        ws.current.onopen = () => {
            setConnected(true);
            console.log('WebSocket Connected');
        };

        ws.current.onclose = () => {
            setConnected(false);
            console.log('WebSocket Disconnected');
        };

        ws.current.onmessage = (event) => {
            const data = JSON.parse(event.data);
            setRosState(data);
        };

        return () => {
            ws.current.close();
        };
    }, [socketUrl]);

    const sendCommand = (cmd) => {
        if (ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify(cmd));
        }
    };

    return (
        <div className="min-h-screen bg-slate-950 text-slate-100 p-8 font-sans">
            <header className="mb-8 flex items-center justify-between border-b border-slate-800 pb-4">
                <div>
                    <h1 className="text-3xl font-bold bg-gradient-to-r from-cyan-400 to-blue-500 bg-clip-text text-transparent">
                        Unified Control
                    </h1>
                    <p className="text-slate-400 text-sm">Mission Control Dashboard</p>
                </div>
                <div className={`px-4 py-1 rounded-full text-sm font-medium ${connected ? 'bg-green-500/20 text-green-400' : 'bg-red-500/20 text-red-400'}`}>
                    {connected ? "● System Online" : "○ Disconnected"}
                </div>
            </header>

            <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mb-8">
                {/* Status Card */}
                <div className="bg-slate-900 border border-slate-800 p-6 rounded-xl relative overflow-hidden group">
                    <div className="absolute top-0 right-0 p-4 opacity-10 group-hover:opacity-20 transition-opacity">
                        <Radio size={48} />
                    </div>
                    <h3 className="text-slate-400 text-sm font-medium">ROS Connection</h3>
                    <p className="text-2xl font-bold mt-2">{rosState.status}</p>
                </div>

                {/* Metric Card */}
                <div className="bg-slate-900 border border-slate-800 p-6 rounded-xl relative overflow-hidden group">
                    <div className="absolute top-0 right-0 p-4 opacity-10 group-hover:opacity-20 transition-opacity">
                        <Target size={48} />
                    </div>
                    <h3 className="text-slate-400 text-sm font-medium">Targets Found</h3>
                    <p className="text-4xl font-mono mt-2 text-cyan-400">{rosState.targets_found}</p>
                </div>

                {/* Drone Card */}
                <div className="bg-slate-900 border border-slate-800 p-6 rounded-xl relative overflow-hidden group">
                    <div className="absolute top-0 right-0 p-4 opacity-10 group-hover:opacity-20 transition-opacity">
                        <Plane size={48} />
                    </div>
                    <h3 className="text-slate-400 text-sm font-medium">Drone Status</h3>
                    <p className="text-2xl font-bold mt-2">{rosState.drone_status}</p>
                </div>
            </div>

            {/* Mission Control */}
            <div className="bg-slate-900 border border-slate-800 rounded-xl p-8">
                <h2 className="text-xl font-bold mb-6 flex items-center gap-2">
                    <AlertCircle className="text-yellow-400" /> Mission Control
                </h2>

                <div className="flex gap-4">
                    <button
                        onClick={() => sendCommand({ trigger_mission: "START_SEARCH" })}
                        className="bg-cyan-600 hover:bg-cyan-500 text-white font-bold py-3 px-8 rounded-lg transition-all active:scale-95 shadow-lg shadow-cyan-900/20"
                    >
                        Start Mission
                    </button>
                    <button
                        onClick={() => sendCommand({ trigger_mission: "RETURN_HOME" })}
                        className="bg-slate-800 hover:bg-slate-700 text-white font-bold py-3 px-8 rounded-lg transition-all active:scale-95"
                    >
                        Recall Drone
                    </button>
                </div>
            </div>
        </div>
    )
}

export default App
