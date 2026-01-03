import React, { useState, useEffect, useRef } from 'react';
import { Terminal, Upload, Play, Octagon, Activity } from 'lucide-react';

function App() {
  const [socketUrl] = useState('ws://localhost:8000/ws');
  const [rosState, setRosState] = useState({
    status: "CONNECTING...",
    targets_found: 0,
    drone_status: "OFFLINE"
  });
  const [logs, setLogs] = useState(["Initialize System...", "Awaiting Handshake..."]);
  const ws = useRef(null);

  // Add log helper
  const addLog = (msg) => {
    const timestamp = new Date().toLocaleTimeString();
    const formattedMsg = `[${timestamp}] ${msg}`;
    setLogs(prev => [formattedMsg, ...prev].slice(0, 8));
  }

  useEffect(() => {
    ws.current = new WebSocket(socketUrl);

    ws.current.onopen = () => {
      addLog("LINK ESTABLISHED: ws://localhost:8000");
      setRosState(prev => ({ ...prev, status: "CONNECTED" })); // Update status on connection
    };

    ws.current.onclose = () => {
      addLog("LINK LOST: Retrying...");
      setRosState(prev => ({ ...prev, status: "DISCONNECTED" })); // Update status on disconnection
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
      addLog(`TX_CMD: ${JSON.stringify(cmd)}`);
    } else {
      addLog("ERROR: NO CARRIER");
    }
  };

  const handleFileUpload = (e) => {
    const file = e.target.files[0];
    if (file) {
      addLog(`READING: ${file.name}`);
      const reader = new FileReader();
      reader.onload = (ev) => {
        const content = ev.target.result;
        sendCommand({ upload_kml: content });
        addLog("TX_DATA: KML PAYLOAD SENT");
      };
      reader.readAsText(file);
    }
  };

  return (
    <div className="min-h-screen relative p-4 font-mono text-sm bg-black text-green-400">
      {/* CRT Overlay - assuming CSS for .scanline is provided elsewhere */}
      <div className="scanline pointer-events-none"></div>

      {/* Header */}
      <header className="mb-12 border-b-2 border-green-800 pb-4 flex justify-between items-end">
        <div>
          <h1 className="text-4xl font-bold uppercase tracking-widest glow">
            UNIFIED_CONTROL<span className="animate-pulse">_</span>
          </h1>
          <p className="text-xs uppercase text-green-700 mt-2">v.2.0.4 // SECURE CONNECTION</p>
        </div>
        <div className="text-right">
          <p>SYS_TIME: {new Date().toLocaleTimeString()}</p>
          <p>NET_STATUS: <span className={rosState.status === "CONNECTED" ? "text-green-400 bg-green-900 px-1" : "text-red-500 bg-red-900 px-1"}>{rosState.status}</span></p>
        </div>
      </header>

      <main className="grid grid-cols-1 lg:grid-cols-2 gap-12">
        {/* Metrics & Controls */}
        <section className="space-y-8">

          {/* Stats Block */}
          <div className="border border-green-800 p-4 bg-black relative">
            <div className="absolute -top-3 left-4 bg-black px-2 text-green-700 text-xs">TELEMETRY_DUMP</div>
            <div className="grid grid-cols-2 gap-4">
              <div>
                <p className="text-green-700 text-xs">TARGETS_ACQUIRED</p>
                <p className="text-5xl font-bold glow">{rosState.targets_found.toString().padStart(2, '0')}</p>
              </div>
              <div>
                <p className="text-green-700 text-xs">DRONE_STATE</p>
                <p className="text-2xl mt-2">{rosState.drone_status}</p>
              </div>
            </div>
          </div>

          {/* Controls Block */}
          <div className="border border-green-800 p-4 bg-black relative">
            <div className="absolute -top-3 left-4 bg-black px-2 text-green-700 text-xs">MISSION_OVERRIDE</div>

            <div className="flex flex-col gap-4">
              {/* KML Upload */}
              <div className="flex items-center gap-4 border border-green-900 p-2 hover:bg-green-900/20 cursor-pointer transition-colors group">
                <Upload size={20} />
                <label className="flex-1 cursor-pointer">
                  <span className="text-sm group-hover:underline">&gt; UPLOAD_SEARCH_GRID.KML</span>
                  <input type="file" onChange={handleFileUpload} accept=".kml" className="hidden" />
                </label>
              </div>

              <button onClick={() => sendCommand({ trigger_mission: "START_SEARCH" })} className="flex items-center gap-4 border border-green-900 p-2 hover:bg-green-500 hover:text-black cursor-pointer transition-colors text-left">
                <Play size={20} />
                <span className="text-sm font-bold">&gt; EXECUTE_PATTERN_ALPHA</span>
              </button>

              <button onClick={() => sendCommand({ trigger_mission: "RETURN_HOME" })} className="flex items-center gap-4 border border-red-900/50 p-2 hover:bg-red-900/50 text-red-500 hover:text-red-300 cursor-pointer transition-colors text-left">
                <Octagon size={20} />
                <span className="text-sm font-bold">&gt; ABORT / RTL</span>
              </button>
            </div>
          </div>
        </section>

        {/* System Log */}
        <section className="border border-green-800 p-4 h-96 relative">
          <div className="absolute -top-3 left-4 bg-black px-2 text-green-700 text-xs">SYSTEM_LOG</div>
          <div className="h-full overflow-y-auto font-mono text-sm space-y-1 scrollbar-hide">
            {logs.map((log, i) => (
              <div key={i} className="opacity-80 hover:opacity-100 transition-opacity">
                <span className="text-green-700 mr-2">&gt;</span>
                {log}
              </div>
            ))}
            <div className="animate-pulse">_</div>
          </div>
        </section>
      </main>
    </div>
  )
}

export default App
