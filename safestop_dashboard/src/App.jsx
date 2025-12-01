import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import { AlertTriangle, Activity, Map, Terminal, Wifi, WifiOff, Zap } from 'lucide-react';

function App() {
  const [connected, setConnected] = useState(false);
  const [ros, setRos] = useState(null);
  const [faultData, setFaultData] = useState(null);
  const [logs, setLogs] = useState([]);
  const [robotPose, setRobotPose] = useState({ x: 0, y: 0 });
  const [emergencyMode, setEmergencyMode] = useState(false);

  // Initialize ROS Connection
  useEffect(() => {
    const rosConnection = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    rosConnection.on('connection', () => {
      console.log('Connected to websocket server.');
      setConnected(true);
    });

    rosConnection.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      setConnected(false);
    });

    rosConnection.on('close', () => {
      console.log('Connection to websocket server closed.');
      setConnected(false);
    });

    setRos(rosConnection);

    return () => {
      rosConnection.close();
    };
  }, []);

  // Subscribers
  useEffect(() => {
    if (!ros) return;

    // Fault Trigger Subscriber
    const faultListener = new ROSLIB.Topic({
      ros: ros,
      name: '/fault_trigger',
      messageType: 'std_msgs/String'
    });

    faultListener.subscribe((message) => {
      const data = JSON.parse(message.data);
      setFaultData(data);
      setEmergencyMode(true);
      addLog(`FAULT: ${data.type}`, 'error');
    });

    // V2X Alert Subscriber (Mocking via same topic or separate)
    // In our python code v2x_bridge publishes to MQTT, but for dashboard we can listen to /fault_trigger 
    // OR we can listen to a new topic if we updated v2x_bridge to publish back to ROS.
    // For simplicity, we'll use /fault_trigger as the source of truth for "Alerts".

    // Odom Subscriber for Map
    const odomListener = new ROSLIB.Topic({
      ros: ros,
      name: '/odom',
      messageType: 'nav_msgs/Odometry'
    });

    odomListener.subscribe((message) => {
      // Simplified 2D pose
      setRobotPose({
        x: message.pose.pose.position.x,
        y: message.pose.pose.position.y
      });
    });

    return () => {
      faultListener.unsubscribe();
      odomListener.unsubscribe();
    };
  }, [ros]);

  const addLog = (msg, type = 'info') => {
    setLogs(prev => [{ time: new Date().toLocaleTimeString(), msg, type }, ...prev.slice(0, 9)]);
  };

  const injectFault = (type) => {
    if (!ros) return;

    const controlTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mock/control',
      messageType: 'std_msgs/String'
    });

    let payload = {};
    if (type === 'LIDAR') payload = { fail_lidar: true };
    if (type === 'CAMERA') payload = { fail_camera: true };
    if (type === 'IMU') payload = { fail_imu: true };

    const msg = new ROSLIB.Message({
      data: JSON.stringify(payload)
    });

    controlTopic.publish(msg);
    addLog(`Injecting ${type} Fault...`, 'warn');
  };

  const resetSystem = () => {
    if (!ros) return;
    // Reset mock sensors
    const controlTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mock/control',
      messageType: 'std_msgs/String'
    });
    const msg = new ROSLIB.Message({
      data: JSON.stringify({ fail_lidar: false, fail_camera: false, fail_imu: false })
    });
    controlTopic.publish(msg);

    setEmergencyMode(false);
    setFaultData(null);
    addLog('System Reset', 'success');
  };

  return (
    <div className="min-h-screen bg-slate-900 text-white p-6 font-sans">
      {/* Header */}
      <header className="flex justify-between items-center mb-8 border-b border-slate-700 pb-4">
        <div>
          <h1 className="text-3xl font-bold bg-gradient-to-r from-blue-400 to-cyan-300 bg-clip-text text-transparent">
            Vehicle SafeStop AI
          </h1>
          <p className="text-slate-400">Team 4 Investor Demo</p>
        </div>
        <div className={`flex items-center gap-2 px-4 py-2 rounded-full ${connected ? 'bg-green-900/30 text-green-400 border border-green-800' : 'bg-red-900/30 text-red-400 border border-red-800'}`}>
          {connected ? <Wifi size={20} /> : <WifiOff size={20} />}
          <span className="font-semibold">{connected ? 'Connected to ROS2' : 'Disconnected'}</span>
        </div>
      </header>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">

        {/* Left Column: Status & Controls */}
        <div className="space-y-6">

          {/* Status Gauge */}
          <div className={`relative p-8 rounded-2xl border-2 flex flex-col items-center justify-center transition-all duration-500 ${emergencyMode ? 'bg-red-950/50 border-red-500 shadow-[0_0_50px_rgba(239,68,68,0.3)]' : 'bg-slate-800/50 border-green-500/50'}`}>
            <div className={`w-32 h-32 rounded-full flex items-center justify-center mb-4 ${emergencyMode ? 'bg-red-500 animate-pulse' : 'bg-green-500'}`}>
              <Activity size={48} className="text-white" />
            </div>
            <h2 className="text-2xl font-bold tracking-wider">
              {emergencyMode ? 'EMERGENCY STOP' : 'SYSTEM NORMAL'}
            </h2>
            {emergencyMode && faultData && (
              <p className="mt-2 text-red-300 font-mono bg-red-900/50 px-3 py-1 rounded">
                CODE: {faultData.type}
              </p>
            )}
          </div>

          {/* Fault Controls */}
          <div className="bg-slate-800/50 p-6 rounded-xl border border-slate-700">
            <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
              <Zap size={20} className="text-yellow-400" />
              Fault Injection
            </h3>
            <div className="grid grid-cols-2 gap-3">
              <button onClick={() => injectFault('LIDAR')} className="p-3 bg-slate-700 hover:bg-red-900/50 hover:border-red-500 border border-slate-600 rounded-lg transition-all text-sm font-medium">
                Inject LiDAR Loss
              </button>
              <button onClick={() => injectFault('CAMERA')} className="p-3 bg-slate-700 hover:bg-red-900/50 hover:border-red-500 border border-slate-600 rounded-lg transition-all text-sm font-medium">
                Inject Camera Freeze
              </button>
              <button onClick={() => injectFault('IMU')} className="p-3 bg-slate-700 hover:bg-red-900/50 hover:border-red-500 border border-slate-600 rounded-lg transition-all text-sm font-medium">
                Inject IMU Drift
              </button>
              <button onClick={resetSystem} className="p-3 bg-blue-900/30 hover:bg-blue-800/50 border border-blue-700 text-blue-300 rounded-lg transition-all text-sm font-medium">
                Reset System
              </button>
            </div>
          </div>
        </div>

        {/* Middle Column: Live Map */}
        <div className="lg:col-span-2 space-y-6">
          <div className="bg-slate-800/50 p-6 rounded-xl border border-slate-700 h-[400px] relative overflow-hidden">
            <h3 className="text-lg font-semibold mb-4 flex items-center gap-2 absolute top-6 left-6 z-10">
              <Map size={20} className="text-blue-400" />
              Live Navigation
            </h3>

            {/* Simple CSS Map Visualization */}
            <div className="w-full h-full bg-slate-900 rounded-lg relative grid grid-cols-[repeat(20,1fr)] grid-rows-[repeat(20,1fr)] opacity-50">
              {/* Grid Lines */}
              {Array.from({ length: 400 }).map((_, i) => (
                <div key={i} className="border-[0.5px] border-slate-800" />
              ))}
            </div>

            {/* Robot */}
            <div
              className="absolute w-6 h-6 bg-blue-500 rounded-full border-2 border-white shadow-[0_0_20px_rgba(59,130,246,0.5)] transition-all duration-300"
              style={{
                left: `calc(50% + ${robotPose.x * 20}px)`,
                top: `calc(50% - ${robotPose.y * 20}px)`,
                transform: 'translate(-50%, -50%)'
              }}
            >
              <div className="absolute -top-8 left-1/2 -translate-x-1/2 text-xs whitespace-nowrap bg-slate-800 px-2 py-1 rounded border border-slate-600">
                TB3 Burger
              </div>
            </div>

            {/* Safe Zones (Mocked positions) */}
            <div className="absolute top-1/2 left-[70%] w-16 h-24 border-2 border-dashed border-green-500/50 bg-green-500/10 rounded flex items-center justify-center">
              <span className="text-green-500/50 text-xs font-bold">SAFE ZONE</span>
            </div>
          </div>

          {/* Log Feed */}
          <div className="bg-slate-800/50 p-6 rounded-xl border border-slate-700 h-[200px] overflow-hidden flex flex-col">
            <h3 className="text-lg font-semibold mb-2 flex items-center gap-2">
              <Terminal size={20} className="text-purple-400" />
              V2X Alert Feed
            </h3>
            <div className="flex-1 overflow-y-auto space-y-2 font-mono text-sm p-2 bg-slate-900/50 rounded">
              {logs.length === 0 && <span className="text-slate-600 italic">No alerts yet...</span>}
              {logs.map((log, i) => (
                <div key={i} className={`flex gap-3 ${log.type === 'error' ? 'text-red-400' : log.type === 'warn' ? 'text-yellow-400' : 'text-blue-300'}`}>
                  <span className="text-slate-500">[{log.time}]</span>
                  <span>{log.msg}</span>
                </div>
              ))}
            </div>
          </div>
        </div>

      </div>
    </div>
  );
}

export default App;
