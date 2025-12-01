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
        <div className="flex items-center gap-4">
          {!connected && (
            <div className="text-xs text-red-400 animate-pulse">
              Waiting for ROS2... (Is ./run_demo.sh running?)
            </div>
          )}
          <div className={`flex items-center gap-2 px-4 py-2 rounded-full ${connected ? 'bg-green-900/30 text-green-400 border border-green-800' : 'bg-red-900/30 text-red-400 border border-red-800'}`}>
            {connected ? <Wifi size={20} /> : <WifiOff size={20} />}
            <span className="font-semibold">{connected ? 'Connected' : 'Disconnected'}</span>
          </div>
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

          {/* Scenario Controller */}
          <div className="bg-slate-800/50 p-6 rounded-xl border border-slate-700">
            <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
              <Zap size={20} className="text-yellow-400" />
              Investor Scenarios
            </h3>
            <div className="space-y-4">
              {/* Scenario 1 */}
              <div className="p-4 bg-slate-700/50 rounded-lg border border-slate-600 hover:border-blue-400 transition-all cursor-pointer group" onClick={() => injectFault('LIDAR')}>
                <div className="flex justify-between items-center mb-2">
                  <span className="font-bold text-blue-300">Scenario A: LiDAR Failure</span>
                  <span className="text-xs bg-red-900/50 text-red-300 px-2 py-1 rounded border border-red-800">Critical</span>
                </div>
                <p className="text-sm text-slate-400 mb-2">
                  Simulates a sudden hardware failure of the primary LiDAR sensor during autonomous navigation.
                </p>
                <div className="text-xs font-mono text-green-400 opacity-0 group-hover:opacity-100 transition-opacity">
                  Expected Reaction: Immediate Emergency Stop
                </div>
              </div>

              {/* Scenario 2 */}
              <div className="p-4 bg-slate-700/50 rounded-lg border border-slate-600 hover:border-blue-400 transition-all cursor-pointer group" onClick={() => injectFault('CAMERA')}>
                <div className="flex justify-between items-center mb-2">
                  <span className="font-bold text-blue-300">Scenario B: Camera Freeze</span>
                  <span className="text-xs bg-yellow-900/50 text-yellow-300 px-2 py-1 rounded border border-yellow-800">Warning</span>
                </div>
                <p className="text-sm text-slate-400 mb-2">
                  Simulates a frozen video feed from the front-facing camera.
                </p>
                <div className="text-xs font-mono text-green-400 opacity-0 group-hover:opacity-100 transition-opacity">
                  Expected Reaction: Safe Stop (Deceleration)
                </div>
              </div>

              {/* Reset */}
              <button onClick={resetSystem} className="w-full p-3 bg-blue-900/30 hover:bg-blue-800/50 border border-blue-700 text-blue-300 rounded-lg transition-all text-sm font-bold tracking-wider">
                RESET SIMULATION
              </button>
            </div>
          </div>
        </div>

        {/* Middle Column: Live Map (Canvas Visualization) */}
        <div className="lg:col-span-2 space-y-6">
          <div className="bg-slate-900 p-1 rounded-xl border border-slate-700 h-[500px] relative overflow-hidden shadow-2xl">
            <h3 className="text-lg font-semibold mb-4 flex items-center gap-2 absolute top-6 left-6 z-10 text-white/80 bg-slate-900/50 px-3 py-1 rounded-full backdrop-blur-sm border border-white/10">
              <Map size={20} className="text-blue-400" />
              LIDAR SLAM Visualization
            </h3>

            {/* Overlay UI */}
            <div className="absolute top-6 right-6 z-10 flex flex-col items-end gap-2">
              <div className="text-xs font-mono text-blue-400/80">VELOCITY: {(Math.random() * 0.5).toFixed(2)} m/s</div>
              <div className="text-xs font-mono text-blue-400/80">HEADING: {(robotPose.x * 10).toFixed(1)}°</div>
            </div>

            <CanvasMap
              robotPose={robotPose}
              emergencyMode={emergencyMode}
              faultData={faultData}
            />
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

// Canvas Visualization Component
function CanvasMap({ robotPose, emergencyMode, faultData }) {
  const canvasRef = useRef(null);
  const [rotation, setRotation] = useState(0);

  useEffect(() => {
    let animationFrameId;

    const render = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const width = canvas.width;
      const height = canvas.height;

      // Clear
      ctx.fillStyle = '#0f172a'; // Slate-900
      ctx.fillRect(0, 0, width, height);

      // Draw Grid
      ctx.strokeStyle = '#1e293b'; // Slate-800
      ctx.lineWidth = 1;
      const gridSize = 40;
      for (let x = 0; x < width; x += gridSize) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
      }
      for (let y = 0; y < height; y += gridSize) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
      }

      // Center (Robot Position)
      const cx = width / 2 + robotPose.x * 20;
      const cy = height / 2 - robotPose.y * 20;

      // Draw Safe Zone (Static relative to map center for demo)
      const szX = width / 2 + 100;
      const szY = height / 2;
      ctx.strokeStyle = '#22c55e'; // Green-500
      ctx.setLineDash([5, 5]);
      ctx.strokeRect(szX - 40, szY - 40, 80, 80);
      ctx.fillStyle = 'rgba(34, 197, 94, 0.1)';
      ctx.fillRect(szX - 40, szY - 40, 80, 80);
      ctx.fillStyle = '#22c55e';
      ctx.font = '12px monospace';
      ctx.fillText('SAFE ZONE', szX - 30, szY - 50);
      ctx.setLineDash([]);

      // Draw LIDAR Points (Simulated)
      if (!faultData || faultData.type !== 'LIDAR') {
        ctx.fillStyle = '#60a5fa'; // Blue-400
        for (let i = 0; i < 360; i += 10) {
          const angle = (i * Math.PI) / 180 + rotation;
          const dist = 50 + Math.random() * 100;
          const px = cx + Math.cos(angle) * dist;
          const py = cy + Math.sin(angle) * dist;
          ctx.beginPath();
          ctx.arc(px, py, 2, 0, Math.PI * 2);
          ctx.fill();
        }
      }

      // Draw Robot Body
      ctx.save();
      ctx.translate(cx, cy);
      // Robot Shape
      ctx.fillStyle = emergencyMode ? '#ef4444' : '#3b82f6'; // Red or Blue
      ctx.shadowColor = emergencyMode ? '#ef4444' : '#3b82f6';
      ctx.shadowBlur = 20;
      ctx.beginPath();
      ctx.arc(0, 0, 12, 0, Math.PI * 2);
      ctx.fill();

      // Heading Indicator
      ctx.rotate(rotation * 2); // Spin effect for demo
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(15, 0);
      ctx.stroke();
      ctx.restore();

      // Radar Sweep Effect
      if (!faultData || faultData.type !== 'LIDAR') {
        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate(rotation);
        const gradient = ctx.createConicGradient(0, 0, 0);
        gradient.addColorStop(0, 'rgba(59, 130, 246, 0)');
        gradient.addColorStop(0.8, 'rgba(59, 130, 246, 0.1)');
        gradient.addColorStop(1, 'rgba(59, 130, 246, 0.4)');
        ctx.fillStyle = gradient;
        ctx.beginPath();
        ctx.arc(0, 0, 200, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }

      // Update Rotation
      setRotation(r => r + 0.02);
      animationFrameId = requestAnimationFrame(render);
    };

    render();
    return () => cancelAnimationFrame(animationFrameId);
  }, [robotPose, emergencyMode, faultData, rotation]);

  return <canvas ref={canvasRef} width={800} height={500} className="w-full h-full bg-slate-900" />;
}

export default App;
