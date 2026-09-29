"""
FastAPI Server & WebSocket Bridge for Delta Robot Controller
============================================================
Provides REST API endpoints and real-time WebSocket telemetry for the web UI.
"""

import os
import json
import asyncio
from typing import Dict, Any, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel

from delta_controller import DeltaController

app = FastAPI(title="3D Delta Robot Controller API")

controller = DeltaController()

# Connected WebSocket clients
connected_clients = set()

# Models
class JogCartesianRequest(BaseModel):
    axis: str
    step: float
    feedrate: Optional[float] = None

class JogThetaRequest(BaseModel):
    joint: int  # 0, 1, or 2
    step: float
    feedrate: Optional[float] = None

class MoveCartesianRequest(BaseModel):
    x: float
    y: float
    z: float
    feedrate: Optional[float] = None

class MoveThetaRequest(BaseModel):
    theta1: float
    theta2: float
    theta3: float
    feedrate: Optional[float] = None

class SerialConnectRequest(BaseModel):
    port: str
    baudrate: int = 115200

class GcodeRequest(BaseModel):
    command: str

class TrajectoryRequest(BaseModel):
    type: str
    params: Optional[Dict[str, Any]] = None

class ConfigUpdateRequest(BaseModel):
    bicep_length: Optional[float] = None
    forearm_length: Optional[float] = None
    base_radius: Optional[float] = None
    platform_radius: Optional[float] = None
    theta_min: Optional[float] = None
    theta_max: Optional[float] = None
    z_min: Optional[float] = None
    z_max: Optional[float] = None

# Background telemetry broadcaster
async def broadcast_telemetry_loop():
    while True:
        if connected_clients:
            state = controller.get_state()
            state_json = json.dumps({"type": "telemetry", "data": state})
            disconnected = set()
            for ws in list(connected_clients):
                try:
                    await ws.send_text(state_json)
                except Exception:
                    disconnected.add(ws)
            for ws in disconnected:
                connected_clients.discard(ws)
        await asyncio.sleep(0.04)  # ~25 Hz broadcast rate for smooth animation

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_telemetry_loop())

# WebSocket endpoint
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    try:
        # Send initial state immediately
        await websocket.send_text(json.dumps({"type": "telemetry", "data": controller.get_state()}))
        while True:
            data_text = await websocket.receive_text()
            try:
                msg = json.loads(data_text)
                action = msg.get("action")
                if action == "jog_cartesian":
                    controller.jog_cartesian(msg["axis"], msg["step"], msg.get("feedrate"))
                elif action == "jog_theta":
                    controller.jog_theta(msg["joint"], msg["step"], msg.get("feedrate"))
                elif action == "move_cartesian":
                    controller.move_to_cartesian(msg["x"], msg["y"], msg["z"], msg.get("feedrate"))
                elif action == "move_thetas":
                    controller.move_to_thetas(msg["theta1"], msg["theta2"], msg["theta3"], msg.get("feedrate"))
                elif action == "home":
                    controller.home()
                elif action == "estop":
                    controller.emergency_stop()
                elif action == "reset_estop":
                    controller.reset_estop()
                elif action == "gcode":
                    controller.send_gcode(msg["command"])
            except Exception as e:
                print(f"WS command error: {e}")
    except WebSocketDisconnect:
        connected_clients.discard(websocket)

# REST Endpoints
@app.get("/api/state")
def get_state():
    return controller.get_state()

@app.post("/api/jog/cartesian")
def jog_cartesian(req: JogCartesianRequest):
    success, msg = controller.jog_cartesian(req.axis, req.step, req.feedrate)
    return {"success": success, "message": msg, "state": controller.get_state()}

@app.post("/api/jog/theta")
def jog_theta(req: JogThetaRequest):
    success, msg = controller.jog_theta(req.joint, req.step, req.feedrate)
    return {"success": success, "message": msg, "state": controller.get_state()}

@app.post("/api/move/cartesian")
def move_cartesian(req: MoveCartesianRequest):
    success, msg = controller.move_to_cartesian(req.x, req.y, req.z, req.feedrate)
    return {"success": success, "message": msg, "state": controller.get_state()}

@app.post("/api/move/theta")
def move_theta(req: MoveThetaRequest):
    success, msg = controller.move_to_thetas(req.theta1, req.theta2, req.theta3, req.feedrate)
    return {"success": success, "message": msg, "state": controller.get_state()}

@app.post("/api/home")
def home_robot():
    success, msg = controller.home()
    return {"success": success, "message": msg, "state": controller.get_state()}

@app.post("/api/estop")
def estop_robot():
    success, msg = controller.emergency_stop()
    return {"success": success, "message": msg, "state": controller.get_state()}

@app.post("/api/estop/reset")
def reset_estop():
    success, msg = controller.reset_estop()
    return {"success": success, "message": msg, "state": controller.get_state()}

@app.post("/api/trajectory")
def run_trajectory(req: TrajectoryRequest):
    success, msg = controller.run_trajectory(req.type, req.params)
    return {"success": success, "message": msg}

@app.post("/api/trajectory/stop")
def stop_trajectory():
    success, msg = controller.stop_motion()
    return {"success": success, "message": msg}

@app.get("/api/serial/ports")
def get_serial_ports():
    return {"ports": controller.list_serial_ports()}

@app.post("/api/serial/connect")
def connect_serial(req: SerialConnectRequest):
    success, msg = controller.connect_serial(req.port, req.baudrate)
    return {"success": success, "message": msg, "connected": controller.is_connected, "simulation": controller.is_simulation}

@app.post("/api/serial/disconnect")
def disconnect_serial():
    success, msg = controller.disconnect_serial()
    return {"success": success, "message": msg}

@app.post("/api/gcode")
def send_gcode(req: GcodeRequest):
    res = controller.send_gcode(req.command)
    return {"success": res, "state": controller.get_state()}

@app.post("/api/config")
def update_config(req: ConfigUpdateRequest):
    updates = {k: v for k, v in req.dict().items() if v is not None}
    success, msg = controller.update_kinematics_config(updates)
    return {"success": success, "message": msg, "config": controller.kinematics.get_config()}

@app.get("/api/workspace")
def get_workspace():
    pts = controller.kinematics.generate_workspace_cloud()
    return {"points": pts}

# Static web files
static_dir = os.path.join(os.path.dirname(__file__), "web")
if os.path.exists(static_dir):
    app.mount("/static", StaticFiles(directory=static_dir), name="static")
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="web_root")


if __name__ == "__main__":
    import uvicorn
    print("\n" + "="*60)
    print("  3D DELTA ROBOT CONTROLLER - STARTING SERVER")
    print("  Actuators: 3x NEMA 23 (57BYG250C)")
    print("  Bicep Length: 150 mm | Forearm Length: 400 mm")
    print("  Web Interface: http://localhost:8000")
    print("="*60 + "\n")
    uvicorn.run(app, host="127.0.0.1", port=8000)
