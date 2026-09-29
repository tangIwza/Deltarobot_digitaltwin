"""
Delta Robot Motion Controller & Hardware Interface
===================================================
Manages state, Cartesian jogging, Joint jogging, trajectory planning,
and PySerial / G-code communication with NEMA 23 stepper drivers.
"""

import time
import math
import threading
import json
import os
from typing import Dict, Any, List, Optional, Tuple

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from delta_kinematics import DeltaKinematics

class DeltaController:
    def __init__(self, config_path: Optional[str] = None):
        self.config_path = config_path or os.path.join(os.path.dirname(__file__), "config.json")
        self.config = self._load_default_config()
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r") as f:
                    user_cfg = json.load(f)
                    self.config.update(user_cfg)
            except Exception as e:
                print(f"Warning: could not load config.json: {e}")

        # Initialize kinematics
        self.kinematics = DeltaKinematics(
            bicep_length=self.config["kinematics"]["bicep_length"],
            forearm_length=self.config["kinematics"]["forearm_length"],
            base_radius=self.config["kinematics"]["base_radius"],
            platform_radius=self.config["kinematics"]["platform_radius"],
            phi_angles=self.config["kinematics"].get("phi_angles", [0.0, 120.0, 240.0]),
            theta_min_deg=self.config["limits"]["theta_min"],
            theta_max_deg=self.config["limits"]["theta_max"],
            z_min=self.config["limits"]["z_min"],
            z_max=self.config["limits"]["z_max"],
        )

        # Current Robot State
        self.cartesian_pos = list(self.config["home_position"]["cartesian"]) # [X, Y, Z]
        init_thetas, _ = self.kinematics.inverse_kinematics(*self.cartesian_pos)
        self.joint_thetas = init_thetas or [4.73, 4.73, 4.73]
        
        self.target_cartesian = list(self.cartesian_pos)
        self.target_thetas = list(self.joint_thetas)

        # Motion Parameters
        self.feedrate = self.config["motion"]["default_feedrate"]  # mm/s
        self.joint_feedrate = self.config["motion"]["joint_feedrate"] # deg/s
        self.is_homed = False
        self.is_moving = False
        self.emergency_stopped = False

        # Serial / Hardware
        self.serial_port = None
        self.baudrate = self.config["hardware"]["baudrate"]
        self.is_connected = False
        self.is_simulation = True
        self.serial_lock = threading.Lock()
        self.read_thread = None
        self.stop_read_thread = False
        self.command_log: List[Dict[str, Any]] = []

        # Trajectory execution thread
        self.motion_thread = None
        self.cancel_motion = False

    def _load_default_config(self) -> Dict[str, Any]:
        return {
            "robot_name": "3D Delta Robot NEMA 23",
            "kinematics": {
                "bicep_length": 150.0,      # mm
                "forearm_length": 400.0,    # mm
                "base_radius": 110.0,       # mm
                "platform_radius": 45.0,    # mm
                "phi_angles": [0.0, 120.0, 240.0]
            },
            "actuators": {
                "motor_type": "NEMA 23 (57BYG250C)",
                "step_angle_deg": 1.8,
                "microstepping": 16,
                "gear_ratio": 1.0,
                "steps_per_rev": 3200,      # 200 * 16
                "steps_per_deg": 8.888889,  # 3200 / 360
                "invert_axis": [False, False, False]
            },
            "limits": {
                "theta_min": -45.0,
                "theta_max": 85.0,
                "z_min": -550.0,
                "z_max": -200.0,
                "r_max": 240.0
            },
            "motion": {
                "default_feedrate": 100.0,   # mm/s
                "max_feedrate": 400.0,       # mm/s
                "joint_feedrate": 45.0,      # deg/s
                "acceleration": 800.0        # mm/s^2
            },
            "home_position": {
                "cartesian": [0.0, 0.0, -350.0],
                "thetas": [4.73, 4.73, 4.73]
            },
            "hardware": {
                "com_port": "AUTO",
                "baudrate": 115200
            }
        }

    def save_config(self) -> None:
        """Persist current configuration to config.json."""
        try:
            with open(self.config_path, "w") as f:
                json.dump(self.config, f, indent=2)
        except Exception as e:
            print(f"Error saving config: {e}")

    def update_kinematics_config(self, new_cfg: Dict[str, Any]) -> Tuple[bool, str]:
        """Update robot physical parameters at runtime."""
        try:
            self.kinematics.update_config(**new_cfg)
            if "bicep_length" in new_cfg:
                self.config["kinematics"]["bicep_length"] = float(new_cfg["bicep_length"])
            if "forearm_length" in new_cfg:
                self.config["kinematics"]["forearm_length"] = float(new_cfg["forearm_length"])
            if "base_radius" in new_cfg:
                self.config["kinematics"]["base_radius"] = float(new_cfg["base_radius"])
            if "platform_radius" in new_cfg:
                self.config["kinematics"]["platform_radius"] = float(new_cfg["platform_radius"])
            self.save_config()
            # Recompute joint thetas for current XYZ
            th, st = self.kinematics.inverse_kinematics(*self.cartesian_pos)
            if th:
                self.joint_thetas = th
            return True, "Kinematics configuration updated successfully"
        except Exception as e:
            return False, f"Failed to update kinematics: {e}"

    # =========================================================================
    # Hardware Connection & Serial Communication
    # =========================================================================

    def list_serial_ports(self) -> List[Dict[str, str]]:
        """List available system COM ports."""
        if not SERIAL_AVAILABLE:
            return [{"port": "SIMULATION", "desc": "Virtual Simulator Mode"}]
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append({"port": p.device, "desc": f"{p.device} - {p.description}"})
        if not ports:
            ports.append({"port": "SIMULATION", "desc": "No Hardware Found - Simulator Mode"})
        return ports

    def connect_serial(self, port: str, baudrate: int = 115200) -> Tuple[bool, str]:
        """Connect to hardware serial port (Arduino / ESP32) or start simulation mode."""
        self.emergency_stopped = False
        if port == "SIMULATION" or not SERIAL_AVAILABLE:
            self.is_connected = True
            self.is_simulation = True
            self._log_command("SYSTEM", f"Started in Virtual Simulator Mode")
            return True, "Connected in Virtual Simulation Mode"

        try:
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
                time.sleep(1.5) # Allow Arduino reset sequence
                self.is_connected = True
                self.is_simulation = False
                self.config["hardware"]["com_port"] = port
                self.config["hardware"]["baudrate"] = baudrate
                self.save_config()

            # Start background reading thread
            self.stop_read_thread = False
            self.read_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
            self.read_thread.start()

            self._log_command("SYSTEM", f"Connected to {port} @ {baudrate} baud")
            # Query firmware identification
            self.send_gcode("M115")
            return True, f"Successfully connected to {port}"
        except Exception as e:
            self.is_connected = False
            self.is_simulation = True
            return False, f"Failed to connect to {port}: {str(e)}"

    def disconnect_serial(self) -> Tuple[bool, str]:
        """Disconnect from serial port."""
        self.stop_read_thread = True
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.serial_port = None
        self.is_connected = False
        self.is_simulation = True
        self._log_command("SYSTEM", "Disconnected from hardware")
        return True, "Disconnected"

    def _serial_reader_loop(self):
        """Continuously reads telemetry and ok responses from firmware."""
        while not self.stop_read_thread and self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self._log_command("RECV", line)
                    # Parse status responses if firmware reports coordinates
                    if line.startswith("<") or "MPos:" in line or "POS:" in line:
                        self._parse_firmware_telemetry(line)
            except Exception:
                time.sleep(0.05)

    def _parse_firmware_telemetry(self, msg: str):
        # Optional telemetry parser for custom firmware or GRBL
        pass

    def send_gcode(self, gcode_str: str) -> bool:
        """Send G-code or command string to hardware."""
        self._log_command("SEND", gcode_str)
        if self.is_simulation or not self.serial_port or not self.serial_port.is_open:
            # Simulated hardware responds ok
            return True
        try:
            with self.serial_lock:
                self.serial_port.write((gcode_str + "\n").encode('utf-8'))
            return True
        except Exception as e:
            self._log_command("ERROR", f"Serial write error: {e}")
            return False

    def _log_command(self, direction: str, text: str):
        entry = {
            "time": time.strftime("%H:%M:%S"),
            "dir": direction,
            "text": text
        }
        self.command_log.append(entry)
        if len(self.command_log) > 100:
            self.command_log.pop(0)

    # =========================================================================
    # Cartesian Jogging (X+, X-, Y+, Y-, Z+, Z-)
    # =========================================================================

    def jog_cartesian(self, axis: str, step_mm: float, feedrate: Optional[float] = None) -> Tuple[bool, str]:
        """
        Jog along Cartesian axis ('X', 'Y', or 'Z') by step_mm.
        Uses Inverse Kinematics to calculate target joint angles, checks limits,
        and sends motion command.
        """
        if self.emergency_stopped:
            return False, "Cannot jog: Robot is in Emergency Stop state!"

        axis = axis.upper()
        if axis not in ['X', 'Y', 'Z']:
            return False, f"Invalid axis: {axis}. Must be 'X', 'Y', or 'Z'."

        feed = feedrate or self.feedrate
        new_x = self.cartesian_pos[0] + (step_mm if axis == 'X' else 0.0)
        new_y = self.cartesian_pos[1] + (step_mm if axis == 'Y' else 0.0)
        new_z = self.cartesian_pos[2] + (step_mm if axis == 'Z' else 0.0)

        return self.move_to_cartesian(new_x, new_y, new_z, feed)

    def move_to_cartesian(self, x: float, y: float, z: float, feedrate: Optional[float] = None) -> Tuple[bool, str]:
        """
        Moves the end-effector to an absolute Cartesian coordinate (X, Y, Z).
        """
        if self.emergency_stopped:
            return False, "Movement blocked: Emergency Stop active!"

        feed = feedrate or self.feedrate
        # Calculate Inverse Kinematics
        thetas, status = self.kinematics.inverse_kinematics(x, y, z)
        if thetas is None:
            return False, f"IK Error: {status}"

        # Update targets
        self.target_cartesian = [round(x, 2), round(y, 2), round(z, 2)]
        self.target_thetas = thetas

        # Send G-code command to hardware
        gcode = f"G1 X{x:.2f} Y{y:.2f} Z{z:.2f} F{feed * 60:.0f}"
        # Also include direct Joint format if controller firmware accepts it:
        # e.g.: "T th1 th2 th3"
        joint_cmd = f"G1 A{thetas[0]:.2f} B{thetas[1]:.2f} C{thetas[2]:.2f} F{feed * 60:.0f}"
        self.send_gcode(gcode)

        # Update internal position (in simulation, instantaneous or smooth)
        self.cartesian_pos = list(self.target_cartesian)
        self.joint_thetas = list(self.target_thetas)

        return True, f"Moved to ({x:.1f}, {y:.1f}, {z:.1f}) -> Thetas: [{thetas[0]}°, {thetas[1]}°, {thetas[2]}°]"

    # =========================================================================
    # Theta / Joint Jogging (Theta1, Theta2, Theta3)
    # =========================================================================

    def jog_theta(self, joint_idx: int, step_deg: float, feedrate: Optional[float] = None) -> Tuple[bool, str]:
        """
        Jog a specific joint (0, 1, or 2 for Theta1, Theta2, Theta3) by step_deg.
        Calculates new Cartesian position via Forward Kinematics.
        """
        if self.emergency_stopped:
            return False, "Cannot jog: Robot is in Emergency Stop state!"

        if joint_idx not in [0, 1, 2]:
            return False, f"Invalid joint index {joint_idx}. Must be 0, 1, or 2."

        feed = feedrate or self.joint_feedrate
        new_thetas = list(self.joint_thetas)
        new_thetas[joint_idx] += step_deg
        new_thetas[joint_idx] = round(new_thetas[joint_idx], 2)

        return self.move_to_thetas(new_thetas[0], new_thetas[1], new_thetas[2], feed)

    def move_to_thetas(self, th1: float, th2: float, th3: float, feedrate: Optional[float] = None) -> Tuple[bool, str]:
        """
        Moves the robot to absolute joint angles (Theta1, Theta2, Theta3 in degrees).
        """
        if self.emergency_stopped:
            return False, "Movement blocked: Emergency Stop active!"

        feed = feedrate or self.joint_feedrate
        thetas = [th1, th2, th3]

        # Check joint limits
        for i, th in enumerate(thetas):
            if th < self.kinematics.theta_min or th > self.kinematics.theta_max:
                return False, f"Theta {i+1} ({th:.1f}°) exceeds limits [{self.kinematics.theta_min}°, {self.kinematics.theta_max}°]"

        # Calculate Forward Kinematics to find corresponding Cartesian position
        cart_pos, status = self.kinematics.forward_kinematics(th1, th2, th3)
        if cart_pos is None:
            return False, f"FK Error: {status}"

        self.target_thetas = [round(th, 2) for th in thetas]
        self.target_cartesian = cart_pos

        # Send joint motion command to hardware
        gcode = f"G1 A{th1:.2f} B{th2:.2f} C{th3:.2f} F{feed * 60:.0f}"
        self.send_gcode(gcode)

        self.joint_thetas = list(self.target_thetas)
        self.cartesian_pos = list(self.target_cartesian)

        return True, f"Moved joints to [{th1:.1f}°, {th2:.1f}°, {th3:.1f}°] -> XYZ: ({cart_pos[0]}, {cart_pos[1]}, {cart_pos[2]})"

    # =========================================================================
    # Homing & Emergency Stop
    # =========================================================================

    def home(self) -> Tuple[bool, str]:
        """Execute Homing routine."""
        self.emergency_stopped = False
        self.send_gcode("G28")
        home_xyz = self.config["home_position"]["cartesian"]
        th, _ = self.kinematics.inverse_kinematics(*home_xyz)
        self.cartesian_pos = list(home_xyz)
        self.target_cartesian = list(home_xyz)
        if th:
            self.joint_thetas = list(th)
            self.target_thetas = list(th)
        self.is_homed = True
        return True, "Robot Homed successfully to default center position"

    def emergency_stop(self) -> Tuple[bool, str]:
        """Immediate Emergency Stop."""
        self.emergency_stopped = True
        self.is_moving = False
        self.cancel_motion = True
        self.send_gcode("M112") # Standard Emergency Stop
        self._log_command("ALERT", "EMERGENCY STOP TRIGGERED!")
        return True, "EMERGENCY STOP ACTIVATED! All motion terminated."

    def reset_estop(self) -> Tuple[bool, str]:
        """Reset emergency stop status."""
        self.emergency_stopped = False
        self.send_gcode("M999") # Reset fault
        self._log_command("SYSTEM", "Emergency Stop cleared.")
        return True, "Emergency Stop Cleared. Ready for operation."

    # =========================================================================
    # Trajectory Generation & Automated Sequences
    # =========================================================================

    def run_trajectory(self, trajectory_type: str, params: Optional[Dict[str, Any]] = None) -> Tuple[bool, str]:
        """
        Executes pre-programmed path sequences:
        - 'circle': smooth horizontal circle
        - 'pick_and_place': typical industrial pick-and-place routine
        - 'spiral': spiral raster scan
        """
        if self.is_moving:
            return False, "A motion sequence is already currently running."
        if self.emergency_stopped:
            return False, "Cannot run trajectory: Robot is in Emergency Stop state!"

        params = params or {}
        self.cancel_motion = False

        def trajectory_worker():
            self.is_moving = True
            try:
                if trajectory_type == "circle":
                    radius = float(params.get("radius", 60.0))
                    z_height = float(params.get("z", -360.0))
                    points_count = int(params.get("points", 36))
                    for i in range(points_count + 1):
                        if self.cancel_motion or self.emergency_stopped:
                            break
                        angle = 2.0 * math.pi * i / points_count
                        px = radius * math.cos(angle)
                        py = radius * math.sin(angle)
                        self.move_to_cartesian(px, py, z_height, feedrate=120.0)
                        time.sleep(0.04)

                elif trajectory_type == "pick_and_place":
                    # Pick at (-70, -50, -420), lift to -320, place at (70, 50, -420), return to home
                    waypoints = [
                        (0.0, 0.0, -320.0),      # Ready position
                        (-70.0, -50.0, -320.0),  # Above pick
                        (-70.0, -50.0, -420.0),  # Lower to pick
                        (-70.0, -50.0, -420.0),  # Dwell / grab
                        (-70.0, -50.0, -320.0),  # Lift up
                        (70.0, 50.0, -320.0),    # Traverse across
                        (70.0, 50.0, -420.0),    # Lower to place
                        (70.0, 50.0, -420.0),    # Release
                        (70.0, 50.0, -320.0),    # Lift up
                        (0.0, 0.0, -350.0),      # Return to home
                    ]
                    for wp in waypoints:
                        if self.cancel_motion or self.emergency_stopped:
                            break
                        self.move_to_cartesian(wp[0], wp[1], wp[2], feedrate=150.0)
                        time.sleep(0.15)

                elif trajectory_type == "spiral":
                    turns = int(params.get("turns", 4))
                    max_r = float(params.get("max_radius", 80.0))
                    z_height = float(params.get("z", -370.0))
                    total_pts = turns * 30
                    for i in range(total_pts + 1):
                        if self.cancel_motion or self.emergency_stopped:
                            break
                        frac = i / total_pts
                        r = frac * max_r
                        angle = frac * turns * 2.0 * math.pi
                        px = r * math.cos(angle)
                        py = r * math.sin(angle)
                        self.move_to_cartesian(px, py, z_height, feedrate=100.0)
                        time.sleep(0.03)

            finally:
                self.is_moving = False

        self.motion_thread = threading.Thread(target=trajectory_worker, daemon=True)
        self.motion_thread.start()
        return True, f"Started '{trajectory_type}' trajectory routine."

    def stop_motion(self) -> Tuple[bool, str]:
        """Cancel current trajectory motion."""
        self.cancel_motion = True
        return True, "Trajectory execution cancelled."

    # =========================================================================
    # Full Telemetry & State Export
    # =========================================================================

    def get_state(self) -> Dict[str, Any]:
        """Returns complete real-time telemetry dictionary for frontend sync."""
        joints = self.kinematics.get_joint_positions(self.joint_thetas, self.cartesian_pos)
        return {
            "cartesian": {
                "x": round(self.cartesian_pos[0], 2),
                "y": round(self.cartesian_pos[1], 2),
                "z": round(self.cartesian_pos[2], 2)
            },
            "thetas": {
                "theta1": round(self.joint_thetas[0], 2),
                "theta2": round(self.joint_thetas[1], 2),
                "theta3": round(self.joint_thetas[2], 2)
            },
            "target_cartesian": self.target_cartesian,
            "target_thetas": self.target_thetas,
            "feedrate": self.feedrate,
            "joint_feedrate": self.joint_feedrate,
            "status": {
                "is_connected": self.is_connected,
                "is_simulation": self.is_simulation,
                "is_homed": self.is_homed,
                "is_moving": self.is_moving,
                "emergency_stopped": self.emergency_stopped
            },
            "joints_3d": joints,
            "kinematics_config": self.kinematics.get_config(),
            "actuators_config": self.config["actuators"],
            "limits": self.config["limits"],
            "command_log": self.command_log[-20:]
        }
