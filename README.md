# 3D Delta Robot Controller (NEMA 23 Actuators)

A high-performance control system and real-time 3D digital twin simulator for a **3-DOF Delta Robot** equipped with **NEMA 23 stepper actuators**, **150 mm bicep arms**, and **400 mm parallel carbon-fiber forearms**, modeled directly after the SolidWorks CAD assembly in `Delta_robot`.

---

## 🚀 Quick Start

### 1. Launch the Controller (Windows)
Double-click either:
- **`Launch_Delta_Robot_Controller.bat`** (located directly in `Delta_robot/`)
- Or run in terminal:
  ```bash
  cd delta_controller
  python main.py
  ```
The web controller will automatically launch in your browser at `http://localhost:8000`.

---

## 🎮 Jogging Modes & Features

### 1. Cartesian Jogging (XYZ)
- **Interactive 2D D-Pad**: Jog $X+, X-, Y+, Y-$ with tactile directional controls.
- **Z-Axis Elevation**: Dedicated $Z+$ (Up) and $Z-$ (Down) buttons.
- **Selectable Step Sizes**: `0.1 mm`, `1.0 mm`, `5.0 mm`, `10.0 mm`, `25.0 mm`, `50.0 mm`.
- **Variable Feedrate**: Smooth slider adjusting linear velocity from $10\text{ mm/s}$ to $400\text{ mm/s}$.
- **Direct Target Coordinates**: Enter exact $(X, Y, Z)$ coordinates and click **MOVE**.
- **Real-Time Kinematic Validation**: Automatically prevents out-of-reach moves and singularity zones.

### 2. Theta / Joint Jogging (Joints 1, 2, 3)
- **Direct Motor Control**: Control Motor 1 ($\theta_1, 0^\circ$), Motor 2 ($\theta_2, 120^\circ$), and Motor 3 ($\theta_3, 240^\circ$).
- **Angular Step Sizes**: `0.5°`, `1.0°`, `5.0°`, `10.0°`, `20.0°`, `45.0°`.
- **Individual Nudge Buttons & Sliders**: Dedicated `-` and `+` buttons and angular sliders.
- **Closed-Form Forward Kinematics (FK)**: Instantly computes and displays the resulting Cartesian position.

### 3. ⌨️ Keyboard Shortcuts
| Key | Action |
| :--- | :--- |
| **Arrow Right / Left** | Jog $X+$ / $X-$ |
| **Arrow Up / Down** | Jog $Y+$ / $Y-$ |
| **Page Up / Page Down** | Jog $Z+$ / $Z-$ |
| **1 / Q** | Jog Motor 1 $+$ / $-$ |
| **2 / W** | Jog Motor 2 $+$ / $-$ |
| **3 / E** | Jog Motor 3 $+$ / $-$ |
| **Spacebar** | **EMERGENCY STOP (E-STOP)** |

### 4. 🤖 Automated Trajectories & Presets
- **🏠 Home (G28)**: Return to calibrated home position.
- **🎯 Center Park**: Position at standard central workspace $(0, 0, -350\text{ mm})$.
- **⭕ Circle Path**: Executes a smooth circular trajectory in the horizontal XY plane.
- **📦 Pick & Place**: Executes a complete pick-lift-traverse-place cycle.

---

## 🔌 Hardware Connection Options

### Option A: WebSerial (Direct Browser-to-USB)
- Works directly inside Chrome / Edge with zero software drivers or server bridges needed.
- Select `⚡ WebSerial USB (Direct Browser)`, click **Connect**, and choose your Arduino / ESP32 COM port.

### Option B: Python Server (PySerial)
- Native serial streaming on Windows (`COM1`, `COM3`, etc.) with automatic port detection and telemetry feedback.

### Option C: Virtual Simulator
- Operates automatically if no physical hardware is plugged in, allowing complete software testing and trajectory preview.

---

## 🛠️ Microcontroller Firmware

The complete firmware is located in:
[`firmware/delta_robot_firmware.ino`](file:///c:/Users/DELL/Desktop/Delta_robot/delta_controller/firmware/delta_robot_firmware.ino)

Features:
- Multi-axis Bresenham stepping algorithm for 3x NEMA 23 stepper drivers.
- Closed-form onboard Inverse Kinematics.
- Standard G-code parser (`G0/G1`, `G28`, `M112`, `M114`, `M115`, `M999`).
- Homing limit switch support.
- Hardware wiring guide and DIP switch settings documented in [`firmware/README_FIRMWARE.md`](file:///c:/Users/DELL/Desktop/Delta_robot/delta_controller/firmware/README_FIRMWARE.md).
