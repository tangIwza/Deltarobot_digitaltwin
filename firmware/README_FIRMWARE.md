# 3D Delta Robot Hardware & Firmware Guide (NEMA 23)

This guide details the hardware wiring, stepper driver setup, and firmware flashing for the 3-DOF Delta Robot with **NEMA 23 actuators**, **150 mm bicep**, and **400 mm forearm**.

---

## 1. System Specifications

| Component | Specification |
| :--- | :--- |
| **Actuators** | 3x NEMA 23 Stepper Motors (Model: 57BYG250C) |
| **Step Angle** | 1.8° (200 full steps per revolution) |
| **Current Rating** | 2.5A – 3.0A per phase |
| **Bicep Length ($L$)** | **150.0 mm** |
| **Forearm Length ($l$)** | **400.0 mm** (parallel carbon rod pairs) |
| **Base Radius ($R$)** | **110.0 mm** (distance from base origin to motor shaft) |
| **Platform Radius ($r$)** | **45.0 mm** (distance from wrist center to ball joints) |
| **Recommended Drivers** | TB6600, DM542, TMC2209, or RAMPS 1.4 / DRV8825 |
| **Power Supply** | 24V DC, 10A – 15A (240W – 360W) |

---

## 2. Stepper Driver DIP Switch Settings (TB6600 / DM542)

For smooth, quiet motion and high positional accuracy with NEMA 23:

1. **Microstepping**: Set to **1/16 Microstepping** (3200 steps/revolution).
   - Yields $\approx 8.889\text{ steps per degree}$.
2. **Current Limit**: Set to **2.5A – 2.8A RMS** (matching 57BYG250C motor rating).
3. **Standstill Current**: Half Current (50%) to reduce motor heating when idling.

---

## 3. Microcontroller Pinout (Arduino Mega 2560 / RAMPS 1.4)

| Signal | RAMPS 1.4 Pin | Arduino Mega Pin | Description |
| :--- | :--- | :--- | :--- |
| **Motor 1 STEP** | X_STEP | Pin 54 | Step pulse for Arm 1 (0°) |
| **Motor 1 DIR** | X_DIR | Pin 55 | Direction for Arm 1 |
| **Motor 1 EN** | X_ENABLE | Pin 38 | Driver Enable (Active LOW) |
| **Motor 2 STEP** | Y_STEP | Pin 60 | Step pulse for Arm 2 (120°) |
| **Motor 2 DIR** | Y_DIR | Pin 61 | Direction for Arm 2 |
| **Motor 2 EN** | Y_ENABLE | Pin 56 | Driver Enable (Active LOW) |
| **Motor 3 STEP** | Z_STEP | Pin 46 | Step pulse for Arm 3 (240°) |
| **Motor 3 DIR** | Z_DIR | Pin 48 | Direction for Arm 3 |
| **Motor 3 EN** | Z_ENABLE | Pin 62 | Driver Enable (Active LOW) |
| **Endstop 1** | X_MIN | Pin 3 | Optical / Mechanical microswitch |
| **Endstop 2** | Y_MIN | Pin 14 | Optical / Mechanical microswitch |
| **Endstop 3** | Z_MIN | Pin 18 | Optical / Mechanical microswitch |

*(Note: If using an ESP32 or custom board, update the pin defines at the top of `delta_robot_firmware.ino` accordingly).*

---

## 4. Flashing the Firmware

1. Open the Arduino IDE.
2. Select **Tools > Board > Arduino Mega 2560** (or your respective board).
3. Select your microcontroller's **COM Port**.
4. Open [`delta_robot_firmware.ino`](file:///c:/Users/DELL/Desktop/Delta_robot/delta_controller/firmware/delta_robot_firmware.ino).
5. Click **Upload** (Ctrl + U).
6. Once uploaded, open the Delta Robot Web Controller:
   - Connect via **WebSerial** or select the COM port in the interface.
   - Baud Rate: **115200 baud**.

---

## 5. Supported G-Code & Serial Commands

| Command | Description | Example |
| :--- | :--- | :--- |
| `G1 X... Y... Z... F...` | Move End-Effector to Cartesian $(X,Y,Z)$ with onboard IK | `G1 X20 Y-10 Z-360 F3000` |
| `G1 A... B... C... F...` | Move Joint angles ($\theta_1, \theta_2, \theta_3$) directly | `G1 A10.5 B5.0 C12.0 F1800` |
| `G28` | Home all 3 arms to default reference | `G28` |
| `M112` | **Emergency Stop**: disables motor outputs immediately | `M112` |
| `M999` | **Reset Fault**: re-enables stepper drivers after E-Stop | `M999` |
| `M114` | Query current Cartesian & Joint position | `M114` |
| `M115` | Report firmware identification & parameters | `M115` |
