/**
 * =========================================================================
 * 3D DELTA ROBOT FIRMWARE FOR NEMA 23 ACTUATORS
 * =========================================================================
 * Hardware Target: Arduino Mega 2560 / RAMPS 1.4 / CNC Shield / ESP32
 * Motors: 3x NEMA 23 Stepper Motors (57BYG250C)
 * Drivers: TB6600, DM542, A4988, DRV8825, or TMC2209
 * Kinematics:
 *   - Bicep length (L): 150.0 mm
 *   - Forearm length (l): 400.0 mm
 *   - Base radius (R): 110.0 mm
 *   - Platform radius (r): 45.0 mm
 * =========================================================================
 */

#include <Arduino.h>

// =========================================================================
// PIN DEFINITIONS (Default RAMPS 1.4 / Arduino Mega Pinout)
// =========================================================================
// Motor 1 (Axis A / Theta 1 - 0 deg) -> X-Axis on RAMPS
#define STEP_PIN_1        54
#define DIR_PIN_1         55
#define ENABLE_PIN_1      38
#define MIN_PIN_1          3   // Limit Switch / Endstop 1

// Motor 2 (Axis B / Theta 2 - 120 deg) -> Y-Axis on RAMPS
#define STEP_PIN_2        60
#define DIR_PIN_2         61
#define ENABLE_PIN_2      56
#define MIN_PIN_2         14   // Limit Switch / Endstop 2

// Motor 3 (Axis C / Theta 3 - 240 deg) -> Z-Axis on RAMPS
#define STEP_PIN_3        46
#define DIR_PIN_3         48
#define ENABLE_PIN_3      62
#define MIN_PIN_3         18   // Limit Switch / Endstop 3

// =========================================================================
// KINEMATICS & ACTUATOR CONSTANTS
// =========================================================================
const float BICEP_L     = 150.0;   // L: 150 mm
const float FOREARM_L   = 400.0;   // l: 400 mm
const float BASE_R      = 110.0;   // R: 110 mm
const float PLAT_R      = 45.0;    // r: 45 mm
const float R_EFF       = BASE_R - PLAT_R; // 65.0 mm

// NEMA 23 Configuration
// 1.8 deg/step = 200 steps/rev. 1/16 Microstepping = 3200 steps/rev
const float STEPS_PER_REV  = 3200.0;
const float STEPS_PER_DEG  = STEPS_PER_REV / 360.0; // 8.888889 steps/deg

// Angles of the 3 arms (radians)
const float PHI[3] = { 0.0, 2.0943951, 4.1887902 }; // 0°, 120°, 240°

// Limits
const float THETA_MIN = -45.0;
const float THETA_MAX =  85.0;
const float Z_MIN     = -550.0;
const float Z_MAX     = -200.0;

// Current state
float cur_x = 0.0, cur_y = 0.0, cur_z = -350.0;
float cur_th[3] = { 4.73, 4.73, 4.73 };
long cur_steps[3] = { 0, 0, 0 };
bool emergency_stop = false;

// Serial Buffer
String input_string = "";
bool string_complete = false;

// Forward declarations
bool delta_ik(float x, float y, float z, float out_thetas[3]);
void move_thetas(float t1, float t2, float t3, float feed_deg_s);
void step_motor(int motor_idx, bool dir);

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  input_string.reserve(128);

  // Configure Pins
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);

  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);

  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(ENABLE_PIN_3, OUTPUT);

  // Active LOW Enable for stepper drivers
  digitalWrite(ENABLE_PIN_1, LOW);
  digitalWrite(ENABLE_PIN_2, LOW);
  digitalWrite(ENABLE_PIN_3, LOW);

  pinMode(MIN_PIN_1, INPUT_PULLUP);
  pinMode(MIN_PIN_2, INPUT_PULLUP);
  pinMode(MIN_PIN_3, INPUT_PULLUP);

  // Initialize Position
  delta_ik(cur_x, cur_y, cur_z, cur_th);
  for (int i = 0; i < 3; i++) {
    cur_steps[i] = round(cur_th[i] * STEPS_PER_DEG);
  }

  Serial.println(F("{\"firmware\":\"3D Delta Robot NEMA 23\",\"status\":\"ONLINE\",\"L\":150,\"l\":400}"));
  Serial.println(F("ok"));
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  if (string_complete) {
    parse_command(input_string);
    input_string = "";
    string_complete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (input_string.length() > 0) {
        string_complete = true;
      }
    } else {
      input_string += inChar;
    }
  }
}

// =========================================================================
// ONBOARD INVERSE KINEMATICS
// =========================================================================
bool delta_ik(float x, float y, float z, float out_thetas[3]) {
  if (z > Z_MAX || z < Z_MIN) return false;

  for (int i = 0; i < 3; i++) {
    float cp = cos(PHI[i]);
    float sp = sin(PHI[i]);

    // Project into arm radial plane
    float x_p = x * cp + y * sp;
    float y_p = -x * sp + y * cp;
    float z_p = z;

    float dx = x_p - R_EFF;
    float dy = y_p;
    float dz = z_p;

    float E = -2.0 * BICEP_L * dx;
    float F = 2.0 * BICEP_L * dz;
    float G = dx*dx + dy*dy + dz*dz + BICEP_L*BICEP_L - FOREARM_L*FOREARM_L;

    float A = G - E;
    float B = 2.0 * F;
    float C = G + E;

    float disc = B*B - 4.0 * A * C;
    if (disc < 0.0) return false; // Out of reach

    float sqrt_disc = sqrt(disc);
    float t = (-B - sqrt_disc) / (2.0 * A);
    float th_deg = 2.0 * atan(t) * 180.0 / PI;

    if (th_deg < THETA_MIN || th_deg > THETA_MAX) return false;
    out_thetas[i] = th_deg;
  }
  return true;
}

// =========================================================================
// MOTION ENGINE: MULTI-AXIS BRESENHAM SYNCHRONOUS STEPPING
// =========================================================================
void move_thetas(float t1, float t2, float t3, float feed_deg_s) {
  if (emergency_stop) return;

  long target_steps[3];
  target_steps[0] = round(t1 * STEPS_PER_DEG);
  target_steps[1] = round(t2 * STEPS_PER_DEG);
  target_steps[2] = round(t3 * STEPS_PER_DEG);

  long delta_steps[3];
  int dir[3];
  long max_delta = 0;

  for (int i = 0; i < 3; i++) {
    delta_steps[i] = target_steps[i] - cur_steps[i];
    if (delta_steps[i] >= 0) {
      dir[i] = HIGH;
    } else {
      dir[i] = LOW;
      delta_steps[i] = -delta_steps[i];
    }
    if (delta_steps[i] > max_delta) max_delta = delta_steps[i];
  }

  if (max_delta == 0) return;

  // Set Directions
  digitalWrite(DIR_PIN_1, dir[0]);
  digitalWrite(DIR_PIN_2, dir[1]);
  digitalWrite(DIR_PIN_3, dir[2]);

  // Calculate step delay based on feedrate
  float step_freq = feed_deg_s * STEPS_PER_DEG;
  if (step_freq < 100.0) step_freq = 100.0;
  if (step_freq > 12000.0) step_freq = 12000.0; // 12 kHz max step rate
  unsigned int step_delay_us = round(1000000.0 / step_freq);

  // Bresenham line stepping
  long accum[3] = { 0, 0, 0 };

  for (long step = 0; step < max_delta; step++) {
    if (emergency_stop) break;

    // Motor 1
    accum[0] += delta_steps[0];
    if (accum[0] >= max_delta) {
      accum[0] -= max_delta;
      digitalWrite(STEP_PIN_1, HIGH);
    }

    // Motor 2
    accum[1] += delta_steps[1];
    if (accum[1] >= max_delta) {
      accum[1] -= max_delta;
      digitalWrite(STEP_PIN_2, HIGH);
    }

    // Motor 3
    accum[2] += delta_steps[2];
    if (accum[2] >= max_delta) {
      accum[2] -= max_delta;
      digitalWrite(STEP_PIN_3, HIGH);
    }

    delayMicroseconds(5); // Minimum pulse width for NEMA 23 driver
    digitalWrite(STEP_PIN_1, LOW);
    digitalWrite(STEP_PIN_2, LOW);
    digitalWrite(STEP_PIN_3, LOW);

    delayMicroseconds(step_delay_us > 10 ? step_delay_us - 5 : 5);
  }

  cur_steps[0] = target_steps[0];
  cur_steps[1] = target_steps[1];
  cur_steps[2] = target_steps[2];

  cur_th[0] = t1;
  cur_th[1] = t2;
  cur_th[2] = t3;
}

// =========================================================================
// G-CODE COMMAND PARSER
// =========================================================================
void parse_command(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // M112: Emergency Stop
  if (cmd.startsWith("M112")) {
    emergency_stop = true;
    digitalWrite(ENABLE_PIN_1, HIGH); // Disable drivers
    digitalWrite(ENABLE_PIN_2, HIGH);
    digitalWrite(ENABLE_PIN_3, HIGH);
    Serial.println(F("!! EMERGENCY STOP TRIGGERED !!"));
    return;
  }

  // M999: Clear Emergency Stop Fault
  if (cmd.startsWith("M999")) {
    emergency_stop = false;
    digitalWrite(ENABLE_PIN_1, LOW); // Re-enable drivers
    digitalWrite(ENABLE_PIN_2, LOW);
    digitalWrite(ENABLE_PIN_3, LOW);
    Serial.println(F("Fault Cleared. Drivers Enabled."));
    Serial.println(F("ok"));
    return;
  }

  if (emergency_stop) {
    Serial.println(F("Error: E-Stop active! Send M999 to reset."));
    return;
  }

  // G28: Home
  if (cmd.startsWith("G28")) {
    // Return to default center position
    cur_x = 0.0; cur_y = 0.0; cur_z = -350.0;
    delta_ik(cur_x, cur_y, cur_z, cur_th);
    move_thetas(cur_th[0], cur_th[1], cur_th[2], 30.0);
    Serial.println(F("Homed to (0, 0, -350)"));
    Serial.println(F("ok"));
    return;
  }

  // M114: Report Position
  if (cmd.startsWith("M114")) {
    Serial.print(F("X:")); Serial.print(cur_x);
    Serial.print(F(" Y:")); Serial.print(cur_y);
    Serial.print(F(" Z:")); Serial.print(cur_z);
    Serial.print(F(" A:")); Serial.print(cur_th[0]);
    Serial.print(F(" B:")); Serial.print(cur_th[1]);
    Serial.print(F(" C:")); Serial.println(cur_th[2]);
    Serial.println(F("ok"));
    return;
  }

  // G0 or G1: Linear Move
  if (cmd.startsWith("G0") || cmd.startsWith("G1")) {
    float target_x = cur_x;
    float target_y = cur_y;
    float target_z = cur_z;
    float target_a = cur_th[0];
    float target_b = cur_th[1];
    float target_c = cur_th[2];
    float feedrate = 60.0; // deg/s

    bool has_cartesian = false;
    bool has_joint = false;

    // Parse X, Y, Z
    int x_idx = cmd.indexOf('X');
    if (x_idx != -1) { target_x = cmd.substring(x_idx + 1).toFloat(); has_cartesian = true; }
    int y_idx = cmd.indexOf('Y');
    if (y_idx != -1) { target_y = cmd.substring(y_idx + 1).toFloat(); has_cartesian = true; }
    int z_idx = cmd.indexOf('Z');
    if (z_idx != -1) { target_z = cmd.substring(z_idx + 1).toFloat(); has_cartesian = true; }

    // Parse A, B, C (Joint angles)
    int a_idx = cmd.indexOf('A');
    if (a_idx != -1) { target_a = cmd.substring(a_idx + 1).toFloat(); has_joint = true; }
    int b_idx = cmd.indexOf('B');
    if (b_idx != -1) { target_b = cmd.substring(b_idx + 1).toFloat(); has_joint = true; }
    int c_idx = cmd.indexOf('C');
    if (c_idx != -1) { target_c = cmd.substring(c_idx + 1).toFloat(); has_joint = true; }

    // Feedrate F (mm/min or deg/min)
    int f_idx = cmd.indexOf('F');
    if (f_idx != -1) {
      feedrate = cmd.substring(f_idx + 1).toFloat() / 60.0; // convert to per second
    }

    if (has_cartesian) {
      float next_th[3];
      if (delta_ik(target_x, target_y, target_z, next_th)) {
        move_thetas(next_th[0], next_th[1], next_th[2], feedrate);
        cur_x = target_x;
        cur_y = target_y;
        cur_z = target_z;
        Serial.println(F("ok"));
      } else {
        Serial.println(F("Error: Cartesian target outside delta kinematic envelope!"));
      }
    } else if (has_joint) {
      move_thetas(target_a, target_b, target_c, feedrate);
      Serial.println(F("ok"));
    } else {
      Serial.println(F("ok"));
    }
    return;
  }

  // M115: Firmware info
  if (cmd.startsWith("M115")) {
    Serial.println(F("FIRMWARE_NAME: Delta_Robot_NEMA23 PROTOCOL_VERSION: 1.0 MACHINE_TYPE: Delta ROBOT_L: 150 ROBOT_l: 400"));
    Serial.println(F("ok"));
    return;
  }

  Serial.println(F("ok"));
}
