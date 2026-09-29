"""
Delta Robot Kinematics Engine
=============================
Pure Python mathematical implementation of 3-DOF Delta Robot kinematics:
- Inverse Kinematics (IK): Cartesian (X, Y, Z) -> Joint Angles (theta1, theta2, theta3)
- Forward Kinematics (FK): Joint Angles (theta1, theta2, theta3) -> Cartesian (X, Y, Z)
- Jacobian Matrix & Singularity Analysis
- Workspace Boundary Validation

Robot Configuration:
- Actuator: 3x NEMA 23 stepper motors mounted at 120° intervals
- Bicep length (L): 150 mm
- Forearm length (l): 400 mm
- Base radius (R): distance from robot center to motor shaft
- Platform radius (r): distance from traveling plate center to wrist ball joints
"""

import math
from typing import Tuple, List, Optional, Dict, Any

class DeltaKinematics:
    def __init__(
        self,
        bicep_length: float = 150.0,      # L (mm)
        forearm_length: float = 400.0,    # l (mm)
        base_radius: float = 110.0,       # R (mm) - shaft center from origin
        platform_radius: float = 45.0,    # r (mm) - wrist joint from platform center
        phi_angles: Optional[List[float]] = None, # Azimuth angles of arms in degrees
        theta_min_deg: float = -45.0,     # Max upward angle
        theta_max_deg: float = 85.0,      # Max downward angle
        z_min: float = -550.0,            # Max downward reach (mm)
        z_max: float = -200.0,            # Highest operating height (mm)
    ):
        self.L = float(bicep_length)
        self.l = float(forearm_length)
        self.R = float(base_radius)
        self.r = float(platform_radius)
        
        # Azimuth angles of the 3 arms (default 0°, 120°, 240°)
        if phi_angles is None:
            self.phi_deg = [0.0, 120.0, 240.0]
        else:
            self.phi_deg = [float(a) for a in phi_angles]
        self.phi_rad = [math.radians(a) for a in self.phi_deg]
        
        # Joint limits (degrees)
        self.theta_min = theta_min_deg
        self.theta_max = theta_max_deg
        self.z_min = z_min
        self.z_max = z_max

    @property
    def R_eff(self) -> float:
        """Effective base radius R - r."""
        return self.R - self.r

    def get_config(self) -> Dict[str, Any]:
        """Returns the dictionary representation of kinematic parameters."""
        return {
            "bicep_length": self.L,
            "forearm_length": self.l,
            "base_radius": self.R,
            "platform_radius": self.r,
            "phi_angles": self.phi_deg,
            "theta_min": self.theta_min,
            "theta_max": self.theta_max,
            "z_min": self.z_min,
            "z_max": self.z_max
        }

    def update_config(self, **kwargs) -> None:
        """Update kinematic parameters dynamically."""
        if "bicep_length" in kwargs:
            self.L = float(kwargs["bicep_length"])
        if "forearm_length" in kwargs:
            self.l = float(kwargs["forearm_length"])
        if "base_radius" in kwargs:
            self.R = float(kwargs["base_radius"])
        if "platform_radius" in kwargs:
            self.r = float(kwargs["platform_radius"])
        if "phi_angles" in kwargs:
            self.phi_deg = [float(a) for a in kwargs["phi_angles"]]
            self.phi_rad = [math.radians(a) for a in self.phi_deg]
        if "theta_min" in kwargs:
            self.theta_min = float(kwargs["theta_min"])
        if "theta_max" in kwargs:
            self.theta_max = float(kwargs["theta_max"])
        if "z_min" in kwargs:
            self.z_min = float(kwargs["z_min"])
        if "z_max" in kwargs:
            self.z_max = float(kwargs["z_max"])

    def inverse_kinematics(self, x: float, y: float, z: float) -> Tuple[Optional[List[float]], str]:
        """
        Calculate Joint Angles (theta1, theta2, theta3 in degrees) for a target (X, Y, Z in mm).
        Returns:
            ( [theta1_deg, theta2_deg, theta3_deg], status_message )
            If unreachable or singular, returns (None, error_msg).
        """
        if z > self.z_max or z < self.z_min:
            return None, f"Target Z ({z:.1f}mm) out of vertical range [{self.z_min:.0f}, {self.z_max:.0f}]"

        thetas_deg = []
        R_eff = self.R_eff
        L = self.L
        l = self.l

        for i in range(3):
            phi = self.phi_rad[i]
            cp = math.cos(phi)
            sp = math.sin(phi)

            # Project (x, y, z) into the arm's local 2D radial plane
            # Local X is radial outward, Local Y is perpendicular, Local Z is vertical
            x_proj = x * cp + y * sp
            y_proj = -x * sp + y * cp
            z_proj = z

            # Relative offset from effective base motor center
            dx = x_proj - R_eff
            dy = y_proj
            dz = z_proj

            # Quadratic equation in t = tan(theta / 2):
            # E * cos(theta) + F * sin(theta) + G = 0
            E = -2.0 * L * dx
            F = 2.0 * L * dz
            G = dx * dx + dy * dy + dz * dz + L * L - l * l

            A = G - E
            B = 2.0 * F
            C = G + E

            discriminant = B * B - 4.0 * A * C
            if discriminant < -1e-7:
                return None, f"Arm {i+1} cannot reach position ({x:.1f}, {y:.1f}, {z:.1f}) - target beyond arm length"

            if discriminant < 0:
                discriminant = 0.0

            sqrt_disc = math.sqrt(discriminant)

            # Two roots: t1 = (-B - sqrt_disc) / (2A), t2 = (-B + sqrt_disc) / (2A)
            # For standard delta configuration (elbow pointing outwards), t1 gives physical angle
            if abs(A) < 1e-9:
                if abs(B) < 1e-9:
                    return None, f"Arm {i+1} kinematic singularity encountered"
                t = -C / B
            else:
                t = (-B - sqrt_disc) / (2.0 * A)

            theta_rad = 2.0 * math.atan(t)
            theta_deg = math.degrees(theta_rad)

            # Validate joint limits
            if theta_deg < self.theta_min or theta_deg > self.theta_max:
                return None, f"Arm {i+1} angle ({theta_deg:.1f}°) exceeds limits [{self.theta_min}°, {self.theta_max}°]"

            thetas_deg.append(round(theta_deg, 3))

        return thetas_deg, "OK"

    def forward_kinematics(self, th1_deg: float, th2_deg: float, th3_deg: float) -> Tuple[Optional[List[float]], str]:
        """
        Calculate Cartesian coordinates (X, Y, Z in mm) given Joint Angles (theta1, theta2, theta3 in degrees).
        Solves the intersection of 3 spheres of radius l = 400mm.
        Returns:
            ( [X, Y, Z], status_message )
        """
        thetas_deg = [th1_deg, th2_deg, th3_deg]
        for i, th in enumerate(thetas_deg):
            if th < self.theta_min - 5.0 or th > self.theta_max + 5.0:
                return None, f"Joint {i+1} angle {th:.1f}° is out of valid bounds"

        R_eff = self.R_eff
        L = self.L
        l = self.l

        # Compute sphere centers C_i for each arm
        # C_i is the virtual center of the sphere of radius l = forearm
        C = []
        for i in range(3):
            phi = self.phi_rad[i]
            th = math.radians(thetas_deg[i])
            r_rad = R_eff + L * math.cos(th)
            cx = r_rad * math.cos(phi)
            cy = r_rad * math.sin(phi)
            cz = -L * math.sin(th)
            C.append([cx, cy, cz])

        p1, p2, p3 = C[0], C[1], C[2]

        # Trilateration: Sphere 1 (p1), Sphere 2 (p2), Sphere 3 (p3), all radius l
        # Vector from p1 to p2
        p2_p1 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]
        d = math.sqrt(p2_p1[0]**2 + p2_p1[1]**2 + p2_p1[2]**2)
        if d < 1e-6:
            return None, "Singularity: Arm 1 and Arm 2 centers coincide"

        # Unit vector ex
        ex = [p2_p1[0] / d, p2_p1[1] / d, p2_p1[2] / d]

        # Vector from p1 to p3
        p3_p1 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]]
        i_val = ex[0] * p3_p1[0] + ex[1] * p3_p1[1] + ex[2] * p3_p1[2]

        # Unit vector ey
        ey_temp = [p3_p1[0] - i_val * ex[0], p3_p1[1] - i_val * ex[1], p3_p1[2] - i_val * ex[2]]
        j_norm = math.sqrt(ey_temp[0]**2 + ey_temp[1]**2 + ey_temp[2]**2)
        if j_norm < 1e-6:
            return None, "Singularity: All 3 arm centers are collinear"

        ey = [ey_temp[0] / j_norm, ey_temp[1] / j_norm, ey_temp[2] / j_norm]

        # Unit vector ez = ex x ey
        ez = [
            ex[1] * ey[2] - ex[2] * ey[1],
            ex[2] * ey[0] - ex[0] * ey[2],
            ex[0] * ey[1] - ex[1] * ey[0],
        ]

        j_val = ey[0] * p3_p1[0] + ey[1] * p3_p1[1] + ey[2] * p3_p1[2]

        # Coordinates in the trilateration frame
        x_tril = (d * d) / (2.0 * d)
        y_tril = (i_val * i_val + j_val * j_val) / (2.0 * j_val) - (i_val / j_val) * x_tril

        z_sq = l * l - x_tril * x_tril - y_tril * y_tril
        if z_sq < -1e-5:
            return None, "Geometric condition not met: Forearms do not intersect"

        z_tril = math.sqrt(max(0.0, z_sq))

        # Two possible intersection points: p1 + x*ex + y*ey +/- z*ez
        sol1 = [
            p1[k] + x_tril * ex[k] + y_tril * ey[k] + z_tril * ez[k] for k in range(3)
        ]
        sol2 = [
            p1[k] + x_tril * ex[k] + y_tril * ey[k] - z_tril * ez[k] for k in range(3)
        ]

        # Physical solution has negative Z (pointing downward from base)
        chosen = sol1 if sol1[2] < sol2[2] else sol2
        return [round(chosen[0], 3), round(chosen[1], 3), round(chosen[2], 3)], "OK"

    def get_joint_positions(self, thetas_deg: List[float], ee_pos: Optional[List[float]] = None) -> Dict[str, Any]:
        """
        Returns full 3D coordinates for all joints (base motors, bicep elbows, wrist mounts, and end effector)
        for real-time 3D visualization.
        """
        if ee_pos is None:
            ee_pos, _ = self.forward_kinematics(*thetas_deg)
            if ee_pos is None:
                ee_pos = [0.0, 0.0, -350.0]

        joints = {
            "base_joints": [],
            "elbow_joints": [],
            "wrist_joints": [],
            "end_effector": ee_pos
        }

        for i in range(3):
            phi = self.phi_rad[i]
            th = math.radians(thetas_deg[i])
            cp = math.cos(phi)
            sp = math.sin(phi)

            # Motor base joint (on base plate)
            bx = self.R * cp
            by = self.R * sp
            bz = 0.0
            joints["base_joints"].append([round(bx, 2), round(by, 2), round(bz, 2)])

            # Elbow joint (end of bicep)
            ex = (self.R + self.L * math.cos(th)) * cp
            ey = (self.R + self.L * math.cos(th)) * sp
            ez = -self.L * math.sin(th)
            joints["elbow_joints"].append([round(ex, 2), round(ey, 2), round(ez, 2)])

            # Wrist joint (on traveling plate)
            wx = ee_pos[0] + self.r * cp
            wy = ee_pos[1] + self.r * sp
            wz = ee_pos[2]
            joints["wrist_joints"].append([round(wx, 2), round(wy, 2), round(wz, 2)])

        return joints

    def generate_workspace_cloud(self, resolution_xy: int = 15, resolution_z: int = 15) -> List[List[float]]:
        """
        Generates a 3D point cloud of valid reachable positions within the workspace.
        Used for rendering the workspace envelope in 3D viewer.
        """
        points = []
        r_max = 240.0
        z_start = -250.0
        z_end = -500.0

        z_steps = [z_start + i * (z_end - z_start) / (resolution_z - 1) for i in range(resolution_z)]
        for z in z_steps:
            for i in range(resolution_xy):
                angle = 2.0 * math.pi * i / resolution_xy
                for r in [r_max * 0.25, r_max * 0.5, r_max * 0.75, r_max]:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    thetas, status = self.inverse_kinematics(x, y, z)
                    if thetas is not None:
                        points.append([round(x, 1), round(y, 1), round(z, 1)])
        return points
