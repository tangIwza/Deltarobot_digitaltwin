/**
 * Client-Side Delta Robot Kinematics (JavaScript)
 * ===============================================
 * Closed-form analytical Inverse and Forward Kinematics for 60 FPS client responsiveness.
 * Actuator: 3x NEMA 23
 * Bicep Length: 150 mm
 * Forearm Length: 400 mm
 */

class DeltaKinematicsJS {
    constructor(config = {}) {
        this.L = config.bicep_length || 150.0;     // Bicep (mm)
        this.l = config.forearm_length || 400.0;   // Forearm (mm)
        this.R = config.base_radius || 110.0;      // Base radius (mm)
        this.r = config.platform_radius || 45.0;   // Wrist radius (mm)

        this.phi_deg = config.phi_angles || [0.0, 120.0, 240.0];
        this.phi_rad = this.phi_deg.map(deg => deg * Math.PI / 180.0);

        this.theta_min = config.theta_min !== undefined ? config.theta_min : -45.0;
        this.theta_max = config.theta_max !== undefined ? config.theta_max : 85.0;
        this.z_min = config.z_min !== undefined ? config.z_min : -550.0;
        this.z_max = config.z_max !== undefined ? config.z_max : -200.0;
    }

    get R_eff() {
        return this.R - this.r;
    }

    updateConfig(newCfg) {
        if (newCfg.bicep_length !== undefined) this.L = parseFloat(newCfg.bicep_length);
        if (newCfg.forearm_length !== undefined) this.l = parseFloat(newCfg.forearm_length);
        if (newCfg.base_radius !== undefined) this.R = parseFloat(newCfg.base_radius);
        if (newCfg.platform_radius !== undefined) this.r = parseFloat(newCfg.platform_radius);
        if (newCfg.theta_min !== undefined) this.theta_min = parseFloat(newCfg.theta_min);
        if (newCfg.theta_max !== undefined) this.theta_max = parseFloat(newCfg.theta_max);
        if (newCfg.z_min !== undefined) this.z_min = parseFloat(newCfg.z_min);
        if (newCfg.z_max !== undefined) this.z_max = parseFloat(newCfg.z_max);
    }

    /**
     * Inverse Kinematics: (X, Y, Z) -> [theta1, theta2, theta3] (in degrees)
     */
    inverseKinematics(x, y, z) {
        if (z > this.z_max || z < this.z_min) {
            return { thetas: null, error: `Z (${z.toFixed(1)}) exceeds limits [${this.z_min}, ${this.z_max}]` };
        }

        const thetas = [];
        const R_eff = this.R_eff;
        const L = this.L;
        const l = this.l;

        for (let i = 0; i < 3; i++) {
            const phi = this.phi_rad[i];
            const cp = Math.cos(phi);
            const sp = Math.sin(phi);

            // Project into arm's radial plane
            const x_p = x * cp + y * sp;
            const y_p = -x * sp + y * cp;
            const z_p = z;

            const dx = x_p - R_eff;
            const dy = y_p;
            const dz = z_p;

            const E = -2.0 * L * dx;
            const F = 2.0 * L * dz;
            const G = dx * dx + dy * dy + dz * dz + L * L - l * l;

            const A = G - E;
            const B = 2.0 * F;
            const C = G + E;

            let disc = B * B - 4.0 * A * C;
            if (disc < -1e-6) {
                return { thetas: null, error: `Arm ${i + 1} unreachable at (${x.toFixed(1)}, ${y.toFixed(1)}, ${z.toFixed(1)})` };
            }
            if (disc < 0) disc = 0;

            const sqrt_disc = Math.sqrt(disc);
            let t;
            if (Math.abs(A) < 1e-9) {
                if (Math.abs(B) < 1e-9) return { thetas: null, error: "Singularity encountered" };
                t = -C / B;
            } else {
                t = (-B - sqrt_disc) / (2.0 * A);
            }

            const th_rad = 2.0 * Math.atan(t);
            const th_deg = th_rad * 180.0 / Math.PI;

            if (th_deg < this.theta_min || th_deg > this.theta_max) {
                return { thetas: null, error: `Joint ${i + 1} angle ${th_deg.toFixed(1)}° exceeds [${this.theta_min}°, ${this.theta_max}°]` };
            }

            thetas.push(Number(th_deg.toFixed(2)));
        }

        return { thetas, error: null };
    }

    /**
     * Forward Kinematics: [th1, th2, th3] -> [X, Y, Z]
     */
    forwardKinematics(th1, th2, th3) {
        const thetas = [th1, th2, th3];
        const R_eff = this.R_eff;
        const L = this.L;
        const l = this.l;

        // Centers of 3 virtual spheres
        const C = [];
        for (let i = 0; i < 3; i++) {
            const phi = this.phi_rad[i];
            const th = thetas[i] * Math.PI / 180.0;
            const r_rad = R_eff + L * Math.cos(th);
            const cx = r_rad * Math.cos(phi);
            const cy = r_rad * Math.sin(phi);
            const cz = -L * Math.sin(th);
            C.push([cx, cy, cz]);
        }

        const [p1, p2, p3] = C;

        // Vector p1->p2
        const p2_p1 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
        const d = Math.hypot(p2_p1[0], p2_p1[1], p2_p1[2]);
        if (d < 1e-6) return { pos: null, error: "Singularity: coincident sphere centers" };

        const ex = [p2_p1[0] / d, p2_p1[1] / d, p2_p1[2] / d];

        const p3_p1 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]];
        const i_val = ex[0] * p3_p1[0] + ex[1] * p3_p1[1] + ex[2] * p3_p1[2];

        const ey_raw = [p3_p1[0] - i_val * ex[0], p3_p1[1] - i_val * ex[1], p3_p1[2] - i_val * ex[2]];
        const j_norm = Math.hypot(ey_raw[0], ey_raw[1], ey_raw[2]);
        if (j_norm < 1e-6) return { pos: null, error: "Singularity: collinear arm positions" };

        const ey = [ey_raw[0] / j_norm, ey_raw[1] / j_norm, ey_raw[2] / j_norm];

        // ez = ex x ey
        const ez = [
            ex[1] * ey[2] - ex[2] * ey[1],
            ex[2] * ey[0] - ex[0] * ey[2],
            ex[0] * ey[1] - ex[1] * ey[0],
        ];

        const j_val = ey[0] * p3_p1[0] + ey[1] * p3_p1[1] + ey[2] * p3_p1[2];

        const x_tril = (d * d) / (2.0 * d);
        const y_tril = (i_val * i_val + j_val * j_val) / (2.0 * j_val) - (i_val / j_val) * x_tril;

        const z_sq = l * l - x_tril * x_tril - y_tril * y_tril;
        if (z_sq < -1e-5) return { pos: null, error: "Forearms cannot meet (outside workspace)" };

        const z_tril = Math.sqrt(Math.max(0.0, z_sq));

        const sol1 = [
            p1[0] + x_tril * ex[0] + y_tril * ey[0] + z_tril * ez[0],
            p1[1] + x_tril * ex[1] + y_tril * ey[1] + z_tril * ez[1],
            p1[2] + x_tril * ex[2] + y_tril * ey[2] + z_tril * ez[2],
        ];
        const sol2 = [
            p1[0] + x_tril * ex[0] + y_tril * ey[0] - z_tril * ez[0],
            p1[1] + x_tril * ex[1] + y_tril * ey[1] - z_tril * ez[1],
            p1[2] + x_tril * ex[2] + y_tril * ey[2] - z_tril * ez[2],
        ];

        const chosen = sol1[2] < sol2[2] ? sol1 : sol2;
        return {
            pos: [Number(chosen[0].toFixed(2)), Number(chosen[1].toFixed(2)), Number(chosen[2].toFixed(2))],
            error: null
        };
    }

    /**
     * Compute full 3D coordinates of all joints for 3D rendering
     */
    getJointPositions(thetas, ee_pos) {
        if (!ee_pos) {
            const fk = this.forwardKinematics(thetas[0], thetas[1], thetas[2]);
            ee_pos = fk.pos || [0, 0, -350];
        }

        const base_joints = [];
        const elbow_joints = [];
        const wrist_joints = [];

        for (let i = 0; i < 3; i++) {
            const phi = this.phi_rad[i];
            const th = thetas[i] * Math.PI / 180.0;
            const cp = Math.cos(phi);
            const sp = Math.sin(phi);

            // Base motor shaft
            base_joints.push([this.R * cp, this.R * sp, 0]);

            // Elbow joint
            const ex = (this.R + this.L * Math.cos(th)) * cp;
            const ey = (this.R + this.L * Math.cos(th)) * sp;
            const ez = -this.L * Math.sin(th);
            elbow_joints.push([ex, ey, ez]);

            // Wrist joint on traveling plate
            const wx = ee_pos[0] + this.r * cp;
            const wy = ee_pos[1] + this.r * sp;
            const wz = ee_pos[2];
            wrist_joints.push([wx, wy, wz]);
        }

        return {
            base_joints,
            elbow_joints,
            wrist_joints,
            end_effector: ee_pos
        };
    }
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = DeltaKinematicsJS;
}
