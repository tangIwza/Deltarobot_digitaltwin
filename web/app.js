/**
 * Delta Robot UI Application Logic
 * =================================
 * Coordinates Cartesian Jogging, Theta Jogging, 3D Digital Twin,
 * Kinematics calculations, and Hardware Communications.
 */

document.addEventListener('DOMContentLoaded', () => {
    // 1. Initialize Kinematics (Bicep 150mm, Forearm 400mm)
    const kinematics = new DeltaKinematicsJS({
        bicep_length: 150.0,
        forearm_length: 400.0,
        base_radius: 110.0,
        platform_radius: 45.0,
        theta_min: -45.0,
        theta_max: 85.0,
        z_min: -550.0,
        z_max: -200.0
    });

    // 2. Initialize 3D Visualizer
    const robot3d = new DeltaRobot3D('viewport3d', kinematics);

    // 3. State
    let cartesianPos = [0.0, 0.0, -350.0];
    let jointThetas = [4.73, 4.73, 4.73];
    let cartesianStep = 10.0; // mm
    let thetaStep = 5.0;      // deg
    let feedrate = 120.0;     // mm/s
    let activeMode = 'cartesian'; // 'cartesian' | 'theta'
    let isEmergencyStopped = false;

    // 4. Initialize Hardware Communicator
    const comms = new HardwareCommunicator(
        (telemetry) => handleTelemetryUpdate(telemetry),
        (dir, text) => logConsoleMessage(dir, text)
    );

    // Initial 3D update
    robot3d.updatePose(jointThetas, cartesianPos);
    updateTelemetryDisplay();

    // =========================================================================
    // Telemetry Sync
    // =========================================================================

    function handleTelemetryUpdate(state) {
        if (!state) return;

        if (state.cartesian) {
            cartesianPos = [state.cartesian.x, state.cartesian.y, state.cartesian.z];
        }
        if (state.thetas) {
            jointThetas = [state.thetas.theta1, state.thetas.theta2, state.thetas.theta3];
        }

        // Update 3D Model
        robot3d.updatePose(jointThetas, cartesianPos);
        updateTelemetryDisplay();

        // Update Status Badges
        if (state.status) {
            const badge = document.getElementById('connectionBadge');
            if (state.status.emergency_stopped) {
                isEmergencyStopped = true;
                badge.textContent = "EMERGENCY STOP";
                badge.className = "status-badge badge-danger";
            } else if (state.status.is_connected && !state.status.is_simulation) {
                badge.textContent = "HARDWARE ONLINE";
                badge.className = "status-badge badge-success";
            } else {
                badge.textContent = "SIMULATOR ACTIVE";
                badge.className = "status-badge badge-info";
            }
        }
    }

    function updateTelemetryDisplay() {
        // Cartesian coordinates
        document.getElementById('dispX').textContent = cartesianPos[0].toFixed(2);
        document.getElementById('dispY').textContent = cartesianPos[1].toFixed(2);
        document.getElementById('dispZ').textContent = cartesianPos[2].toFixed(2);

        document.getElementById('inputX').value = cartesianPos[0].toFixed(1);
        document.getElementById('inputY').value = cartesianPos[1].toFixed(1);
        document.getElementById('inputZ').value = cartesianPos[2].toFixed(1);

        // Theta angles
        document.getElementById('dispTh1').textContent = jointThetas[0].toFixed(2) + "°";
        document.getElementById('dispTh2').textContent = jointThetas[1].toFixed(2) + "°";
        document.getElementById('dispTh3').textContent = jointThetas[2].toFixed(2) + "°";

        document.getElementById('sliderTh1').value = jointThetas[0];
        document.getElementById('sliderTh2').value = jointThetas[1];
        document.getElementById('sliderTh3').value = jointThetas[2];

        document.getElementById('valTh1').textContent = jointThetas[0].toFixed(1) + "°";
        document.getElementById('valTh2').textContent = jointThetas[1].toFixed(1) + "°";
        document.getElementById('valTh3').textContent = jointThetas[2].toFixed(1) + "°";
    }

    // =========================================================================
    // Mode Switcher (Cartesian vs Theta Jogging)
    // =========================================================================

    const btnTabCartesian = document.getElementById('tabCartesian');
    const btnTabTheta = document.getElementById('tabTheta');
    const panelCartesian = document.getElementById('panelCartesian');
    const panelTheta = document.getElementById('panelTheta');

    btnTabCartesian.addEventListener('click', () => {
        activeMode = 'cartesian';
        btnTabCartesian.classList.add('active');
        btnTabTheta.classList.remove('active');
        panelCartesian.classList.remove('hidden');
        panelTheta.classList.add('hidden');
    });

    btnTabTheta.addEventListener('click', () => {
        activeMode = 'theta';
        btnTabTheta.classList.add('active');
        btnTabCartesian.classList.remove('active');
        panelTheta.classList.remove('hidden');
        panelCartesian.classList.add('hidden');
    });

    // =========================================================================
    // Cartesian Jogging Controls
    // =========================================================================

    // Step buttons
    document.querySelectorAll('.cart-step-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.cart-step-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            cartesianStep = parseFloat(btn.dataset.step);
        });
    });

    // Feedrate Slider
    const feedSlider = document.getElementById('feedrateSlider');
    const feedDisplay = document.getElementById('feedrateVal');
    feedSlider.addEventListener('input', (e) => {
        feedrate = parseFloat(e.target.value);
        feedDisplay.textContent = feedrate + " mm/s";
    });

    // Jog Cartesian Action
    function executeCartesianJog(axis, direction) {
        if (isEmergencyStopped) {
            logConsoleMessage('ALERT', 'Robot in Emergency Stop! Click Reset first.');
            return;
        }

        const step = cartesianStep * direction;
        const targetX = cartesianPos[0] + (axis === 'X' ? step : 0);
        const targetY = cartesianPos[1] + (axis === 'Y' ? step : 0);
        const targetZ = cartesianPos[2] + (axis === 'Z' ? step : 0);

        // Check Kinematics locally for instant response & limit validation
        const ik = kinematics.inverseKinematics(targetX, targetY, targetZ);
        if (ik.error) {
            logConsoleMessage('WARN', ik.error);
            showLimitWarning(ik.error);
            return;
        }

        // Apply new position
        cartesianPos = [targetX, targetY, targetZ];
        jointThetas = ik.thetas;

        robot3d.updatePose(jointThetas, cartesianPos);
        updateTelemetryDisplay();

        // Dispatch G-code
        const gcode = `G1 X${targetX.toFixed(2)} Y${targetY.toFixed(2)} Z${targetZ.toFixed(2)} F${Math.round(feedrate * 60)}`;
        comms.sendGcode(gcode);

        // Also notify backend if WebSocket is active
        comms.sendWsAction('jog_cartesian', { axis, step, feedrate });
    }

    // Direction buttons
    document.getElementById('jogXPlus').addEventListener('click', () => executeCartesianJog('X', 1));
    document.getElementById('jogXMinus').addEventListener('click', () => executeCartesianJog('X', -1));
    document.getElementById('jogYPlus').addEventListener('click', () => executeCartesianJog('Y', 1));
    document.getElementById('jogYMinus').addEventListener('click', () => executeCartesianJog('Y', -1));
    document.getElementById('jogZPlus').addEventListener('click', () => executeCartesianJog('Z', 1));
    document.getElementById('jogZMinus').addEventListener('click', () => executeCartesianJog('Z', -1));

    // Direct Cartesian Go button
    document.getElementById('btnGoCartesian').addEventListener('click', () => {
        const x = parseFloat(document.getElementById('inputX').value) || 0;
        const y = parseFloat(document.getElementById('inputY').value) || 0;
        const z = parseFloat(document.getElementById('inputZ').value) || -350;

        const ik = kinematics.inverseKinematics(x, y, z);
        if (ik.error) {
            logConsoleMessage('WARN', ik.error);
            showLimitWarning(ik.error);
            return;
        }

        cartesianPos = [x, y, z];
        jointThetas = ik.thetas;
        robot3d.updatePose(jointThetas, cartesianPos);
        updateTelemetryDisplay();

        const gcode = `G1 X${x.toFixed(2)} Y${y.toFixed(2)} Z${z.toFixed(2)} F${Math.round(feedrate * 60)}`;
        comms.sendGcode(gcode);
        comms.sendWsAction('move_cartesian', { x, y, z, feedrate });
    });

    // =========================================================================
    // Theta / Joint Jogging Controls
    // =========================================================================

    // Theta Step buttons
    document.querySelectorAll('.theta-step-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.theta-step-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            thetaStep = parseFloat(btn.dataset.step);
        });
    });

    function executeThetaJog(jointIdx, direction) {
        if (isEmergencyStopped) {
            logConsoleMessage('ALERT', 'Robot in Emergency Stop! Click Reset first.');
            return;
        }

        const newThetas = [...jointThetas];
        newThetas[jointIdx] += thetaStep * direction;
        newThetas[jointIdx] = Math.round(newThetas[jointIdx] * 100) / 100;

        // Check Forward Kinematics
        const fk = kinematics.forwardKinematics(newThetas[0], newThetas[1], newThetas[2]);
        if (fk.error) {
            logConsoleMessage('WARN', fk.error);
            showLimitWarning(fk.error);
            return;
        }

        jointThetas = newThetas;
        cartesianPos = fk.pos;

        robot3d.updatePose(jointThetas, cartesianPos);
        updateTelemetryDisplay();

        const gcode = `G1 A${jointThetas[0].toFixed(2)} B${jointThetas[1].toFixed(2)} C${jointThetas[2].toFixed(2)} F${Math.round(feedrate * 60)}`;
        comms.sendGcode(gcode);
        comms.sendWsAction('jog_theta', { joint: jointIdx, step: thetaStep * direction, feedrate: 45 });
    }

    // Joint minus/plus buttons
    document.getElementById('th1Minus').addEventListener('click', () => executeThetaJog(0, -1));
    document.getElementById('th1Plus').addEventListener('click', () => executeThetaJog(0, 1));
    document.getElementById('th2Minus').addEventListener('click', () => executeThetaJog(1, -1));
    document.getElementById('th2Plus').addEventListener('click', () => executeThetaJog(1, 1));
    document.getElementById('th3Minus').addEventListener('click', () => executeThetaJog(2, -1));
    document.getElementById('th3Plus').addEventListener('click', () => executeThetaJog(2, 1));

    // Sliders
    [0, 1, 2].forEach(idx => {
        const slider = document.getElementById(`sliderTh${idx + 1}`);
        slider.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            const newThetas = [...jointThetas];
            newThetas[idx] = val;

            const fk = kinematics.forwardKinematics(newThetas[0], newThetas[1], newThetas[2]);
            if (fk.pos) {
                jointThetas = newThetas;
                cartesianPos = fk.pos;
                robot3d.updatePose(jointThetas, cartesianPos);
                updateTelemetryDisplay();
            }
        });
        slider.addEventListener('change', () => {
            const gcode = `G1 A${jointThetas[0].toFixed(2)} B${jointThetas[1].toFixed(2)} C${jointThetas[2].toFixed(2)} F${Math.round(feedrate * 60)}`;
            comms.sendGcode(gcode);
            comms.sendWsAction('move_thetas', { theta1: jointThetas[0], theta2: jointThetas[1], theta3: jointThetas[2] });
        });
    });

    // =========================================================================
    // Preset Trajectories & Actions
    // =========================================================================

    // Home
    document.getElementById('btnHome').addEventListener('click', () => {
        isEmergencyStopped = false;
        cartesianPos = [0.0, 0.0, -350.0];
        const ik = kinematics.inverseKinematics(0, 0, -350);
        if (ik.thetas) jointThetas = ik.thetas;

        robot3d.updatePose(jointThetas, cartesianPos);
        updateTelemetryDisplay();

        comms.sendGcode("G28");
        comms.sendWsAction('home');
        logConsoleMessage('SYSTEM', 'Homed to center reference position (0, 0, -350)');
    });

    // Park
    document.getElementById('btnPark').addEventListener('click', () => {
        cartesianPos = [0.0, 0.0, -320.0];
        const ik = kinematics.inverseKinematics(0, 0, -320);
        if (ik.thetas) jointThetas = ik.thetas;
        robot3d.updatePose(jointThetas, cartesianPos);
        updateTelemetryDisplay();
        comms.sendGcode("G1 X0 Y0 Z-320 F3000");
    });

    // Circle Trajectory Demo
    document.getElementById('btnCircleDemo').addEventListener('click', () => {
        logConsoleMessage('SYSTEM', 'Starting Circular Trajectory demonstration...');
        if (comms.ws && comms.ws.readyState === WebSocket.OPEN) {
            comms.sendWsAction('run_trajectory', { type: 'circle', params: { radius: 70, z: -360, points: 40 } });
        } else {
            // Local client-side animated circle
            runLocalCircleTrajectory(70, -360, 40);
        }
    });

    function runLocalCircleTrajectory(radius, z, count) {
        let i = 0;
        const interval = setInterval(() => {
            if (i > count || isEmergencyStopped) {
                clearInterval(interval);
                return;
            }
            const angle = 2 * Math.PI * i / count;
            const x = radius * Math.cos(angle);
            const y = radius * Math.sin(angle);
            const ik = kinematics.inverseKinematics(x, y, z);
            if (ik.thetas) {
                cartesianPos = [x, y, z];
                jointThetas = ik.thetas;
                robot3d.updatePose(jointThetas, cartesianPos);
                updateTelemetryDisplay();
                comms.sendGcode(`G1 X${x.toFixed(1)} Y${y.toFixed(1)} Z${z.toFixed(1)} F6000`);
            }
            i++;
        }, 50);
    }

    // Pick & Place Demo
    document.getElementById('btnPickPlaceDemo').addEventListener('click', () => {
        logConsoleMessage('SYSTEM', 'Starting Automated Pick & Place routine...');
        if (comms.ws && comms.ws.readyState === WebSocket.OPEN) {
            comms.sendWsAction('run_trajectory', { type: 'pick_and_place' });
        } else {
            runLocalPickAndPlace();
        }
    });

    function runLocalPickAndPlace() {
        const waypoints = [
            [0, 0, -320],
            [-70, -50, -320],
            [-70, -50, -420],
            [-70, -50, -320],
            [70, 50, -320],
            [70, 50, -420],
            [70, 50, -320],
            [0, 0, -350]
        ];
        let idx = 0;
        const step = () => {
            if (idx >= waypoints.length || isEmergencyStopped) return;
            const [x, y, z] = waypoints[idx];
            const ik = kinematics.inverseKinematics(x, y, z);
            if (ik.thetas) {
                cartesianPos = [x, y, z];
                jointThetas = ik.thetas;
                robot3d.updatePose(jointThetas, cartesianPos);
                updateTelemetryDisplay();
                comms.sendGcode(`G1 X${x} Y${y} Z${z} F4500`);
            }
            idx++;
            setTimeout(step, 400);
        };
        step();
    }

    // =========================================================================
    // Emergency Stop (E-STOP)
    // =========================================================================

    const btnEstop = document.getElementById('btnEstop');
    const btnResetEstop = document.getElementById('btnResetEstop');

    btnEstop.addEventListener('click', () => {
        isEmergencyStopped = true;
        comms.sendGcode("M112");
        comms.sendWsAction('estop');
        logConsoleMessage('ALERT', 'EMERGENCY STOP TRIGGERED!');
        btnEstop.classList.add('estop-active');
        showLimitWarning("EMERGENCY STOP ACTIVE");
    });

    btnResetEstop.addEventListener('click', () => {
        isEmergencyStopped = false;
        comms.sendGcode("M999");
        comms.sendWsAction('reset_estop');
        logConsoleMessage('SYSTEM', 'Emergency Stop Cleared.');
        btnEstop.classList.remove('estop-active');
        hideLimitWarning();
    });

    // =========================================================================
    // Keyboard Hotkeys for Jogging
    // =========================================================================

    window.addEventListener('keydown', (e) => {
        // Ignore if typing in text input
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

        if (e.key === ' ' || e.code === 'Space') {
            e.preventDefault();
            btnEstop.click();
            return;
        }

        if (activeMode === 'cartesian') {
            if (e.key === 'ArrowRight') executeCartesianJog('X', 1);
            else if (e.key === 'ArrowLeft') executeCartesianJog('X', -1);
            else if (e.key === 'ArrowUp') executeCartesianJog('Y', 1);
            else if (e.key === 'ArrowDown') executeCartesianJog('Y', -1);
            else if (e.key === 'PageUp') executeCartesianJog('Z', 1);
            else if (e.key === 'PageDown') executeCartesianJog('Z', -1);
        } else {
            if (e.key === '1') executeThetaJog(0, 1);
            else if (e.key === 'q' || e.key === 'Q') executeThetaJog(0, -1);
            else if (e.key === '2') executeThetaJog(1, 1);
            else if (e.key === 'w' || e.key === 'W') executeThetaJog(1, -1);
            else if (e.key === '3') executeThetaJog(2, 1);
            else if (e.key === 'e' || e.key === 'E') executeThetaJog(2, -1);
        }
    });

    // =========================================================================
    // 3D Camera Controls & View Toggles
    // =========================================================================

    document.querySelectorAll('.view-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const view = btn.dataset.view;
            robot3d.setCameraView(view);
        });
    });

    document.getElementById('toggleWorkspace').addEventListener('change', (e) => {
        robot3d.toggleWorkspace(e.target.checked);
    });

    document.getElementById('toggleTrail').addEventListener('change', (e) => {
        robot3d.toggleTrail(e.target.checked);
    });

    document.getElementById('btnClearTrail').addEventListener('click', () => {
        robot3d.clearTrail();
    });

    // =========================================================================
    // Hardware Connection Controls
    // =========================================================================

    const btnConnect = document.getElementById('btnConnect');
    const selectPort = document.getElementById('selectPort');
    const selectBaud = document.getElementById('selectBaud');

    // Populate serial ports from backend if available
    fetch('/api/serial/ports')
        .then(res => res.json())
        .then(data => {
            selectPort.innerHTML = '';
            data.ports.forEach(p => {
                const opt = document.createElement('option');
                opt.value = p.port;
                opt.textContent = p.desc;
                selectPort.appendChild(opt);
            });
            // Also add WebSerial option
            const webOpt = document.createElement('option');
            webOpt.value = 'WEBSERIAL_DIRECT';
            webOpt.textContent = '⚡ WebSerial USB (Direct Browser)';
            selectPort.appendChild(webOpt);
        })
        .catch(() => {
            selectPort.innerHTML = '<option value="WEBSERIAL_DIRECT">⚡ WebSerial USB (Direct Browser)</option><option value="SIMULATION">Virtual Simulator Mode</option>';
        });

    btnConnect.addEventListener('click', async () => {
        const port = selectPort.value;
        const baud = parseInt(selectBaud.value);

        if (comms.isConnected) {
            await comms.disconnect();
            btnConnect.textContent = "Connect";
            btnConnect.classList.remove('btn-danger');
            btnConnect.classList.add('btn-primary');
            return;
        }

        if (port === 'WEBSERIAL_DIRECT') {
            const ok = await comms.connectWebSerial(baud);
            if (ok) {
                btnConnect.textContent = "Disconnect";
                btnConnect.classList.add('btn-danger');
            }
        } else {
            // Use backend PySerial
            fetch('/api/serial/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ port, baudrate: baud })
            })
            .then(res => res.json())
            .then(res => {
                logConsoleMessage('SYSTEM', res.message);
                if (res.connected) {
                    btnConnect.textContent = "Disconnect";
                    btnConnect.classList.add('btn-danger');
                }
            })
            .catch(err => {
                logConsoleMessage('ERROR', `Connection failed: ${err.message}`);
            });
        }
    });

    // =========================================================================
    // G-Code Terminal Console
    // =========================================================================

    const inputGcode = document.getElementById('inputGcode');
    const btnSendGcode = document.getElementById('btnSendGcode');
    const consoleOutput = document.getElementById('consoleOutput');

    function sendManualGcode() {
        const cmd = inputGcode.value.trim();
        if (!cmd) return;
        comms.sendGcode(cmd);
        inputGcode.value = '';
    }

    btnSendGcode.addEventListener('click', sendManualGcode);
    inputGcode.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') sendManualGcode();
    });

    function logConsoleMessage(type, message) {
        const line = document.createElement('div');
        line.className = `log-line log-${type.toLowerCase()}`;
        const time = new Date().toLocaleTimeString();
        line.innerHTML = `<span class="log-time">[${time}]</span> <span class="log-tag tag-${type.toLowerCase()}">${type}</span> ${message}`;
        consoleOutput.appendChild(line);
        consoleOutput.scrollTop = consoleOutput.scrollHeight;
    }

    function showLimitWarning(msg) {
        const warnBox = document.getElementById('limitWarning');
        warnBox.textContent = `⚠️ ${msg}`;
        warnBox.classList.remove('hidden');
        setTimeout(() => hideLimitWarning(), 3500);
    }

    function hideLimitWarning() {
        const warnBox = document.getElementById('limitWarning');
        warnBox.classList.add('hidden');
    }

    // =========================================================================
    // Configuration Modal
    // =========================================================================

    const btnOpenSettings = document.getElementById('btnOpenSettings');
    const modalSettings = document.getElementById('modalSettings');
    const btnCloseSettings = document.getElementById('btnCloseSettings');
    const btnSaveSettings = document.getElementById('btnSaveSettings');

    btnOpenSettings.addEventListener('click', () => {
        modalSettings.classList.remove('hidden');
        document.getElementById('cfgBicep').value = kinematics.L;
        document.getElementById('cfgForearm').value = kinematics.l;
        document.getElementById('cfgBaseR').value = kinematics.R;
        document.getElementById('cfgPlatR').value = kinematics.r;
    });

    btnCloseSettings.addEventListener('click', () => {
        modalSettings.classList.add('hidden');
    });

    btnSaveSettings.addEventListener('click', () => {
        const L = parseFloat(document.getElementById('cfgBicep').value);
        const l = parseFloat(document.getElementById('cfgForearm').value);
        const R = parseFloat(document.getElementById('cfgBaseR').value);
        const r = parseFloat(document.getElementById('cfgPlatR').value);

        kinematics.updateConfig({ bicep_length: L, forearm_length: l, base_radius: R, platform_radius: r });

        // Update backend
        fetch('/api/config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ bicep_length: L, forearm_length: l, base_radius: R, platform_radius: r })
        }).catch(() => {});

        modalSettings.classList.add('hidden');
        logConsoleMessage('SYSTEM', `Updated Kinematics: Bicep=${L}mm, Forearm=${l}mm, BaseR=${R}mm, PlatR=${r}mm`);
    });
});
