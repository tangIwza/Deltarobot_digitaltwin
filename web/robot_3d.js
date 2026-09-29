/**
 * 3D Delta Robot Visualizer (Three.js)
 * =====================================
 * High-fidelity 3D digital twin rendering the physical Delta Robot:
 * - 4-post 2020 V-slot aluminum frame
 * - Base mounting top plate with cutouts
 * - 3x NEMA 23 stepper motors (57BYG250C)
 * - 3x Bicep arms (150 mm)
 * - 3x Forearm parallel carbon rod pairs (400 mm)
 * - Wrist platform & Tool tip
 * - Workspace envelope & dynamic motion trail
 */

class DeltaRobot3D {
    constructor(containerId, kinematics) {
        this.container = document.getElementById(containerId);
        this.kinematics = kinematics;

        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;

        // Visual components
        this.motors = [];
        this.biceps = [];
        this.forearms = []; // pairs of rods
        this.wristPlate = null;
        this.tcpMarker = null;
        this.workspaceMesh = null;
        this.trailLine = null;
        this.trailPoints = [];
        this.maxTrailPoints = 120;

        // Settings
        this.showWorkspace = true;
        this.showTrail = true;
        this.rodPairsSpacing = 45.0; // mm between parallel forearm rods

        this.init();
    }

    init() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;

        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0c1017);
        this.scene.fog = new THREE.FogExp2(0x0c1017, 0.0008);

        // Camera
        this.camera = new THREE.PerspectiveCamera(45, width / height, 10, 3000);
        this.camera.position.set(450, -550, 200); // Three.js Y is UP, but in Delta Z is down.
        // Let's adopt standard convention:
        // World: X right, Y forward, Z up.
        // In robot coordinates: robot X -> World X, robot Y -> World Y, robot Z -> World Z (Z=0 is base plate, negative Z is downward).
        this.camera.up.set(0, 0, 1);
        this.camera.position.set(450, -450, 150);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.1;
        this.container.appendChild(this.renderer.domElement);

        // OrbitControls
        if (typeof THREE.OrbitControls !== 'undefined') {
            this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
            this.controls.enableDamping = true;
            this.controls.dampingFactor = 0.08;
            this.controls.target.set(0, 0, -320);
            this.controls.maxDistance = 1500;
            this.controls.minDistance = 100;
        }

        // Lighting
        this.setupLighting();

        // Environment & Frame
        this.buildGroundAndGrid();
        this.buildAluminumFrame();
        this.buildBasePlate();

        // Robot dynamic parts
        this.buildMotorsAndBiceps();
        this.buildWristAndForearms();
        this.buildTrajectoryTrail();
        this.buildWorkspaceEnvelope();

        // Resize handler
        window.addEventListener('resize', () => this.onResize());

        // Initial pose at (0, 0, -350)
        this.updatePose([4.73, 4.73, 4.73], [0, 0, -350]);

        // Start animation loop
        this.animate();
    }

    setupLighting() {
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
        this.scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
        dirLight.position.set(300, -200, 500);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 2048;
        dirLight.shadow.mapSize.height = 2048;
        dirLight.shadow.camera.near = 50;
        dirLight.shadow.camera.far = 1500;
        const d = 400;
        dirLight.shadow.camera.left = -d;
        dirLight.shadow.camera.right = d;
        dirLight.shadow.camera.top = d;
        dirLight.shadow.camera.bottom = -d;
        dirLight.shadow.bias = -0.0005;
        this.scene.add(dirLight);

        // Secondary cyan rim light
        const rimLight = new THREE.DirectionalLight(0x00f2fe, 0.7);
        rimLight.position.set(-300, 300, 200);
        this.scene.add(rimLight);

        // Warm fill from below
        const fillLight = new THREE.DirectionalLight(0x4facfe, 0.4);
        fillLight.position.set(0, -300, -500);
        this.scene.add(fillLight);
    }

    buildGroundAndGrid() {
        // Grid floor at bottom bed (Z = -600 mm)
        const gridHelper = new THREE.GridHelper(800, 40, 0x00f2fe, 0x1e293b);
        gridHelper.rotation.x = Math.PI / 2;
        gridHelper.position.z = -600;
        this.scene.add(gridHelper);

        // Soft floor plane
        const floorGeo = new THREE.PlaneGeometry(1200, 1200);
        const floorMat = new THREE.MeshStandardMaterial({
            color: 0x070a10,
            roughness: 0.9,
            metalness: 0.1
        });
        const floor = new THREE.Mesh(floorGeo, floorMat);
        floor.position.z = -605;
        floor.receiveShadow = true;
        this.scene.add(floor);
    }

    buildAluminumFrame() {
        // 2020 Aluminum extrusions (black anodized)
        const vslotMat = new THREE.MeshStandardMaterial({
            color: 0x1e2430,
            roughness: 0.5,
            metalness: 0.8
        });

        const frameGroup = new THREE.Group();
        const legH = 700; // Height of frame
        const legGeo = new THREE.BoxGeometry(20, 20, legH);
        const halfW = 250; // 500x500 frame

        // 4 Legs
        const legOffsets = [
            [-halfW, -halfW],
            [ halfW, -halfW],
            [ halfW,  halfW],
            [-halfW,  halfW]
        ];
        legOffsets.forEach(([lx, ly]) => {
            const leg = new THREE.Mesh(legGeo, vslotMat);
            leg.position.set(lx, ly, -legH / 2 + 50);
            leg.castShadow = true;
            leg.receiveShadow = true;
            frameGroup.add(leg);
        });

        // Top horizontal bars
        const horizGeoX = new THREE.BoxGeometry(halfW * 2, 20, 20);
        const barTop1 = new THREE.Mesh(horizGeoX, vslotMat);
        barTop1.position.set(0, -halfW, 40);
        frameGroup.add(barTop1);
        const barTop2 = new THREE.Mesh(horizGeoX, vslotMat);
        barTop2.position.set(0, halfW, 40);
        frameGroup.add(barTop2);

        const horizGeoY = new THREE.BoxGeometry(20, halfW * 2 - 40, 20);
        const barTop3 = new THREE.Mesh(horizGeoY, vslotMat);
        barTop3.position.set(-halfW, 0, 40);
        frameGroup.add(barTop3);
        const barTop4 = new THREE.Mesh(horizGeoY, vslotMat);
        barTop4.position.set(halfW, 0, 40);
        frameGroup.add(barTop4);

        // Lower Bed Plate (as seen in CAD green bed plate)
        const bedGeo = new THREE.BoxGeometry(halfW * 2 - 20, halfW * 2 - 20, 8);
        const bedMat = new THREE.MeshStandardMaterial({
            color: 0x2b4c3f, // Industrial sage green from user CAD Bed.SLDPRT
            roughness: 0.6,
            metalness: 0.3
        });
        const bed = new THREE.Mesh(bedGeo, bedMat);
        bed.position.set(0, 0, -480);
        bed.receiveShadow = true;
        frameGroup.add(bed);

        this.scene.add(frameGroup);
    }

    buildBasePlate() {
        // Top mounting plate with central 3-lobed aperture (aluminum sheet)
        const plateShape = new THREE.Shape();
        const s = 250;
        plateShape.moveTo(-s, -s);
        plateShape.lineTo(s, -s);
        plateShape.lineTo(s, s);
        plateShape.lineTo(-s, s);
        plateShape.closePath();

        // 3 Triangular holes for arm clearance
        for (let i = 0; i < 3; i++) {
            const phi = (i * 120) * Math.PI / 180.0;
            const hole = new THREE.Path();
            const r_in = 60;
            const r_out = 180;
            const w = 45;

            const p1 = [r_in * Math.cos(phi) - w * Math.sin(phi), r_in * Math.sin(phi) + w * Math.cos(phi)];
            const p2 = [r_out * Math.cos(phi) - w * 1.5 * Math.sin(phi), r_out * Math.sin(phi) + w * 1.5 * Math.cos(phi)];
            const p3 = [r_out * Math.cos(phi) + w * 1.5 * Math.sin(phi), r_out * Math.sin(phi) - w * 1.5 * Math.cos(phi)];
            const p4 = [r_in * Math.cos(phi) + w * Math.sin(phi), r_in * Math.sin(phi) - w * Math.cos(phi)];

            hole.moveTo(p1[0], p1[1]);
            hole.lineTo(p2[0], p2[1]);
            hole.lineTo(p3[0], p3[1]);
            hole.lineTo(p4[0], p4[1]);
            hole.closePath();
            plateShape.holes.push(hole);
        }

        const extrudeSettings = { depth: 5, bevelEnabled: true, bevelSegments: 2, steps: 1, bevelSize: 1, bevelThickness: 1 };
        const plateGeo = new THREE.ExtrudeGeometry(plateShape, extrudeSettings);
        const plateMat = new THREE.MeshStandardMaterial({
            color: 0x94a3b8, // anodized aluminum
            metalness: 0.85,
            roughness: 0.35
        });
        const basePlate = new THREE.Mesh(plateGeo, plateMat);
        basePlate.position.z = 0; // Top surface at Z = 5
        basePlate.castShadow = true;
        basePlate.receiveShadow = true;
        this.scene.add(basePlate);
    }

    buildMotorsAndBiceps() {
        // NEMA 23 Motors: 56.4 x 56.4 x 76 mm
        const motorBodyGeo = new THREE.BoxGeometry(56.4, 76, 56.4);
        const motorMat = new THREE.MeshStandardMaterial({ color: 0x111317, roughness: 0.5, metalness: 0.6 });
        const faceplateMat = new THREE.MeshStandardMaterial({ color: 0xcfd8dc, roughness: 0.3, metalness: 0.9 });
        const bicepMat = new THREE.MeshStandardMaterial({
            color: 0xd97757, // Metallic copper / pink anodized bicep from user CAD
            metalness: 0.8,
            roughness: 0.25
        });
        const bracketMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.4, metalness: 0.5 }); // Golden/zinc plated bracket

        const L = this.kinematics.L; // 150 mm
        const R = this.kinematics.R; // Base radius

        for (let i = 0; i < 3; i++) {
            const phi = (i * 120) * Math.PI / 180.0;

            // Motor Assembly Container
            const motorGroup = new THREE.Group();

            // Mounting Bracket (L-shape)
            const bracket = new THREE.Mesh(new THREE.BoxGeometry(60, 60, 6), bracketMat);
            bracket.position.set(0, 0, 3);
            motorGroup.add(bracket);

            // Motor Body
            const motorMesh = new THREE.Mesh(motorBodyGeo, motorMat);
            motorMesh.position.set(0, -35, 34);
            motorMesh.castShadow = true;
            motorGroup.add(motorMesh);

            // Motor Faceplate
            const faceplate = new THREE.Mesh(new THREE.BoxGeometry(56.4, 8, 56.4), faceplateMat);
            faceplate.position.set(0, 5, 34);
            motorGroup.add(faceplate);

            // Motor Shaft & Flange
            const shaftGeo = new THREE.CylinderGeometry(4, 4, 25, 16);
            const shaft = new THREE.Mesh(shaftGeo, faceplateMat);
            shaft.rotation.x = Math.PI / 2;
            shaft.position.set(0, 16, 34);
            motorGroup.add(shaft);

            // Place motor on base plate at distance R
            motorGroup.rotation.z = phi - Math.PI / 2;
            motorGroup.position.set(R * Math.cos(phi), R * Math.sin(phi), 5);
            this.scene.add(motorGroup);
            this.motors.push(motorGroup);

            // ==========================================
            // Bicep Arm (Pivot is at Motor Shaft: Z=34+5=39, R)
            // Length = 150 mm
            // ==========================================
            const bicepGroup = new THREE.Group();

            // Bicep geometry: CAD shaped tapered arm with lightening holes
            const armShape = new THREE.Shape();
            armShape.moveTo(0, -22);
            armShape.lineTo(L, -12);
            armShape.absarc(L, 0, 12, -Math.PI / 2, Math.PI / 2, false);
            armShape.lineTo(0, 22);
            armShape.absarc(0, 0, 22, Math.PI / 2, -Math.PI / 2, false);

            // Holes
            const holeH1 = new THREE.Path();
            holeH1.absarc(0, 0, 8, 0, Math.PI * 2, true);
            armShape.holes.push(holeH1);
            const holeH2 = new THREE.Path();
            holeH2.absarc(L, 0, 6, 0, Math.PI * 2, true);
            armShape.holes.push(holeH2);

            const bicepGeo = new THREE.ExtrudeGeometry(armShape, { depth: 8, bevelEnabled: true, bevelThickness: 1, bevelSize: 1, bevelSegments: 2 });
            const bicepMesh = new THREE.Mesh(bicepGeo, bicepMat);
            bicepMesh.position.set(0, 0, -4);
            bicepMesh.castShadow = true;
            bicepGroup.add(bicepMesh);

            // M6 Cross Stud at elbow (stud_m6_80mm.SLDPRT)
            const studGeo = new THREE.CylinderGeometry(3, 3, 70, 16);
            const chromeMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.95, roughness: 0.15 });
            const stud = new THREE.Mesh(studGeo, chromeMat);
            stud.position.set(L, 0, 0);
            bicepGroup.add(stud);

            // Ball joints at ends of stud
            const ballGeo = new THREE.SphereGeometry(6, 16, 16);
            const ball1 = new THREE.Mesh(ballGeo, chromeMat);
            ball1.position.set(L, 0, 32);
            bicepGroup.add(ball1);
            const ball2 = new THREE.Mesh(ballGeo, chromeMat);
            ball2.position.set(L, 0, -32);
            bicepGroup.add(ball2);

            // Motor shaft pivot placement
            bicepGroup.position.set(R * Math.cos(phi), R * Math.sin(phi), 39);
            bicepGroup.rotation.z = phi; // Points radially outward
            this.scene.add(bicepGroup);
            this.biceps.push(bicepGroup);
        }
    }

    buildWristAndForearms() {
        const r = this.kinematics.r; // Platform radius (45 mm)
        const l = this.kinematics.l; // Forearm length (400 mm)

        // Wrist Plate (wrist.SLDPRT - Aluminum triangular traveling platform)
        const wristGroup = new THREE.Group();
        const wristShape = new THREE.Shape();
        for (let i = 0; i < 3; i++) {
            const a1 = (i * 120 - 25) * Math.PI / 180.0;
            const a2 = (i * 120 + 25) * Math.PI / 180.0;
            const px1 = (r + 15) * Math.cos(a1);
            const py1 = (r + 15) * Math.sin(a1);
            const px2 = (r + 15) * Math.cos(a2);
            const py2 = (r + 15) * Math.sin(a2);
            if (i === 0) wristShape.moveTo(px1, py1);
            else wristShape.lineTo(px1, py1);
            wristShape.lineTo(px2, py2);
        }
        wristShape.closePath();

        // Central tooling aperture
        const centerHole = new THREE.Path();
        centerHole.absarc(0, 0, 16, 0, Math.PI * 2, true);
        wristShape.holes.push(centerHole);

        const wristGeo = new THREE.ExtrudeGeometry(wristShape, { depth: 8, bevelEnabled: true, bevelThickness: 1, bevelSize: 1, bevelSegments: 2 });
        const wristMat = new THREE.MeshStandardMaterial({
            color: 0x38bdf8, // Sky-blue / anodized aluminum traveling plate
            metalness: 0.85,
            roughness: 0.25
        });
        const wristMesh = new THREE.Mesh(wristGeo, wristMat);
        wristMesh.position.z = -4;
        wristMesh.castShadow = true;
        wristGroup.add(wristMesh);

        // Tool effector / gripper tip
        const toolGeo = new THREE.CylinderGeometry(5, 1.5, 35, 16);
        const toolMat = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.9, roughness: 0.2 });
        const tool = new THREE.Mesh(toolGeo, toolMat);
        tool.rotation.x = Math.PI / 2;
        tool.position.set(0, 0, -22);
        wristGroup.add(tool);

        // Glow TCP (Tool Center Point) ring
        const ringGeo = new THREE.TorusGeometry(8, 1.5, 16, 32);
        const glowMat = new THREE.MeshBasicMaterial({ color: 0x00f2fe });
        const ring = new THREE.Mesh(ringGeo, glowMat);
        ring.position.set(0, 0, -39);
        wristGroup.add(ring);

        this.scene.add(wristGroup);
        this.wristPlate = wristGroup;

        // ==========================================
        // 3 Forearms: Each arm has 2 parallel carbon fiber rods (400 mm)
        // ==========================================
        const rodGeo = new THREE.CylinderGeometry(3.5, 3.5, l, 16);
        const carbonMat = new THREE.MeshStandardMaterial({
            color: 0x18181b, // Carbon fiber matte dark
            roughness: 0.4,
            metalness: 0.7
        });
        const rodEndMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.9, roughness: 0.2 });

        for (let i = 0; i < 3; i++) {
            const armPair = {
                rodA: null,
                rodB: null,
                group: new THREE.Group()
            };

            // Rod A
            const rodA = new THREE.Mesh(rodGeo, carbonMat);
            rodA.castShadow = true;
            armPair.group.add(rodA);
            armPair.rodA = rodA;

            // Rod B
            const rodB = new THREE.Mesh(rodGeo, carbonMat);
            rodB.castShadow = true;
            armPair.group.add(rodB);
            armPair.rodB = rodB;

            this.scene.add(armPair.group);
            this.forearms.push(armPair);
        }
    }

    buildTrajectoryTrail() {
        const maxPoints = this.maxTrailPoints;
        const positions = new Float32Array(maxPoints * 3);
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        const material = new THREE.LineBasicMaterial({
            color: 0x00f2fe,
            linewidth: 2,
            transparent: true,
            opacity: 0.8
        });

        this.trailLine = new THREE.Line(geometry, material);
        this.scene.add(this.trailLine);
    }

    buildWorkspaceEnvelope() {
        // Transparent dome illustrating the reachable delta workspace
        const points = [];
        const radius = 230;
        const geom = new THREE.CylinderGeometry(radius * 0.95, radius * 0.6, 260, 32, 12, true);
        const mat = new THREE.MeshBasicMaterial({
            color: 0x00f2fe,
            wireframe: true,
            transparent: true,
            opacity: 0.08
        });
        this.workspaceMesh = new THREE.Mesh(geom, mat);
        this.workspaceMesh.position.set(0, 0, -380);
        this.workspaceMesh.rotation.x = Math.PI / 2;
        this.scene.add(this.workspaceMesh);
    }

    /**
     * Updates robot configuration in 3D scene (angles in degrees, Cartesian in mm)
     */
    updatePose(thetas_deg, cartesian_pos) {
        if (!thetas_deg || !cartesian_pos) return;

        const L = this.kinematics.L; // 150 mm
        const l = this.kinematics.l; // 400 mm
        const R = this.kinematics.R;
        const r = this.kinematics.r;
        const d_rods = 32.0; // Half spacing between parallel rods

        // 1. Position Wrist Platform
        this.wristPlate.position.set(cartesian_pos[0], cartesian_pos[1], cartesian_pos[2]);

        // 2. Rotate each Bicep around its motor shaft
        for (let i = 0; i < 3; i++) {
            const phi = this.kinematics.phi_rad[i];
            const th_deg = thetas_deg[i];
            const th_rad = th_deg * Math.PI / 180.0;

            const bicep = this.biceps[i];
            // Pivot rotation:
            // In local coordinates: bicep points along +X, rotates around Y axis (or around shaft axis)
            bicep.rotation.order = 'ZYX';
            bicep.rotation.z = phi;
            bicep.rotation.y = th_rad; // Positive theta tilts downward

            // Calculate exact 3D elbow coordinates:
            const elbow_r = R + L * Math.cos(th_rad);
            const elbow_z = 39 - L * Math.sin(th_rad);
            const elbow_x = elbow_r * Math.cos(phi);
            const elbow_y = elbow_r * Math.sin(phi);

            // Calculate wrist attachment coordinates:
            const wrist_x = cartesian_pos[0] + r * Math.cos(phi);
            const wrist_y = cartesian_pos[1] + r * Math.sin(phi);
            const wrist_z = cartesian_pos[2];

            // Tangential unit vector (perpendicular to radial phi) for rod spacing
            const tan_x = -Math.sin(phi);
            const tan_y = Math.cos(phi);

            // Elbow Joint A & B
            const elbowA = new THREE.Vector3(elbow_x + d_rods * tan_x, elbow_y + d_rods * tan_y, elbow_z);
            const elbowB = new THREE.Vector3(elbow_x - d_rods * tan_x, elbow_y - d_rods * tan_y, elbow_z);

            // Wrist Joint A & B
            const wristA = new THREE.Vector3(wrist_x + d_rods * tan_x, wrist_y + d_rods * tan_y, wrist_z);
            const wristB = new THREE.Vector3(wrist_x - d_rods * tan_x, wrist_y - d_rods * tan_y, wrist_z);

            // Orient Forearm Rod A
            this.alignRod(this.forearms[i].rodA, elbowA, wristA, l);
            // Orient Forearm Rod B
            this.alignRod(this.forearms[i].rodB, elbowB, wristB, l);
        }

        // 3. Update Trajectory Trail
        if (this.showTrail) {
            this.trailPoints.push(new THREE.Vector3(cartesian_pos[0], cartesian_pos[1], cartesian_pos[2] - 39));
            if (this.trailPoints.length > this.maxTrailPoints) {
                this.trailPoints.shift();
            }
            const posAttr = this.trailLine.geometry.attributes.position;
            for (let i = 0; i < this.trailPoints.length; i++) {
                posAttr.setXYZ(i, this.trailPoints[i].x, this.trailPoints[i].y, this.trailPoints[i].z);
            }
            this.trailLine.geometry.setDrawRange(0, this.trailPoints.length);
            posAttr.needsUpdate = true;
        }
    }

    alignRod(mesh, startPoint, endPoint, targetLength) {
        const mid = new THREE.Vector3().addVectors(startPoint, endPoint).multiplyScalar(0.5);
        mesh.position.copy(mid);

        const dir = new THREE.Vector3().subVectors(endPoint, startPoint).normalize();
        const up = new THREE.Vector3(0, 1, 0); // Cylinder original axis is Y
        const quaternion = new THREE.Quaternion().setFromUnitVectors(up, dir);
        mesh.quaternion.copy(quaternion);
    }

    clearTrail() {
        this.trailPoints = [];
        this.trailLine.geometry.setDrawRange(0, 0);
    }

    setCameraView(viewName) {
        if (!this.controls) return;
        const target = this.controls.target;
        const dist = 650;
        if (viewName === 'top') {
            this.camera.position.set(0, 1, dist + target.z);
            this.camera.up.set(0, 1, 0);
        } else if (viewName === 'front') {
            this.camera.position.set(0, -dist, target.z);
            this.camera.up.set(0, 0, 1);
        } else if (viewName === 'side') {
            this.camera.position.set(dist, 0, target.z);
            this.camera.up.set(0, 0, 1);
        } else if (viewName === 'iso') {
            this.camera.position.set(450, -450, 150);
            this.camera.up.set(0, 0, 1);
        }
        this.controls.update();
    }

    toggleWorkspace(show) {
        this.showWorkspace = show !== undefined ? show : !this.showWorkspace;
        if (this.workspaceMesh) this.workspaceMesh.visible = this.showWorkspace;
    }

    toggleTrail(show) {
        this.showTrail = show !== undefined ? show : !this.showTrail;
        if (this.trailLine) this.trailLine.visible = this.showTrail;
    }

    onResize() {
        if (!this.container || !this.renderer || !this.camera) return;
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        if (this.controls) this.controls.update();
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
    }
}
