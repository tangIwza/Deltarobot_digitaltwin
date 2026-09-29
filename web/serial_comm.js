/**
 * Dual Serial & WebSocket Communication Manager
 * =============================================
 * Supports:
 * 1. WebSerial API (Direct browser-to-USB/Arduino connection)
 * 2. WebSocket bridge to Python backend (PySerial)
 * 3. Autonomous Simulator Fallback
 */

class HardwareCommunicator {
    constructor(onTelemetryCallback, onLogCallback) {
        this.onTelemetry = onTelemetryCallback || (() => {});
        this.onLog = onLogCallback || (() => {});

        this.mode = 'DISCONNECTED'; // 'WEBSOCKET' | 'WEBSERIAL' | 'SIMULATION' | 'DISCONNECTED'
        this.ws = null;
        this.serialPort = null;
        this.serialReader = null;
        this.serialWriter = null;
        this.isConnected = false;

        this.initWebSocket();
    }

    initWebSocket() {
        // Attempt connecting to Python server WebSocket
        const host = window.location.host || 'localhost:8000';
        const wsUrl = `ws://${host}/ws`;

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                this.mode = 'WEBSOCKET';
                this.isConnected = true;
                this.onLog('SYSTEM', 'Connected to Python Controller Server');
            };

            this.ws.onmessage = (event) => {
                try {
                    const msg = JSON.parse(event.data);
                    if (msg.type === 'telemetry') {
                        this.onTelemetry(msg.data);
                    }
                } catch (e) {
                    console.error("WS parse error:", e);
                }
            };

            this.ws.onclose = () => {
                if (this.mode === 'WEBSOCKET') {
                    this.mode = 'DISCONNECTED';
                    this.isConnected = false;
                    this.onLog('SYSTEM', 'Python Server connection closed. Ready for WebSerial or Simulator.');
                }
            };

            this.ws.onerror = () => {
                // If backend is not running, we degrade gracefully to WebSerial or local simulation
                this.ws = null;
            };
        } catch (e) {
            this.ws = null;
        }
    }

    // ==========================================
    // WebSerial API (Chrome / Edge direct USB)
    // ==========================================

    async connectWebSerial(baudRate = 115200) {
        if (!('serial' in navigator)) {
            alert("WebSerial API is not supported in this browser. Please use Google Chrome, Microsoft Edge, or the Python backend.");
            return false;
        }

        try {
            this.serialPort = await navigator.serial.requestPort();
            await this.serialPort.open({ baudRate: parseInt(baudRate) });

            this.mode = 'WEBSERIAL';
            this.isConnected = true;
            this.onLog('SYSTEM', `WebSerial Connected @ ${baudRate} baud`);

            this.startWebSerialReader();
            // Send identify
            this.sendGcode("M115");
            return true;
        } catch (err) {
            this.onLog('ERROR', `WebSerial connection failed: ${err.message}`);
            return false;
        }
    }

    async startWebSerialReader() {
        while (this.serialPort && this.serialPort.readable) {
            const textDecoder = new TextDecoderStream();
            const readableStreamClosed = this.serialPort.readable.pipeTo(textDecoder.writable);
            this.serialReader = textDecoder.readable.getReader();

            try {
                let buffer = '';
                while (true) {
                    const { value, done } = await this.serialReader.read();
                    if (done) break;
                    if (value) {
                        buffer += value;
                        const lines = buffer.split('\n');
                        buffer = lines.pop(); // keep remainder
                        for (const line of lines) {
                            const trimmed = line.trim();
                            if (trimmed) this.onLog('RECV', trimmed);
                        }
                    }
                }
            } catch (error) {
                console.error("WebSerial read error:", error);
            } finally {
                this.serialReader.releaseLock();
            }
        }
    }

    async disconnect() {
        if (this.mode === 'WEBSERIAL' && this.serialPort) {
            try {
                if (this.serialReader) {
                    await this.serialReader.cancel();
                }
                await this.serialPort.close();
            } catch (e) {
                console.error(e);
            }
            this.serialPort = null;
        }

        if (this.mode === 'WEBSOCKET') {
            // Send disconnect command to Python backend
            fetch('/api/serial/disconnect', { method: 'POST' }).catch(() => {});
        }

        this.mode = 'DISCONNECTED';
        this.isConnected = false;
        this.onLog('SYSTEM', 'Disconnected from hardware');
    }

    // ==========================================
    // Send Command Dispatcher
    // ==========================================

    async sendGcode(command) {
        this.onLog('SEND', command);

        // 1. If WebSerial is active, send direct to USB port
        if (this.mode === 'WEBSERIAL' && this.serialPort && this.serialPort.writable) {
            try {
                const encoder = new TextEncoder();
                const writer = this.serialPort.writable.getWriter();
                await writer.write(encoder.encode(command + '\n'));
                writer.releaseLock();
                return true;
            } catch (e) {
                this.onLog('ERROR', `WebSerial write error: ${e.message}`);
                return false;
            }
        }

        // 2. If WebSocket is connected, send to Python server
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({ action: "gcode", command: command }));
            return true;
        }

        // 3. Fallback: REST API
        try {
            await fetch('/api/gcode', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command })
            });
            return true;
        } catch (e) {
            return false;
        }
    }

    sendWsAction(action, payload = {}) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({ action, ...payload }));
            return true;
        }
        return false;
    }
}
