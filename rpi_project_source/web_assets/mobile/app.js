/**
 * RCSIM Cockpit - Core Application Logic
 * Obsługa WebRTC, Gamepad API oraz Telemetrii HUD.
 */

const CONFIG = {
    signalingUrl: window.location.origin + '/offer',
    telemetryFreq: 30, // Hz
    controlFreq: 50,   // Hz
    deadzone: 0.05
};

class CockpitApp {
    constructor() {
        this.pc = null;
        this.controlChannel = null;
        this.telemetryChannel = null;
        this.isConnected = false;
        this.isArmed = false;
        
        // Gamepad state
        this.gamepadIdx = null;
        this.uiSteer = 0;
        this.uiThrottle = 0;
        this.joyLeft = null;
        this.joyRight = null;
        
        // DOM Elements
        this.video = document.getElementById('camera-view');
        this.connDot = document.getElementById('conn-indicator');
        this.connText = document.getElementById('conn-text');
        this.armBtn = document.getElementById('arm-btn');
        this.speedVal = document.getElementById('speed-val');
        this.battVal = document.getElementById('batt-val');
        this.thrBar = document.getElementById('thr-bar');
        this.strBar = document.getElementById('str-bar');
        this.modeText = document.getElementById('current-mode');
        this.gamepadInfo = document.getElementById('gamepad-info');
        
        this.init();
    }

    async init() {
        this.setupEventListeners();
        this.setupGamepadDetection();
        this.initJoysticks();
        await this.startWebRTC();
        this.startVideo();
        this.startControlLoop();
        this.registerServiceWorker();
    }

    initJoysticks() {
        const leftEl = document.getElementById('joy-left');
        const rightEl = document.getElementById('joy-right');

        // Throttle Joystick (Left - locked to Y-axis)
        this.joyLeft = nipplejs.create({
            zone: leftEl,
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: 'rgba(255, 0, 85, 0.8)',
            size: 100,
            lockY: true
        });

        this.joyLeft.on('move', (evt, data) => {
            if (data.direction) {
                const maxDist = 50;
                let val = Math.min(data.distance / maxDist, 1.0);
                if (data.direction.y === 'down') {
                    this.uiThrottle = -val;
                } else if (data.direction.y === 'up') {
                    this.uiThrottle = val;
                }
            }
        });

        this.joyLeft.on('end', () => {
            this.uiThrottle = 0;
        });

        // Steering Joystick (Right - locked to X-axis)
        this.joyRight = nipplejs.create({
            zone: rightEl,
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: 'rgba(0, 243, 255, 0.8)',
            size: 100,
            lockX: true
        });

        this.joyRight.on('move', (evt, data) => {
            if (data.direction) {
                const maxDist = 50;
                let val = Math.min(data.distance / maxDist, 1.0);
                if (data.direction.x === 'left') {
                    this.uiSteer = -val;
                } else if (data.direction.x === 'right') {
                    this.uiSteer = val;
                }
            }
        });

        this.joyRight.on('end', () => {
            this.uiSteer = 0;
        });
    }

    setupEventListeners() {
        this.armBtn.addEventListener('click', () => this.toggleArm());
        
        document.getElementById('emergency-toggle').addEventListener('click', () => {
            const joyZone = document.getElementById('joystick-zone');
            joyZone.classList.toggle('hidden');
        });

        window.addEventListener('gamepadconnected', (e) => {
            console.log("Gamepad connected:", e.gamepad.id);
            this.gamepadIdx = e.gamepad.index;
            this.gamepadInfo.classList.remove('hidden');
        });

        window.addEventListener('gamepaddisconnected', () => {
            console.log("Gamepad disconnected");
            this.gamepadIdx = null;
            this.gamepadInfo.classList.add('hidden');
        });
    }

    setupGamepadDetection() {
        // Initial check for already connected pads
        const pads = navigator.getGamepads();
        for (let i = 0; i < pads.length; i++) {
            if (pads[i]) {
                this.gamepadIdx = i;
                this.gamepadInfo.classList.remove('hidden');
                break;
            }
        }
    }

    async startWebRTC() {
        this.pc = new RTCPeerConnection({
            iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
        });

        // Create Data Channels
        this.controlChannel = this.pc.createDataChannel('control', { ordered: false, maxRetransmits: 0 });
        this.telemetryChannel = this.pc.createDataChannel('telemetry');

        this.telemetryChannel.onmessage = (e) => this.handleTelemetry(e.data);
        
        this.pc.onconnectionstatechange = () => {
            console.log("Connection State:", this.pc.connectionState);
            this.updateConnectionStatus(this.pc.connectionState === 'connected');
        };

        // Negotiation
        try {
            const offer = await this.pc.createOffer();
            await this.pc.setLocalDescription(offer);

            const response = await fetch(CONFIG.signalingUrl, {
                method: 'POST',
                body: JSON.stringify({ sdp: this.pc.localDescription.sdp, type: this.pc.localDescription.type }),
                headers: { 'Content-Type': 'application/json' }
            });

            const answer = await response.json();
            await this.pc.setRemoteDescription(new RTCSessionDescription(answer));
        } catch (e) {
            console.error("WebRTC Error:", e);
            this.updateConnectionStatus(false);
        }
    }

    async startVideo() {
        try {
            const videoPc = new RTCPeerConnection({
                iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
            });
            
            videoPc.addTransceiver('video', { direction: 'recvonly' });
            
            videoPc.ontrack = (event) => {
                if (event.track.kind === 'video') {
                    this.video.srcObject = event.streams[0];
                }
            };
            
            const offer = await videoPc.createOffer();
            await videoPc.setLocalDescription(offer);
            
            const signalingPort = 8889;
            const whepUrl = `http://${window.location.hostname}:${signalingPort}/camera_ai/whep`;
            
            const response = await fetch(whepUrl, {
                method: 'POST',
                body: offer.sdp,
                headers: { 'Content-Type': 'application/sdp' }
            });
            
            if (response.ok) {
                const answerSdp = await response.text();
                await videoPc.setRemoteDescription(new RTCSessionDescription({
                    type: 'answer',
                    sdp: answerSdp
                }));
                console.log("WHEP Video Stream Connected successfully.");
            } else {
                console.warn("WHEP signaling response status not OK:", response.status);
            }
        } catch (e) {
            console.error("WHEP Video Error:", e);
        }
    }

    updateConnectionStatus(connected) {
        this.isConnected = connected;
        if (connected) {
            this.connDot.className = 'dot green';
            this.connText.innerText = 'CONNECTED';
        } else {
            this.connDot.className = 'dot red';
            this.connText.innerText = 'DISCONNECTED';
        }
    }

    handleTelemetry(data) {
        try {
            const tele = JSON.parse(data);
            
            // Map short keys to long keys for compatibility
            const speed = tele.sp !== undefined ? tele.sp : (tele.po ? Math.hypot(tele.po[0], tele.po[1]) : 0.0); 
            const voltage = tele.bat ? tele.bat.voltage : (tele.battery ? tele.battery.voltage : 0.0);
            const mode = tele.mo || tele.mode;
            const armed = tele.arm !== undefined ? tele.arm : tele.pca_armed;

            if (speed !== undefined) this.speedVal.innerText = speed.toFixed(1);
            if (voltage !== undefined) this.battVal.innerText = voltage.toFixed(1);
            if (mode) this.modeText.innerText = mode;
            
            if (armed !== undefined) {
                this.setArmedState(armed);
            }
        } catch (e) {}
    }

    setArmedState(armed) {
        this.isArmed = armed;
        if (armed) {
            this.armBtn.classList.add('armed');
            this.armBtn.querySelector('.arm-status').innerText = 'ARMED';
        } else {
            this.armBtn.classList.remove('armed');
            this.armBtn.querySelector('.arm-status').innerText = 'DISARMED';
        }
    }

    toggleArm() {
        const cmd = this.isArmed ? 'DISARM_PCA' : 'ARM_PCA';
        this.sendCommand({ command: cmd });
    }

    sendCommand(msg) {
        if (this.telemetryChannel && this.telemetryChannel.readyState === 'open') {
            this.telemetryChannel.send(JSON.stringify(msg));
        }
    }

    startControlLoop() {
        const loop = () => {
            this.processControls();
            setTimeout(loop, 1000 / CONFIG.controlFreq);
        };
        loop();
    }

    processControls() {
        if (!this.isConnected || this.controlChannel.readyState !== 'open') return;

        let steer = 0;
        let throttle = 0;

        if (this.gamepadIdx !== null) {
            const pad = navigator.getGamepads()[this.gamepadIdx];
            if (pad) {
                // Standard Gamepad Mapping (Xbox/PS)
                // Axis 0: Steering (Left Stick X)
                // Axis 1: Throttle (Left Stick Y - Inverse) or Triggers
                steer = pad.axes[0];
                
                // Trigger logic (RT - LT) for throttle is often better
                const rt = pad.buttons[7] ? pad.buttons[7].value : 0;
                const lt = pad.buttons[6] ? pad.buttons[6].value : 0;
                
                if (rt > 0 || lt > 0) {
                    throttle = rt - lt;
                } else {
                    throttle = -pad.axes[1]; // Fallback to stick
                }
            }
        } else {
            // Fallback to UI virtual joysticks
            steer = this.uiSteer;
            throttle = this.uiThrottle;
        }

        // Apply Deadzone
        if (Math.abs(steer) < CONFIG.deadzone) steer = 0;
        if (Math.abs(throttle) < CONFIG.deadzone) throttle = 0;

        // Update HUD Bars
        this.thrBar.style.width = Math.abs(throttle * 100) + '%';
        this.strBar.style.width = Math.abs(steer * 50) + '%';
        this.strBar.style.marginLeft = steer >= 0 ? '50%' : (50 + steer * 50) + '%';

        // Send Binary Packet (CT format)
        this.sendBinaryControl(steer, throttle);
    }

    sendBinaryControl(steer, throttle) {
        // Map normalized -1..1 to 1000..2000
        const s_pwm = Math.round(1500 + steer * 500);
        const t_pwm = Math.round(1500 + throttle * 500);

        const buffer = new ArrayBuffer(27);
        const view = new DataView(buffer);

        // Header 'CT'
        view.setUint8(0, 0x43);
        view.setUint8(1, 0x54);

        // Channels (8 channels total, only 1-2 used)
        view.setUint16(2, s_pwm, true);
        view.setUint16(4, t_pwm, true);
        for (let i = 2; i < 8; i++) {
            view.setUint16(2 + i * 2, 1500, true);
        }

        // Timestamp (Double, 8 bytes)
        view.setFloat64(18, Date.now() / 1000.0, true);

        // Checksum (XOR)
        let crc = 0;
        const bytes = new Uint8Array(buffer);
        for (let i = 0; i < 26; i++) {
            crc ^= bytes[i];
        }
        view.setUint8(26, crc);

        if (this.controlChannel.readyState === 'open') {
            this.controlChannel.send(buffer);
        }
    }

    registerServiceWorker() {
        if ('serviceWorker' in navigator) {
            window.addEventListener('load', () => {
                // navigator.serviceWorker.register('sw.js').then(...)
                console.log("Service Worker registration skipped for dev mode.");
            });
        }
    }
}

// Start App
window.addEventListener('DOMContentLoaded', () => {
    window.app = new CockpitApp();
});
