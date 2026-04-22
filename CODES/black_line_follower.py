"""
Advanced Vision Line Follower - Python Brain
Features: Adaptive Thresholding, PD Control, Speed Gradient, Web UI, Corner Memory
"""

import cv2
import numpy as np
import serial
import time
from flask import Flask, Response, request

app = Flask(__name__)

# ── Configuration ──────────────────────────────────────────────────────────────
SERIAL_PORT   = '/dev/ttyUSB0' 
BAUD_RATE     = 115200  
MIN_AREA      = 600      

# ── ROBOT STATE (Controlled by Web UI) ─────────────────────────────────────────
robot_state = {
    'running': False,      
    'speed_limit': 80      
}

ACCEL_RATE   = 5         
BRAKE_RATE   = 10        
KP = 0.6                 
KD = 0.3                 
# ───────────────────────────────────────────────────────────────────────────────

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"✅ Arduino connected on {SERIAL_PORT}")
except serial.SerialException as e:
    ser = None
    print(f"⚠️ Serial ERROR: {e}. Running vision only.")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

def process_frame(frame):
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    
    mask = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 61, 15)
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    roi_top = int(height * 0.4)
    roi     = mask[roi_top:, :]
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: return mask, None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_AREA: return mask, None

    M = cv2.moments(largest)
    if M["m00"] == 0: return mask, None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"]) + roi_top
    return mask, (cx, cy)

def gen_frames():
    last_error = 0
    current_base_speed = 0 
    last_known_error = 0 
    search_state = 'TRACKING' 

    while True:
        ret, frame = cap.read()
        if not ret: break

        width = frame.shape[1]
        frame_cx = width // 2
        
        mask, centroid = process_frame(frame)
        
        target_base_speed = 0
        motor_adjust = 0
        max_spd = robot_state['speed_limit']
        turn_spd = int(max_spd * 0.7) 

        if not robot_state['running']:
            target_base_speed = 0
            motor_adjust = 0
            cv2.putText(frame, "PAUSED (Web UI)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            search_state = 'TRACKING' 
        else:
            if centroid is not None:
                search_state = 'TRACKING' 
                cx, cy  = centroid
                error   = cx - frame_cx  
                last_known_error = error 
                
                if abs(error) > 50: 
                    target_base_speed = turn_spd 
                else:
                    target_base_speed = max_spd  

                delta_error = error - last_error
                last_error = error
                motor_adjust = (KP * error) + (KD * delta_error)

                cv2.line(frame, (frame_cx, 0), (frame_cx, frame.shape[0]), (255, 100, 0), 1)
                cv2.circle(frame, (cx, cy), 12, (0, 0, 255), -1)
                cv2.line(frame, (frame_cx, cy), (cx, cy), (0, 255, 0), 2)
                cv2.putText(frame, "TRACKING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            else:
                if search_state == 'TRACKING':
                    if last_known_error > 40:
                        search_state = 'SEARCH_RIGHT'
                    elif last_known_error < -40:
                        search_state = 'SEARCH_LEFT'
                    else:
                        search_state = 'STOPPED'

                if search_state == 'SEARCH_RIGHT':
                    target_base_speed = 0
                    motor_adjust = int(max_spd * 1.5) 
                    cv2.putText(frame, "SEARCHING RIGHT >>>", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                elif search_state == 'SEARCH_LEFT':
                    target_base_speed = 0
                    motor_adjust = -int(max_spd * 1.5) 
                    cv2.putText(frame, "<<< SEARCHING LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                else:
                    target_base_speed = 0
                    motor_adjust = 0
                    cv2.putText(frame, "LINE LOST. STOPPING.", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Speed Gradient
        if current_base_speed < target_base_speed:
            current_base_speed += ACCEL_RATE  
            if current_base_speed > target_base_speed: current_base_speed = target_base_speed
        elif current_base_speed > target_base_speed:
            current_base_speed -= BRAKE_RATE  
            if current_base_speed < target_base_speed: current_base_speed = target_base_speed

        left_speed  = int(current_base_speed + motor_adjust)
        right_speed = int(current_base_speed - motor_adjust)

        left_speed  = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))

        if ser:
            command = f"{left_speed},{right_speed}\n"
            ser.write(command.encode())

        # Dual View setup
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_bgr, "VISION MASK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        combined_frame = np.hstack((frame, mask_bgr))

        ret, buffer = cv2.imencode('.jpg', combined_frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# ── WEB API ROUTES ─────────────────────────────────────────────────────────────

@app.route('/toggle')
def toggle():
    robot_state['running'] = not robot_state['running']
    return {"running": robot_state['running']}

@app.route('/speed')
def set_speed():
    val = request.args.get('val', 80)
    robot_state['speed_limit'] = int(val)
    return {"status": "ok", "speed": robot_state['speed_limit']}

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Dashboard</title>
        <style>
            body { font-family: Arial; text-align: center; background: #222; color: white; padding: 20px;}
            img { border: 4px solid #444; border-radius: 10px; max-width: 100%; }
            .btn { background: #E74C3C; color: white; border: none; padding: 15px 40px; font-size: 24px; font-weight: bold; border-radius: 8px; cursor: pointer; margin: 20px;}
            .btn.running { background: #2ECC71; }
            .slider { width: 50%; margin: 20px; }
        </style>
        <script>
            function toggleBot() {
                fetch('/toggle').then(response => response.json()).then(data => {
                    let btn = document.getElementById('controlBtn');
                    if(data.running) {
                        btn.innerText = "STOP ROBOT";
                        btn.className = "btn";
                    } else {
                        btn.innerText = "START ROBOT";
                        btn.className = "btn running";
                    }
                });
            }
            function updateSpeed(val) {
                fetch('/speed?val=' + val);
                document.getElementById('speedText').innerText = val;
            }
        </script>
    </head>
    <body>
        <h1>Advanced Line Follower Dashboard</h1>
        <img src="/video_feed">
        <br>
        <button id="controlBtn" class="btn running" onclick="toggleBot()">START ROBOT</button>
        <br>
        <h2>Max Speed Limit: <span id="speedText">80</span></h2>
        <input type="range" min="40" max="200" value="80" class="slider" oninput="updateSpeed(this.value)">
    </body>
    </html>
    """
    return html

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        if ser:
            ser.write("0,0\n".encode())
            ser.close()
        cap.release()
