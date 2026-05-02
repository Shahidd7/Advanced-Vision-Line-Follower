import cv2
import numpy as np
import serial
import time
import threading
from flask import Flask, Response, request

app = Flask(__name__)

# ================================================================
#  CONFIG
# ================================================================
SERIAL_PORT  = '/dev/ttyUSB0'
BAUD_RATE    = 115200
MIN_AREA     = 600          # minimum contour area to be considered a line

# PD controller gains
KP = 0.5
KD = 0.25

# Acceleration / braking rates (per frame)
ACCEL_RATE  = 5
BRAKE_RATE  = 20            # INCREASED from 10 → brakes faster before curves

# ── Curve detection thresholds (in pixels, at 320px width) ──────
# direction = horizontal offset between top-centroid and bottom-centroid
MILD_CURVE_THRESHOLD  = 25   # > 25px  →  mild curve
SHARP_CURVE_THRESHOLD = 65   # > 65px  →  sharp curve
#   (old code used 40 for everything, which was too hair-trigger)

# ── Speed fractions per curve state ─────────────────────────────
SPEED_STRAIGHT  = 1.00       # full speed on straights
SPEED_MILD      = 0.75       # 75 % on mild curves
SPEED_SHARP     = 0.45       # 45 % on sharp curves
SPEED_LOST      = 0.35       # 35 % when line is completely lost

# ── Lookahead weight ─────────────────────────────────────────────
# How much the FAR (top) centroid contributes to curvature score.
# 0.6 = top region matters more → robot slows earlier.
LOOKAHEAD_WEIGHT = 0.6

# ── Arduino motor value range ────────────────────────────────────
# Set True  → Arduino accepts -255 … +255  (bidirectional / L298N style)
# Set False → Arduino accepts    0 … 255   (safe default if unsure)
ARDUINO_SUPPORTS_NEGATIVE = False

# ── Curve smoothing ──────────────────────────────────────────────
CURVE_HISTORY_LEN = 5        # frames to average severity over (avoids jitter)


# ================================================================
#  ROBOT STATE
# ================================================================
robot_state = {
    'running'           : False,
    'speed_limit'       : 120,
    'obstacle_distance' : 999,
    'curve_severity'    : 0,   # 0=straight 1=mild 2=sharp 3=lost
}


# ================================================================
#  SERIAL (Arduino)
# ================================================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    print(f"Arduino connected on {SERIAL_PORT}")
except Exception:
    ser = None
    print("Serial connection failed — running in camera-only mode")

def _read_serial():
    """Background thread: reads ultrasonic distance from Arduino."""
    while ser and ser.is_open:
        try:
            raw = ser.readline().decode(errors='ignore').strip()
            if raw.isdigit():
                robot_state['obstacle_distance'] = int(raw)
        except Exception:
            pass
        time.sleep(0.05)

threading.Thread(target=_read_serial, daemon=True).start()


# ================================================================
#  CAMERA
# ================================================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)   # FIX: was cap.set(3, 320) — wrong API
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)   # FIX: was cap.set(4, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)       # FIX: reduce buffer lag


# ================================================================
#  VISION HELPERS
# ================================================================
def get_centroid(region, offset_y):
    """
    Find the centroid of the largest contour in a binary region.
    offset_y shifts the returned y-coordinate back into full-frame space.
    Returns (cx, cy) or None if no valid line is found.
    """
    contours, _ = cv2.findContours(
        region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_AREA:
        return None

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"]) + offset_y
    return (cx, cy)


def process_frame(frame):
    """
    Convert frame → binary mask, then split into THREE horizontal bands:
      bottom  (65–100 %)  : immediate ground ahead — primary steering reference
      middle  (40– 65 %)  : near future            — early curve warning
      top     (15– 40 %)  : far lookahead           — predictive curve detection

    Returns mask, bottom_centroid, mid_centroid, top_centroid
    """
    h, w = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (11, 11), 0)

    mask = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        61, 15
    )

    # ── Slice the three regions ──────────────────────────────────
    bot_y  = int(h * 0.65)
    mid_y  = int(h * 0.40)
    top_y  = int(h * 0.15)

    lower  = mask[bot_y:,        :]
    middle = mask[mid_y:bot_y,   :]
    upper  = mask[top_y:mid_y,   :]

    bottom = get_centroid(lower,  bot_y)
    mid    = get_centroid(middle, mid_y)
    top    = get_centroid(upper,  top_y)

    return mask, bottom, mid, top, (bot_y, mid_y, top_y)


def compute_curve_severity(bottom, mid, top):
    """
    Compute a weighted curvature score from the three centroids.

    Logic:
      • curvature = (top.x − bottom.x) weighted heavily  +
                    (mid.x  − bottom.x) weighted lightly
      • If only some points are visible, use what we have.

    Returns (severity, curvature_score)
      severity 0 = straight
               1 = mild curve
               2 = sharp curve
               3 = line lost (bottom not visible)
    """
    if bottom is None:
        return 3, 0

    bx = bottom[0]

    if top is not None and mid is not None:
        curvature = (
            (top[0] - bx) * LOOKAHEAD_WEIGHT +
            (mid[0] - bx) * (1.0 - LOOKAHEAD_WEIGHT)
        )
    elif top is not None:
        curvature = float(top[0] - bx)
    elif mid is not None:
        curvature = float(mid[0] - bx)
    else:
        curvature = 0.0

    abs_c = abs(curvature)

    if abs_c > SHARP_CURVE_THRESHOLD:
        return 2, curvature
    elif abs_c > MILD_CURVE_THRESHOLD:
        return 1, curvature
    else:
        return 0, curvature


def safe_motor(val):
    """Clamp motor value to the range the Arduino can handle."""
    v = int(val)
    if ARDUINO_SUPPORTS_NEGATIVE:
        return max(-255, min(255, v))
    else:
        return max(0, min(255, v))


# ================================================================
#  MAIN FRAME GENERATOR
# ================================================================
def gen_frames():
    last_error          = 0
    current_speed       = 0
    last_seen_direction = 0        # used for line-lost recovery
    curve_history       = []       # rolling window of severity values

    # Colour palette for the three centroid dots
    DOT_COLOURS = [
        (0,  255,  0),    # bottom → bright green
        (0,  200, 255),   # middle → cyan
        (0,  100, 255),   # top    → orange-red
    ]
    SEV_LABEL  = {0:'STRAIGHT', 1:'MILD CURVE', 2:'SHARP CURVE', 3:'LINE LOST'}
    SEV_COLOUR = {0:(0,255,0), 1:(0,200,255), 2:(0,0,255), 3:(128,0,128)}

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        frame_center = w // 2

        mask, bottom, mid, top, (bot_y, mid_y, top_y) = process_frame(frame)

        max_spd      = robot_state['speed_limit']
        motor_adjust = 0
        target_speed = 0

        # ── Obstacle / stopped check ─────────────────────────────
        blocked = robot_state['obstacle_distance'] < 15

        if not robot_state['running'] or blocked:
            target_speed = 0
            motor_adjust = 0
            curve_history.clear()
            robot_state['curve_severity'] = 0

        else:
            # ── Curvature ─────────────────────────────────────────
            severity, curvature = compute_curve_severity(bottom, mid, top)

            # Rolling max → conservative: react to the worst seen recently
            curve_history.append(severity)
            if len(curve_history) > CURVE_HISTORY_LEN:
                curve_history.pop(0)
            smoothed = max(curve_history)
            robot_state['curve_severity'] = smoothed

            # ── Steering ──────────────────────────────────────────
            if bottom is not None:
                bx, _    = bottom
                error    = bx - frame_center
                delta    = error - last_error

                if smoothed == 2:
                    # ── SHARP CURVE ──────────────────────────────
                    # Slow way down; steer hard toward curvature direction
                    target_speed = int(max_spd * SPEED_SHARP)
                    motor_adjust = curvature * 1.5

                elif smoothed == 1:
                    # ── MILD CURVE ───────────────────────────────
                    # Moderate speed; PD control is enough
                    target_speed = int(max_spd * SPEED_MILD)
                    motor_adjust = (KP * error) + (KD * delta)

                else:
                    # ── STRAIGHT ─────────────────────────────────
                    target_speed = int(max_spd * SPEED_STRAIGHT)
                    motor_adjust = (KP * error) + (KD * delta)

                last_error = error

                # Remember which way the line was curving for recovery
                if top is not None:
                    last_seen_direction = top[0] - bx
                elif mid is not None:
                    last_seen_direction = mid[0] - bx

            else:
                # ── LINE LOST ─────────────────────────────────────
                # Slow to recovery speed; spin toward last known curve direction
                target_speed = int(max_spd * SPEED_LOST)
                sign = 1 if last_seen_direction > 0 else -1
                motor_adjust = sign * max_spd * 0.8

        # ── Speed smoothing ───────────────────────────────────────
        if current_speed < target_speed:
            current_speed = min(current_speed + ACCEL_RATE, target_speed)
        elif current_speed > target_speed:
            current_speed = max(current_speed - BRAKE_RATE, target_speed)

        # ── Final motor values ────────────────────────────────────
        left  = safe_motor(current_speed + motor_adjust)
        right = safe_motor(current_speed - motor_adjust)

        if ser:
            ser.write(f"{left},{right}\n".encode())

        # ── Visualisation ─────────────────────────────────────────
        # 1. Centroid dots + connecting green lines on the live frame
        points = [p for p in [bottom, mid, top] if p is not None]

        for i, pt in enumerate(points):
            col = DOT_COLOURS[i] if i < len(DOT_COLOURS) else (0, 255, 0)
            cv2.circle(frame, pt, 8,  col,          -1)   # filled dot
            cv2.circle(frame, pt, 11, (255,255,255),  2)  # white ring

        for i in range(len(points) - 1):
            cv2.line(frame, points[i], points[i+1], (0, 255, 0), 2)

        # 2. Vertical centre reference line
        cv2.line(frame, (frame_center, 0), (frame_center, h), (255, 0, 0), 1)

        # 3. Curve severity label
        sev = robot_state['curve_severity']
        cv2.putText(frame, SEV_LABEL.get(sev, ''),
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    SEV_COLOUR.get(sev, (255,255,255)), 2)

        # 4. Speed / motor readout
        cv2.putText(frame, f"Spd:{current_speed}  L:{left}  R:{right}",
                    (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 0), 1)

        # 5. Obstacle distance
        cv2.putText(frame, f"Dist: {robot_state['obstacle_distance']} cm",
                    (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 128, 255), 1)

        # ── Mask view (right panel) ───────────────────────────────
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Draw region boundary lines on mask panel
        cv2.line(mask_bgr, (0, bot_y), (w, bot_y), DOT_COLOURS[0], 1)
        cv2.line(mask_bgr, (0, mid_y), (w, mid_y), DOT_COLOURS[1], 1)
        cv2.line(mask_bgr, (0, top_y), (w, top_y), DOT_COLOURS[2], 1)

        # Label the bands
        cv2.putText(mask_bgr, "BOTTOM", (2, bot_y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DOT_COLOURS[0], 1)
        cv2.putText(mask_bgr, "MID",    (2, mid_y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DOT_COLOURS[1], 1)
        cv2.putText(mask_bgr, "TOP",    (2, top_y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DOT_COLOURS[2], 1)

        combined = np.hstack((frame, mask_bgr))

        ok, buffer = cv2.imencode('.jpg', combined)
        if not ok:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               buffer.tobytes() + b'\r\n')


# ================================================================
#  FLASK ROUTES
# ================================================================
@app.route('/video_feed')
def video_feed():
    return Response(
        gen_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/toggle')
def toggle():
    robot_state['running'] = not robot_state['running']
    return {"running": robot_state['running']}

@app.route('/speed')
def speed():
    val = request.args.get('val', 120)
    robot_state['speed_limit'] = int(val)
    return {"ok": True}

@app.route('/status')
def status():
    return {
        "running"  : robot_state['running'],
        "distance" : robot_state['obstacle_distance'],
        "curve"    : robot_state['curve_severity'],
    }

@app.route('/')
def index():
    return """<!DOCTYPE html>
<html>
<head>
  <title>Line Follower Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', Arial, sans-serif;
      background: #0d0d0d; color: #eee;
      display: flex; flex-direction: column;
      align-items: center; padding: 20px; gap: 16px;
    }
    h1 { font-size: 1.4rem; color: #fff; }
    img  { width: 90%; max-width: 700px; border: 2px solid #333; border-radius: 8px; }
    .controls { display: flex; flex-wrap: wrap; gap: 12px; justify-content: center; }
    button {
      padding: 10px 28px; font-size: 15px; border: none;
      border-radius: 6px; cursor: pointer; font-weight: bold;
    }
    #btn-toggle { background: #28a745; color: #fff; }
    .badge {
      padding: 7px 18px; border-radius: 6px;
      font-weight: bold; font-size: 0.9rem;
    }
    #badges { display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; }
    label { font-size: 0.9rem; color: #aaa; }
    input[type=range] { width: 280px; }
  </style>
</head>
<body>
  <h1>&#129302; Line Follower Dashboard</h1>
  <img src="/video_feed" alt="Camera feed">

  <div class="controls">
    <button id="btn-toggle"
      onclick="fetch('/toggle').then(r=>r.json()).then(d=>{
        this.style.background = d.running ? '#dc3545':'#28a745';
        this.textContent = d.running ? 'STOP':'START';
      })">START</button>
  </div>

  <label>Speed Limit: <strong id="spd-val">120</strong></label>
  <input type="range" min="50" max="200" value="120"
    oninput="document.getElementById('spd-val').textContent=this.value;
             fetch('/speed?val='+this.value)">

  <div id="badges">
    <span id="b-run"   class="badge" style="background:#555">STOPPED</span>
    <span id="b-dist"  class="badge" style="background:#17a2b8">Dist: --</span>
    <span id="b-curve" class="badge" style="background:#28a745">STRAIGHT</span>
  </div>

  <script>
    const CURVE_LABELS  = ['STRAIGHT','MILD CURVE','SHARP CURVE','LINE LOST'];
    const CURVE_COLOURS = ['#28a745','#e6a817','#dc3545','#6f42c1'];

    setInterval(() => {
      fetch('/status').then(r=>r.json()).then(d => {
        document.getElementById('b-run').textContent  = d.running ? 'RUNNING':'STOPPED';
        document.getElementById('b-run').style.background = d.running ? '#28a745':'#555';
        document.getElementById('b-dist').textContent = 'Dist: ' + d.distance + ' cm';
        const c = d.curve || 0;
        document.getElementById('b-curve').textContent   = CURVE_LABELS[c];
        document.getElementById('b-curve').style.background = CURVE_COLOURS[c];
      });
    }, 400);
  </script>
</body>
</html>"""


# ================================================================
#  RUN
# ================================================================
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
