# Advanced Computer Vision Line Follower Robot

A professional-grade, dual-microcontroller robotics project utilizing OpenCV, Proportional-Derivative (PD) control, and a web-based dashboard for real-time monitoring and control.

## 🧠 System Architecture
This robot splits its processing between two boards to ensure high frame rates and precise motor control:
* **The Brain (Raspberry Pi 4):** Captures video via an HP USB camera, processes frames using OpenCV adaptive thresholding, calculates centroid error, runs the PD control loop, and hosts a Flask web server for UI.
* **The Muscle (Arduino Nano):** Mounted on a custom breakout board, it receives target Left/Right wheel speeds via Serial (115200 baud) and executes differential steering through a high-power Cytron MDD10A motor driver.

## ✨ Key Features
* **Adaptive Computer Vision:** Uses `cv2.adaptiveThreshold` to ignore global room lighting and detect lines dynamically based on local floor contrast.
* **Kinematic Smoothing:** Implements an acceleration/braking speed gradient to prevent wheel slip and jerky movements.
* **Proportional-Derivative (PD) Control:** Calculates steering angle based on current error and rate of change, resulting in smooth, sweeping turns.
* **Corner Memory:** If the robot encounters a sharp 90-degree corner and loses the line, it references the `last_known_error` to commit to a hard-spin search until the line is re-acquired.
* **Live Web Dashboard:** A Flask-hosted UI featuring a dual-screen camera view (Raw vs B&W Mask), a Start/Stop safety kill switch, and a live top-speed tuning slider.

## 🛠️ Hardware Specifications
This specific build utilizes the following hardware components:
* **SBC:** Raspberry Pi 4
* **Microcontroller:** Arduino Nano (mounted on a custom PCB shield)
* **Motor Driver:** Cytron MDD10A Rev 2.0 (Dual Channel 10A DC Motor Driver)
* **Motors:** 2x DC Gear Motors with Encoders and high-traction rubber tires
* **Vision:** HP USB Web Camera
* **Chassis:** Custom dual-deck chassis structure

## 🚀 How to Run

1. **Flash the Arduino:**
   Upload `arduino_muscle/arduino_muscle.ino` to your Arduino Nano using the Arduino IDE.
   
2. **Setup the Raspberry Pi:**
   SSH into your Raspberry Pi 4 and clone this repository.
   ```bash
   git clone [https://github.com/YOUR_USERNAME/Advanced-Vision-Line-Follower.git](https://github.com/YOUR_USERNAME/Advanced-Vision-Line-Follower.git)
   cd Advanced-Vision-Line-Follower
