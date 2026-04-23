# Step-by-Step Installation & Execution Guide

Follow these steps to configure your Raspberry Pi, flash your Arduino, and run the robot.

## Step 1: Prepare the Raspberry Pi
1. Install **Raspberry Pi OS** (Bullseye or Bookworm) on your Pi 4.
2. Ensure you have SSH enabled or a monitor/keyboard connected.
3. Update your system packages:
   ```bash
   sudo apt update && sudo apt upgrade -y
   
## Step 2: Grant Serial Permissions
To allow the Python script to send data to the Arduino via USB, add your Pi user to the dialout group:

Bash
sudo usermod -aG dialout $USER
(Note: You must reboot your Raspberry Pi for this permission to take effect).

## Step 3: Install Software Dependencies
Clone this repository to your Raspberry Pi:

Bash
git clone [https://github.com/YOUR_USERNAME/Advanced-Vision-Line-Follower.git](https://github.com/YOUR_USERNAME/Advanced-Vision-Line-Follower.git)
cd Advanced-Vision-Line-Follower
Install the required Python libraries. On newer Raspberry Pi OS versions (Bookworm), it is recommended to use the system package manager for OpenCV and Serial:

Bash
sudo apt install python3-opencv python3-flask python3-serial -y
(Alternatively, you can use pip3 install -r requirements.txt --break-system-packages if you prefer pip).

## Step 4: Flash the Arduino
Open the Arduino IDE on your main computer.

Open the arduino_muscle/arduino_muscle.ino file.

Connect your Arduino Nano via USB.

Select Arduino Nano under Tools > Board.

Click Upload.

Once uploaded, disconnect the Arduino from your computer and plug it into the Raspberry Pi.

## Step 5: Start the Robot
Power on your motor battery (connected to the Cytron driver).

On the Raspberry Pi, navigate to the project folder and run the brain script:

Bash
python3 python_brain/follower.py
You should see a terminal message confirming: ✅ Arduino connected on /dev/ttyUSB0.

Open a web browser on any device connected to the same WiFi network and go to:
http://192.168.137.184:5000

Place the robot on the black line and click START ROBOT in the web dashboard!
