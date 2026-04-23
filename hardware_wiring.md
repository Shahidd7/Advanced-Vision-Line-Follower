# Hardware Setup & Wiring Guide

This guide details the physical components and wiring required to build the Advanced Vision Line Follower.

## 📦 Bill of Materials (BOM)
* **Raspberry Pi 4 Model B** (with SD card and power supply)
* **Arduino Nano** (mounted on a custom breakout board)
* **Cytron MDD10A Rev 2.0** (Dual Channel 10A Motor Driver)
* **2x DC Gear Motors** (with built-in encoders, though encoders are not used in this specific software version)
* **HP USB Web Camera**
* **11.1V 3S LiPo Battery** (or appropriate power source for your motors)
* **Robot Chassis & Wheels**
* **Jumper Wires & USB Cables**

## 🔌 Wiring Connections

### 1. Arduino to Cytron MDD10A Motor Driver
The Cytron driver uses standard PWM and DIR pins to control motor speed and direction.
* **Arduino D9 (PWM)** -> Cytron `PWM1`
* **Arduino D8 (DIR)** -> Cytron `DIR1`
* **Arduino D10 (PWM)** -> Cytron `PWM2`
* **Arduino D7 (DIR)** -> Cytron `DIR2`
* **Arduino GND** -> Cytron `GND` (Crucial: Microcontroller and motor driver must share a common ground)

### 2. Cytron MDD10A to Motors & Power
* **Cytron `M1A` & `M1B`** -> Left Motor Terminals
* **Cytron `M2A` & `M2B`** -> Right Motor Terminals
* **Cytron `POWER` (B+ / B-)** -> 11.1V LiPo Battery

### 3. Raspberry Pi Connections
* **USB Port 1:** Connect the HP USB Web Camera.
* **USB Port 2:** Connect the Arduino Nano via a USB Mini-B cable. (This handles both power to the Arduino and the 115200 baud serial communication).
