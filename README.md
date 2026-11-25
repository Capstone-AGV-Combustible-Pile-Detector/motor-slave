# Dhuro Omnibot ROS 2 (Slave Firmware & Bridge)

This folder contains the dedicated ROS 2 firmware for the **Dhuro Omnibot (ESP32)** and the companion **ROS 2 Humble Python Bridge Node** for the **Raspberry Pi 5**.

---

## Architecture Overview

```
[ Raspberry Pi 5 (ROS 2 Humble) ]
       │
       │  Subscribes: /cmd_vel (geometry_msgs/msg/Twist)
       │  Publishes : /odom (nav_msgs/msg/Odometry) & TF (odom -> base_footprint)
       │
       ▼
[ dhuro_ros2_bridge Node ]
       │
       │  UART-to-USB (e.g. /dev/ttyUSB0 or /dev/ttyACM0 at 115200 baud)
       │  Protocol: Command '*10,yaw,vx,vy,w#' <---> Telemetry '*10,px,py,pz,yaw,vx,vy,wz#'
       │
       ▼
[ ESP32 Omnibot Slave ]
  - Core 0 (RTOS): Serial RX/TX, Watchdog (500ms auto-stop), Telemetry
  - Core 1 (RTOS): 4-Wheel Omnidirectional Kinematics & Closed-Loop Motor PID
  - IMU (MPU6050): Real-time gyro heading integration (yaw) on I2C
  - GPIO Buttons : Hardware Emergency Pause / Resume
```

---

## 1. ESP32 Firmware Setup & Flashing

### Files:
- `Dhuro-Omnibot-ROS2.ino`: Main sketch, boots directly into ROS 2 listening mode (no menus required).
- `robot_imu.*`: MPU6050 I2C driver (gyro Z integration, auto-calibration).
- `robot_motor_encoder.*`: Motor PWM (ESP32 Core 2.x & 3.x compatible) and encoder ISRs.
- `robot_control_kinematics.*`: 4-wheel omnidirectional inverse kinematics and odometry calculations.
- `robot_rtos_handler.*`: FreeRTOS dual-core tasks.
- `robot_serial.*`: Handles USB Serial and Hardware UART2 communication.
- `robot_gpio.*`: Push button pins (Emergency Stop toggle).

### Build & Upload from Windows:
Run the batch script:
```cmd
.\build.bat esp32:esp32:esp32 COM22
```
Or interactively:
```cmd
.\build_and_upload.bat COM22
```
*(Press and hold the **BOOT** button on the ESP32 if upload hangs at `Connecting...`)*.

---

## 2. Raspberry Pi 5 Setup (ROS 2 Humble)

### Physical Connection:
Connect the ESP32 to the Raspberry Pi 5 using a USB cable (or external USB-to-UART module).
Verify the device appears on the Raspberry Pi:
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
Add user permission to access serial port if needed:
```bash
sudo usermod -aG dialout $USER
```

### Option A: Direct Python Execution (No Colcon Build Required)
Inside `ros2_ws/dhuro_ros2_bridge/`:
```bash
python3 dhuro_ros2_bridge/bridge_node.py --ros-args -p serial_port:=/dev/ttyUSB0
```

### Option B: Build with Colcon Workspace
```bash
mkdir -p ~/ros2_ws/src
cp -r ros2_ws/dhuro_ros2_bridge ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select dhuro_ros2_bridge
source install/setup.bash

# Run the node
ros2 run dhuro_ros2_bridge bridge_node --ros-args -p serial_port:=/dev/ttyUSB0
```

---

## 3. Testing and Driving the Robot

### Send Test Velocity Command:
Drive forward at 0.15 m/s:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Rotate in place at 0.5 rad/s:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10
```

Keyboard teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Monitor Odometry Telemetry:
```bash
ros2 topic echo /odom
```

---

## 4. Safety Features
1. **Communication Watchdog**: If no velocity command is received for **500 ms**, the ESP32 automatically halts all 4 motors.
2. **Physical Emergency Button**: Pressing `PB_RUN` or `PB_OK` on the robot immediately locks/halts the motors. Pressing it again resumes operation.
3. **Power-On Protection**: Setpoints and PWM are strictly initialized to `0` and RTOS tasks start suspended until hardware is ready, preventing sudden wheel spin at startup.
