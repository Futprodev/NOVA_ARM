# NOVA 3-DOF Arm (ESP32 + micro-ROS)

This repository contains firmware and ROS 2 nodes for the NOVA 3-DOF robotic arm, controlled by an ESP32 running micro-ROS.

---

## 1. Hardware Overview

- **MCU**: ESP32 (micro-ROS client)
- **Joints**:
  - Base: stepper + AS5600
  - Shoulder: stepper + AS5600
  - Elbow: stepper + AS5600
- **Gripper**: Servo on `SERVOPIN`
- **Homing**: 3 limit switches  
  - `LIM_A` – Shoulder  
  - `LIM_B` – Base  
  - `LIM_C` – Elbow
- **Gear ratios**
  - Base: `BASE_GEAR_RATIO` (from `config.hpp`)
  - Shoulder: `16:1`
  - Elbow: `16:1`

---

## 2. Firmware Behavior (ESP32)

On boot:

1. Configure I2C, stepper drivers, encoders, gripper servo.
2. Connect to micro-ROS agent over serial.
3. **Automatic homing**:
   - Each joint moves toward its limit switch.
   - Encoder angle is latched to the configured home angle in `*_LIMITS`.
4. **Post-homing**:
   - Current pose is held (no automatic move to (0,0,0)).
5. **Sequenced moves** for new joint targets:
   1. Shoulder & elbow → 0°
   2. Base → `target_base`
   3. Shoulder & elbow → `target_sh`, `target_el`

Gripper:

- Controlled by integer command:
  - `0` → close
  - `1` → open

---

## 3. micro-ROS Interface

### Node

- Node name: `nova_arm_hw`

### Published Topics

- **`/arm_joint_states`** (`sensor_msgs/JointState`)
  - `name = ["base_joint","shoulder_joint","elbow_joint"]`
  - `position` = joint angles **in radians**

### Subscribed Topics

- **`/nova_arm/command_deg`** (`std_msgs/Float64MultiArray`)
  - `data = [base_deg, shoulder_deg, elbow_deg]` (hardware frame, degrees)
  - Triggers sequenced move (SH/EL → 0 → BASE → SH/EL targets).

- **`/nova_arm/gripper`** (`std_msgs/Int32`)
  - `0` = close gripper  
  - `1` = open gripper  

---

## 4. micro-ROS Agent (PC / WSL)

On Linux / WSL:

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash

# check USB device
ls /dev/ttyACM*

# start agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
