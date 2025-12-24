NOVA 3-DOF Arm – Quick README
1. Hardware Overview

MCU: ESP32 (micro-ROS client)

Joints: Base, Shoulder, Elbow (stepper + AS5600 each)

Gripper: Servo on SERVOPIN

Homing: 3 limit switches LIM_A (SH), LIM_B (BASE), LIM_C (EL)

Gear ratios (current):

Base: BASE_GEAR_RATIO (config)

Shoulder: 16:1

Elbow: 16:1

2. Firmware Behavior

On boot:

ESP32 connects to micro-ROS agent (serial).

Automatic homing:

Each joint moves away from switch, then approaches until switch is hit.

Encoder is latched at hard limit angle from *_LIMITS.

After homing:

Pose is held, no auto move to (0,0,0).

Later motion:

Uses sequenced move when a new target is received:

Shoulder & elbow → 0°

Base → target_base

Shoulder & elbow → target_sh / target_el

Gripper:

Controlled by integer command:

0 → close

1 → open

3. ROS 2 Interface
Node (on ESP32)

Node name: nova_arm_hw

Published

arm_joint_states → sensor_msgs/JointState

name: ["base_joint","shoulder_joint","elbow_joint"]

position: joint angles in rad

Subscribed

nova_arm/command_deg → std_msgs/Float64MultiArray

data = [base_deg, shoulder_deg, elbow_deg] (hardware frame, deg)

Triggers sequenced move (SH/EL→0 → BASE → SH/EL target).

nova_arm/gripper → std_msgs/Int32

0 = close, 1 = open

4. micro-ROS Agent (on WSL / Linux)

Terminal 1 (WSL):

source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash

# check USB
ls /dev/ttyACM*

# run agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200


Keep this running.

5. Basic ROS Testing (PC side)

New terminal:

source /opt/ros/jazzy/setup.bash

# See topics
ros2 topic list

# Echo joint states
ros2 topic echo /arm_joint_states

Send a simple command (e.g. all zeros)
ros2 topic pub /nova_arm/command_deg std_msgs/msg/Float64MultiArray "
layout:
  dim: []
  data_offset: 0
data: [0.0, 30.0, -30.0]
" --once

Gripper test
# Open
ros2 topic pub /nova_arm/gripper std_msgs/msg/Int32 "data: 1" --once

# Close
ros2 topic pub /nova_arm/gripper std_msgs/msg/Int32 "data: 0" --once

6. Hardware IK Node (PC)

Node: nova_arm_planar_ik_hw

Subscribes: planar_goal (Float64MultiArray)

data = [x, y, z] (m, base frame)

Publishes: nova_arm/command_deg

Automatically converts Cartesian (x,y,z) → [base_deg, sh_deg, el_deg].

Example goal:

ros2 topic pub /planar_goal std_msgs/msg/Float64MultiArray "
layout:
  dim: []
  data_offset: 0
data: [0.3, 0.0, 0.25]
" --once

7. Typical Bring-Up Sequence (Real Arm)

Power ESP32 + drivers + 12V.

Start micro-ROS agent on PC.

Reset ESP32 → it will:

Connect to agent

Auto-home using limit switches.

On PC:

ros2 topic echo /arm_joint_states to confirm motion.

Use:

direct ros2 topic pub /nova_arm/command_deg … or

send /planar_goal via IK node.
