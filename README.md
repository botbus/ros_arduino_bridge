# ROS Arduino Bridge

## Overview
The `ros_arduino_bridge` package provides an interface between ROS (Robot Operating System) and an Arduino-based microcontroller for robot control and sensor integration. It enables communication between a ROS-powered system and an Arduino board using serial communication, facilitating the use of motor controllers, sensors, and actuators in robotic applications.

## Features
- Serial communication between ROS and Arduino
- Support for differential drive robots
- Encoder-based odometry feedback
- Sensor integration (IMU, ultrasonic, IR, etc.)
- PID-based motor control
- Configurable parameters via ROS parameter server

## Installation
### Prerequisites
Ensure that your system has the necessary dependencies installed:

```bash
sudo apt update && sudo apt install -y ros-$ROS_DISTRO-rosserial ros-$ROS_DISTRO-rosserial-arduino
```

### Clone and Build
1. Navigate to your ROS workspace and clone the repository:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/botbus/ros_arduino_bridge.git
   ```
2. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

## Hardware Setup
### Supported Arduino Boards
- Arduino Uno
- Arduino Mega
- Other ATmega-based boards

### Connecting the Arduino
- Connect the Arduino to the host computer via USB.
- Ensure the correct port is used (`/dev/ttyUSB0` or `/dev/ttyACM0`).
- Adjust baud rates and configurations in the `config/arduino_params.yaml` file.

### Upload Firmware to Arduino
1. Install the Arduino IDE and `rosserial` libraries:
   ```bash
   sudo apt install arduino
   ```
2. Open the `ros_arduino_firmware` sketch in the Arduino IDE (found in `ros_arduino_bridge/arduino/`).
3. Compile and upload it to the board.

## ROS Configuration
### Launch Files
Use the provided launch files to start the bridge:
```bash
roslaunch ros_arduino_bridge arduino.launch
```

### Parameters
Modify `config/arduino_params.yaml` to configure:
- Baud rate
- Motor driver settings
- PID parameters
- Sensor types and pins

### Topics
#### Published Topics
- `/odom` (nav_msgs/Odometry): Provides odometry feedback
- `/sensor_state` (sensor_msgs/JointState): Reports sensor readings

#### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Receives velocity commands
- `/servo` (std_msgs/UInt16): Controls servo motors

## Troubleshooting
### Common Issues
#### Serial Connection Fails
- Ensure the correct USB port is used.
- Check permissions (`sudo chmod 666 /dev/ttyUSB0`).

#### No Odometry Feedback
- Verify encoder connections.
- Check PID settings.

## Contributing
Pull requests and bug reports are welcome. Follow standard ROS C++ and Python coding conventions.

## License
This project is licensed under the MIT License.

