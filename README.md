# Pioneer 3-DX ROS2 Packages

ROS2 workspace for controlling and simulating the Pioneer 3-DX mobile robot platform.

## Packages

**ros2aria** - ROS2 driver for Pioneer 3-DX hardware using the AriaCoda library. Handles robot communication, odometry, velocity commands, and battery monitoring.

**p3dx_description** - Robot model files (URDF/Xacro), 3D meshes, and launch files for Gazebo simulation and RViz visualization.

## Prerequisites

- Ubuntu 22.04 or compatible Linux
- ROS2 (Humble/Iron/Jazzy)
- Build tools: `colcon`, `cmake`

Install ROS2 dependencies:
```bash
sudo apt install ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-ros-gz \
  python3-colcon-common-extensions
```

## Installation

### 1. Install AriaCoda Library

```bash
cd ~
git clone https://github.com/reedhedges/AriaCoda.git
cd AriaCoda
make -j4
sudo make install
```

### 2. Build Workspace

```bash
cd ~/p3dx_ws
colcon build --symlink-install
source install/setup.bash
```

## Connecting to the Robot

### Hardware Connection

Connect the robot to your computer via:
- USB-to-Serial adapter (typically `/dev/ttyUSB0`)
- Direct serial port (`/dev/ttyS0`)
- Network connection (`hostname:port`)

Grant serial port permissions:
```bash
sudo chmod 666 /dev/ttyUSB0
```

Or permanently add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Running the Driver

Basic usage (default USB on /dev/ttyUSB0):
```bash
ros2 run ros2aria ros2aria_run
```

With custom parameters:
```bash
# Custom serial port
ros2 run ros2aria ros2aria_run --ros-args -p serial_port:=/dev/ttyUSB1

# Network connection
ros2 run ros2aria ros2aria_run --ros-args -p serial_port:=192.168.1.100:8101

# Custom baud rate
ros2 run ros2aria ros2aria_run --ros-args -p serial_baud:=115200
```

### Teleoperation

**Real Robot:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Simulation:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/p3dx/cmd_vel
```

Use `i/j/k/l` keys to control the robot. Press `q/z` to adjust speed.

## Simulation

Launch Gazebo simulation:
```bash
ros2 launch p3dx_description gz.launch.py
```

Launch RViz visualization:
```bash
ros2 launch p3dx_description rviz.launch.py
```

## Topics Reference

### Real Robot Topics (ros2aria)

**Published:**
- `/pose` (nav_msgs/Odometry) - Robot odometry
- `/motors_state` (std_msgs/Bool) - Motor status
- `/battery_state_of_charge` (std_msgs/Float64) - Battery voltage
- `/recharge_state` (std_msgs/Int8) - Charging status
- `/tf` - Transform from odom to base_link

**Subscribed:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Parameters:**
- `serial_port` (default: `/dev/ttyUSB0`) - Serial port or host:port for TCP
- `serial_baud` (default: 0/auto) - Baud rate
- `frame_id_odom` (default: `odom`)
- `frame_id_base_link` (default: `base_link`)

### Simulation Topics (Gazebo)

**Published:**
- `/p3dx/odom` (nav_msgs/Odometry) - Simulated odometry
- `/p3dx/scan` (sensor_msgs/LaserScan) - Laser scanner data
- `/p3dx/imu` (sensor_msgs/Imu) - IMU sensor data
- `/p3dx/joint_states` (sensor_msgs/JointState) - Joint positions and velocities
- `/p3dx/kinect_camera` (sensor_msgs/Image) - Kinect RGB image
- `/p3dx/kinect_camera/camera_info` (sensor_msgs/CameraInfo) - Kinect camera parameters
- `/p3dx/kinect_camera/points` (sensor_msgs/PointCloud2) - Kinect point cloud
- `/p3dx/stereo_camera/left/image_raw` (sensor_msgs/Image) - Left stereo image
- `/p3dx/stereo_camera/left/camera_info` (sensor_msgs/CameraInfo) - Left camera info
- `/p3dx/stereo_camera/right/image_raw` (sensor_msgs/Image) - Right stereo image
- `/p3dx/stereo_camera/right/camera_info` (sensor_msgs/CameraInfo) - Right camera info
- `/robot_description` (std_msgs/String) - URDF model
- `/tf` & `/tf_static` - Transform tree

**Subscribed:**
- `/p3dx/cmd_vel` (geometry_msgs/Twist) - Velocity commands

## Troubleshooting

**Cannot connect to robot**
- Verify serial port: `ls -l /dev/ttyUSB*`
- Check permissions: `sudo chmod 666 /dev/ttyUSB0`
- Ensure robot is powered on
- Try different baud rates if needed

**Robot not moving**
- Check motor state: `ros2 topic echo /motors_state`
- Verify commands are received: `ros2 topic echo /cmd_vel`
- Note: Robot stops after 0.6s without velocity commands (safety watchdog)

**Build errors**
- Verify AriaCoda installation: `ls /usr/local/include/Aria/Aria.h`
- Clean build: `rm -rf build install log && colcon build`

## Monitoring

**Real Robot:**
```bash
# Monitor odometry
ros2 topic echo /pose

# Check battery voltage
ros2 topic echo /battery_state_of_charge

# View node info
ros2 node info /ros2aria_run
```

**Simulation:**
```bash
# Monitor odometry
ros2 topic echo /p3dx/odom

# View laser scan
ros2 topic echo /p3dx/scan

# Check IMU data
ros2 topic echo /p3dx/imu

# View camera images
ros2 run rqt_image_view rqt_image_view
```

**General:**
```bash
# List all topics
ros2 topic list

# View TF tree
ros2 run tf2_tools view_frames
```

## License

GPL-3.0-only
