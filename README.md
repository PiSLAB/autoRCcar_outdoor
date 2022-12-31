# autoRCcar_outdoor

<p align="center"> <img src="images\autoRCcar_test_video.gif" width="500" height="300"/> </p>

## System
### Hardware
<p1 align="center"> <img src="images\autoRCcar.jpg" width="400" height="200"/> </p1>
- 1/8 VRX RH818 (RC Car)
- NVIDIA Jetson Nano
  - ubuntu 20.04 + ROS2 Galactic
- SparkFun GPS-16355 (ublox ZED-F9R)
- ESP32
### Software

<p2 align="center"> <img src="images\ros2_architect.jpg" width="400" height="200"/> </p2>
## Depenency
- [ROS2](https://docs.ros.org/en/galactic/index.html) (Galactic)
- [Eigen](https://eigen.tuxfamily.org/)
- [PyQt](https://pypi.org/project/PyQt5/)　<< GCS
## Packages
```bash
autoRCcar_outdoor
  ├── autorccar_launch : autoRCcar packages lunch file
  │
  ├── autorccar_interfaces : Custom interface messages
  ├── autorccar_ubloxf9r : Sensor DAQ (GNSS & IMU)
  ├── autorccar_navigation : GNSS/INS EKF
  ├── autorccar_path_planning : Bezier curve
  ├── autorccar_control : Pure pursuit
  │
  ├── autorccar_esp32 : PWM generator (Arduino IDE)
  │
  ├── autorccar_keyboard : Manual control
  └── autorccar_gcs : Command & Monitoring
```
## Build
### RC car
**Check point)** Path setting for autorcar_navigation/config.yaml file is required.
```bash
~/autoRCcar_outdoor/autorccar_navigation/src/main_ekf.cpp
```
```C++
std::string config = "/home/{User_Name}/{ROS_Workspace}/src/autoRCcar_outdoor/autorccar_navigation/config.yaml";
```

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
If there is a custom messages error, proceed with the `autorccar_interfaces` build first.
```bash
colcon build --symlink-install --packages-select autorccar_interfaces
source install/setup.bash
```
### (Optional) Remote PC
```bash
cd ~/ros2_ws/src/autoRCcar_outdoor/autorccar_gcs
pip3 install -r requirement.txt

cd ~/ros2_ws
colcon build --symlink-install --packages-select autorccar_gcs autorccar_keyboard
source install/setup.bash
```
## Application
### Autonomous Driving
```bash
ros2 run autorccar_launch launch.py
```
If you want to run an individual package,
```bash
ros2 run autorccar_ubloxf9r ubloxf9r
ros2 run autorccar_navigation gnss_ins
ros2 run autorccar_path_planning bezier_curve
ros2 run autorccar_control pure_pursuit
```
### GCS
```bash
ros2 run autorccar_gcs autorccar_gcs
```
### (optional) Keyboard Control
```bash
ros2 run autorccar_keyboard keyboard_control
```
## Reference
- Control) https://github.com/AtsushiSakai/PythonRobotics
- Keyboard) https://github.com/ros2/teleop_twist_keyboard
## FutureWork
- Waypoint (Reset is required to go to the updated goal point)
- Indoor navigation (SLAM)
