# autoRCcar_outdoor

<p align="center"> <img src=".tmp\autoRCcar_test_video.gif" width="400" height="200"/> </p>  

## System
### Hardware
### Software

## Depenency
- ROS2 (Galactic)
- [Eigen](https://eigen.tuxfamily.org/)
- [PyQt](https://pypi.org/project/PyQt5/)　<< GCS
## Packages
```bash
├── autorccar_launch : autoRCcar packages lunch
│
├── autorccar_interfaces : Custom messages
├── autorccar_ubloxf9r : Sensor DAQ (GNSS & IMU)
├── autorccar_navigation : GNSS/INS EKF
├── autorccar_path_planning : Bezier curve
├── autorccar_control : Pure pursuit
│
├── autorccar_esp32 : PWM generator (Arduino IDE)
│
├── autorccar_keyboard : Manual control
├── autorccar_gcs : Command & Monitoring
│
└── run.sh : package excutable
```
## Build
### RC car
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
cd ~/ros2_ws/src/autorccar_outdoor/autorccar_gcs
pip3 install -r requirement.txt

cd ~/ros2_ws
colcon build --symlink-install --packages-select autorccar_keyboard autorccar_gcs
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

## Contributors
[GS Park](p),　[JH Lee](l),　[JH Bae](b),　[HJ Son](s)

[p]:https://github.com/gspark87
[l]:https://github.com/lee90108
[b]:https://github.com/luke7637
[s]:https://github.com/dlfksj
