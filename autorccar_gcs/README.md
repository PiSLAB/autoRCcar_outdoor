# Ground Control System

## Feature
### Vehicle Position
Plot the real-time position of the vehicle.
<p align="center"> <img src="images\gcs_position.png" width="500" height="300"/> </p>

### Status Monitoring
Monitoring the real-time status(position, velocity, attitude)  of the vehicle.
<p align="center"> <img src="images\gcs_monitoring.png" width="500" height="300"/> </p>

### Set Goal
Send the goal point to the vehicle.
- Mouse-click a point on the graph and set it as your destination.
- Even if you enter the target coordinates and press the `Set Goal`, it is set as the destination.
<p align="center"> <img src="images\gcs_set_goal.png" width="500" height="300"/> </p>

### Set Command
Send the command to the vehicle.
- `Manual` (When you want to change to keyboard control)
- `Start` (Motor On)
- `Stop` (Motor Off)

<p align="center"> <img src="images\gcs_cmd.png" width="500" height="300"/> </p>

### Set Yaw
The initial yaw of the vehicle can be set, when the vehicle's heading is unknown.
- A pop-up window will appear if you click `Set Yaw`.
- Enter the angle value [degree] and press the `OK`.

<p align="center"> <img src="images\gcs_set_yaw.png" width="500" height="300"/> </p>

## Build
### Remote PC
```bash
cd ~/ros2_ws/src/autoRCcar_outdoor/autorccar_gcs
pip3 install -r requirement.txt

cd ~/ros2_ws
colcon build --symlink-install --packages-select autorccar_gcs
source install/setup.bash
```
## Run
```bash
ros2 run autorccar_gcs autorccar_gcs
```
