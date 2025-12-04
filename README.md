### Preliminary
1. You should setup linux environment and ROS1 before this.
2. How to setup Unreal, Airsim, QGroundControl, PX4 and Cesium. [[Youtube](https://www.youtube.com/watch?v=jJ4mqo4Ge8U)]
3. `mv settings.json ~/Documents/Airsim/`

#### Base Setting
- Terminal 1
```bash
cd UnrealEngine-4.27
Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject 
```

- Terminal 2
```bash
./QGroundControl.AppImage 
```

- Terminal 3
```bash
cd AirSim/ros
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch
```

- Terminal 4
```bash
cd PX4-Autopilot
PX4_SIM_MODEL=none_iris ./build px4_sitl_default/bin/px4 -i 0
```

- Terminal 5
```bash
./QGroundControl.AppImage
ROS_NAMESPACE=observe roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580 tgt_system:=1
```

- (Optional) Terminal 6
```bash
cd PX4-Autopilot
PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 -i 1
```

- (Optional) Terminal 7
```bash
./QGroundControl.AppImage
ROS_NAMESPACE=drone1 roslaunch mavros px4.launch fcu_url:=udp://:14541@127.0.0.1:14581 tgt_system:=2
```

[Note] If you want to add more drone, you should modify settings.json and repeat Terminal 6 and 7 within other terminals.

### Install

```bash
$ sudo apt update
$ sudo apt-get install ros-noetic-cv-bridge
$ sudo apt-get install ros-noetic-vision-msgs
$ sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
$ sudo apt-get install ros-noetic-image-common
$ sudo apt-get install ros-noetic-std-msgs ros-noetic-sensor-msgs
$ sudo apt-get install python3-rospy python3-numpy python3-opencv
$ pip3 install flask pandas scikit-learn
```

### Structure
```css
Drone-Observation/
 ├─ src/
 │   ├─ CMakeLists.txt
 │   └─ drone/
 │        ├─ scripts/
 │        │    ├─ node_yolo.py
 │        │    ├─ node_gps.py
 │        │    ├─ node_server.py
 │        │    └─ node_control.py
 │        ├─ launch/
 │        │    └─ multi.launch
 │        ├─ src/
 │        │    └─ template.html
 │        ├─ package.xml
 │        └─ CMakeLists.txt
 ├─ build/
 └─ devel/

```

### How to use
```bash
$ cd Drone-Observation
$ catkin_make
$ source devel/setup.bash
$ roslaunch drone multi.launch
```

### Parameters in launch
| Parameter        | Value                         | Description |
|------------------|-------------------------------|-------------|
| `BASE_DIR`       | `~/mission_data`              | Mission data save directory |
| `IMAGE_TOPIC`    | `/airsim_node/Observe/down_camera/Scene` | Downward camera image topic |
| `GPS_TOPIC`      | `/airsim_node/Observe/gps/Gps` | GPS data topic |
| `STATE_TOPIC`    | `/observe/mavros/state`       | MAVROS state topic |
| `BBOX_TOPIC`     | `/yolo/detections`            | YOLO detection output topic |
| `VIS_TOPIC`      | `/yolo/vis_image`             | YOLO bounding box visualization topic |
| `MODEL_NAME`      | `yolov8x-worldv2.pt`        | YOLO-World model name |
| `CUSTOM_CLASSES`  | `["person", "human", "pedestrian", "soldier"]` | Target classes |
| `CONF_THRESHOLD`  | `0.2`                       | Minimum confidence threshold |
| `MAX_BBOX_RATIO`  | `0.05`                      | Maximum allowed bounding box ratio (area / image) |
| `MIN_SAMPLES`   | `5`     | Minimum number of detections required |
| `EPS_METERS`    | `5.0`   | Clustering distance threshold (meters) |
| `DRONE_IDS`   | `["drone1", "drone2"]`    | Drones managed by fleet controller |


### Node List
| Node Name      | Launch File Entry | Description |
|----------------|------------------|-------------|
| `node_yolo`    | `node_yolo.py`   | YOLO detection + visualization |
| `node_analyze`  | `node_analyze.py` | Summary generation + detection clustering |
| `node_server`  | `node_server.py` | API server for mission data |
| `node_manager` | `node_manager.py` | Fleet manager handling drones |