# 📘 ROS Sonar SLAM Stack — SLAM Toolbox Edition

## NOTE: THIS IS STILL A WORK IN PROCESS

A complete ROS 2 Jazzy–based sonar‑driven SLAM system for the Arduino UNO Q platform, using:

- A servo‑swept ultrasonic sonar  
- A custom ROS 2 package (`ros_sonar`)  
- **SLAM Toolbox** (online async mode)  
- A hardened ARM64 Docker environment  
- TF2‑corrected `LaserScan` publishing  
- RViz visualization  
- Multi‑machine ROS networking  

This README documents the **package**, **firmware**, **Docker images**, **build process**, and **testing pipeline** end‑to‑end.

## 🧱 1. ROS_SONAR_SLAMTOOLBOX STRUCTURE

The entire application consists of the Arduino Uno Q Bridge Requirement and ROS2 Workspace which is a directory containing ROS 2 packages.

```text
ros_sonar_slamtoolbox
├── Dockerfile.ros_sonar_slam
├── python
│   ├── requirements.txt
│   └── src
├── python_utils
│   └── ros_sonar_py
│       ├── __init__.py
│       ├── app_internal
│       │   ├── core
│       │   │   ├── __init__.py
│       │   │   ├── audio.py
│       │   │   ├── ei.py
│       │   │   └── module.py
│       │   └── pipeline
│       │       ├── __init__.py
│       │       ├── adapter.py
│       │       ├── constants.py
│       │       ├── limiter.py
│       │       ├── pipeline.py
│       │       └── task.py
│       ├── app_utils
│       │   ├── __init__.py
│       │   ├── app.py
│       │   ├── audio.py
│       │   ├── brick.py
│       │   ├── bridge.py
│       │   ├── folderwatch.py
│       │   ├── httprequest.py
│       │   ├── image.py
│       │   ├── jsonparser.py
│       │   ├── ledmatrix.py
│       │   ├── leds.py
│       │   ├── logger.py
│       │   ├── slidingwindowbuffer.py
│       │   ├── tls_cert_manager.py
│       │   ├── userinput.py
│       │   └── utils.py
│       └── sonar
│           ├── __init__.py
│           └── sonar.py
└── ros_ws
    └── src
        └── ros_sonar
            ├── CMakeLists.txt
            ├── config
            │   └── slam_toolbox_params.yaml
            ├── launch
            │   ├── slam_launch.launch.py
            │   └── sonar.launch.py
            ├── msg
            │   ├── Sonar.msg
            │   └── SonarScan.msg
            ├── nodes
            │   ├── __init__.py
            │   └── sonar_node
            ├── package.xml
            └── resource
                └── ros_sonar
```
## 📦 2. DOCKER BUILD FILE FOR SLAM TOOLBOX

The SLAM Toolbox Dockerfile builds off of the Ros2 Jazzy base which includes support for nano and tmux, installs the base SLAM Toolbox without graphical support. The Base image also configures `fastDDS`:

**Dockerfile.ros_sonar_slam**:
```text
# ---------------------------------------------------------
# Base: ROS 2 Jazzy (Ubuntu 24.04)
# ---------------------------------------------------------
FROM ros_jazzy_base

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------
# Fix Ubuntu + ROS keyrings (Noble key rotation)
# ---------------------------------------------------------
RUN apt-get update || true && \
    apt-get install -y --reinstall --no-install-recommends \
        ubuntu-keyring \
        debian-archive-keyring \
        ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# Install dependencies (CMake-based ROS package)
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-colcon-common-extensions git \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rclpy \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-slam-toolbox \
    libboost-thread-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libboost-chrono-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# Create ROS workspace
# ---------------------------------------------------------
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws

# ---------------------------------------------------------
# Python utilities (outside ROS workspace)
# ---------------------------------------------------------
COPY ./python_utils/ros_sonar_py /opt/python_utils/ros_sonar_py
ENV PYTHONPATH="/opt/python_utils:${PYTHONPATH}"

COPY ./python /opt/python_app
RUN pip install -r /opt/python_app/requirements.txt --break-system-packages

# ---------------------------------------------------------
# Copy ROS package (CMake-based)
# ---------------------------------------------------------
COPY ./ros_ws/src/ros_sonar /opt/ros_ws/src/ros_sonar

RUN find /opt/ros_ws -name COLCON_IGNORE -print -delete

RUN echo "=== ros_sonar package inside container ===" && \
    ls -R /opt/ros_ws/src/ros_sonar

# ---------------------------------------------------------
# rosdep
# ---------------------------------------------------------
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y"

# ---------------------------------------------------------
# Build workspace (CMake package)
# ---------------------------------------------------------
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rm -rf build install log && \
    colcon build --symlink-install"

ENV ROS_DOMAIN_ID=0

# ---------------------------------------------------------
# Auto-source workspace
# ---------------------------------------------------------
RUN printf '%s\n' \
"source /opt/ros/jazzy/setup.bash" \
"source /opt/ros_ws/install/setup.bash" \
>> /etc/bash.bashrc

RUN printf '%s\n' \
"source /opt/ros/jazzy/setup.bash" \
"source /opt/ros_ws/install/setup.bash" \
>> /root/.bashrc

CMD ["/bin/bash"]
```
## 📦 3. python Directory
The python directory contains 1 file and directory:
`requirements.txt`: any python specific requirements that you want loaded
`src` directory:  EMPTY
## 📦 4. python_utils Directory
A `ros_sonar_py` directory which contains the python support python scripts.  Under this directory there are 3 subdirectories.
1. `app_utils` and `app_internal`:  These are copies from the Arduino Bricks Library (https://github.com/arduino/app-bricks-py) and modified.  All occurences of `arduino.app_utils` have been replaced by `ros_sonar.app_utils`, i.e., [package name].app_utils.
2. and the `sonar` package for the workspace:
   - `__init__.py`:  which contains a single command `from .sonar import Sonar`
   - and `sonar.py` which acts as the `bridge` interface:
 ```python
 import time
from ros_sonar_py.app_utils import Bridge   # Use the same Bridge instance as LED example


class Sonar:
    def __init__(self, min_angle=0.0, max_angle=180.0, step=5.0, settle_sec=0.05):
        # Use the global Bridge instance (do NOT instantiate your own)
        self.bridge = Bridge

        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.step = float(step)
        self.settle_sec = float(settle_sec)

    def _frange(self, start, stop, step):
        x = start
        while x <= stop + 1e-6:
            yield x
            x += step

    def sweep(self):
        angles = []
        distances = []

        #sweep_start = time.time()
        #print("\n=== Starting sweep ===")

        for angle in self._frange(self.min_angle, self.max_angle, self.step):
            step_start = time.time()

            # ----------------------------------------------------
            # 1. Send servo command
            # ----------------------------------------------------
            #t0 = time.time()
            try:
                self.bridge.call("set_servo_angle", float(angle))
            except Exception as e:
                print(f"[ERROR] set_servo_angle({angle}) failed: {e}")
                print("[WARN] Aborting sweep early due to servo timeout")
                distances.append(float("inf"))
                #continue
                break
            #t1 = time.time()

            # Special case for angle 0
            if angle == 0:
                time.sleep(1.0)

            # Servo settle time
            time.sleep(self.settle_sec)

            # ----------------------------------------------------
            # 2. Read distance
            # ----------------------------------------------------
            #t2 = time.time()
            try:
                dist = self.bridge.call("get_distance_m")
            except Exception as e:
                print(f"[ERROR] get_distance_m() failed at angle {angle}: {e}")
                print("[WARN] Aborting sweep early due to distance timeout")
                break
                #dist = -1.0
            #t3 = time.time()

            # Convert to float
            dist = float(dist) if dist is not None else -1.0

            # ----------------------------------------------------
            # 3. Log timing for this step
            # ----------------------------------------------------
            #print(
            #    f"Angle {angle:6.1f} deg | "
            #    f"servo_rpc={t1 - t0:6.3f}s | "
            #    f"settle={self.settle_sec:5.3f}s | "
            #    f"dist_rpc={t3 - t2:6.3f}s | "
            #    f"step_total={t3 - step_start:6.3f}s"
            #)

            angles.append(angle)
            distances.append(dist)

            # Small pacing delay to avoid RouterBridge queue overload
            time.sleep(0.01)

        sweep_end = time.time()
        #print(f"=== Sweep complete: {sweep_end - sweep_start:.3f} seconds ===\n")

        return angles, distances
   ```
## 📦 5. ROS package: `ros_sonar`
To maintain the ros package structure this directory is created within the ROS2 workspace itself:  
```text  
└── ros_ws
    └── src
        └── ros_sonar  
```
### 📄 `CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.8)
project(ros_sonar)

# Default to C++17 (required by ROS 2 generators)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# -----------------------------
# Message generation
# -----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sonar.msg"
  "msg/SonarScan.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------
# Install Python nodes
# -----------------------------
install(
  DIRECTORY nodes/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# -----------------------------
# Install launch files
# -----------------------------
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# -----------------------------
# Install config files
# -----------------------------
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# -----------------------------
# Install resource index
# -----------------------------
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```
### 📄 `package.xml`
```xml
<?xml version="1.0"?>
<package format="3">
  <name>ros_sonar</name>
  <version>0.0.1</version>
  <description>ROS 2 sonar interface and SLAM integration package.</description>

  <maintainer email="you@example.com">Michael</maintainer>
  <license>Apache-2.0</license>

  <!-- Build system -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Message generation -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- Dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>

  <!-- Launch + config -->
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
## 📄 6. Messages
### `msg/Sonar.msg`

```text
float32 angle
float32 distance
```

### `msg/SonarScan.msg`

```text
float32[] angles
float32[] distances
```

## 📄 7. `nodes/sonar_node.py` 
### `nodes/__init__.py`
```text
#Empty init to make nodes executable
```
### `nodes/sonar`
This is an executable file that if you copy from the repo you probably will have to change manually:
```text
chmod +x sonar_node
```

## 📄 8. Launch files

### 📄 8.1 `launch/sonar.launch.py` (sonar only)
Launches a sonar only node:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_sonar",
            executable="sonar_node.py",
            name="sonar_node",
            output="screen",
            parameters=[{
                "min_angle": 0.0,
                "max_angle": 180.0,
                "step": 2.0,
                "settle_sec": 0.15,
                "period_sec": 6.0,
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.10", "0.0", "0.05",
                "0", "0", "0",
                "base_link",
                "sonar_link",
            ],
        ),
    ])
```
### 8.2 `launch/slam_launch.launch.py` (sonar + SLAM Toolbox)
This is the launch file that implements the slam toolbox messages to the PC:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to SLAM Toolbox parameters
    config = os.path.join(
        get_package_share_directory("ros_sonar"),
        "config",
        "slam_toolbox_params.yaml"
    )

    # Path to SLAM Toolbox bringup launch file
    slam_launch_file = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_sync_launch.py"
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            "slam_params_file": config,
        }.items(),
    )

    return LaunchDescription([

        # ---------------------------------------------------------
        # Sonar Node
        # ---------------------------------------------------------
        Node(
            package="ros_sonar",
            executable="sonar_node",
            name="sonar_node",
            output="screen",
        ),

        # ---------------------------------------------------------
        # TF: base_link -> sonar_link
        # ---------------------------------------------------------
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "sonar_link"],
            name="sonar_tf"
        ),

        # ---------------------------------------------------------
        # TF: odom -> base_link (fake odometry)
        # ---------------------------------------------------------
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            name="fake_odom_tf"
        ),

        # ---------------------------------------------------------
        # SLAM Toolbox (correct bringup)
        # ---------------------------------------------------------
        slam_toolbox,
    ])
       
```


### 📄 10. SLAM Toolbox parameters
`config/slam_toolbox_params.yaml`:
```yaml
slam_toolbox:
  ros__parameters:

    # Disable Lifecycle Manager
    use_lifecycle_manager: false

    # Frames
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    scan_topic: "/scan"

    # No real odometry, rely on scan matching
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.0
    minimum_travel_heading: 0.0

    # Map
    resolution: 0.05
    max_laser_range: 4.0
    minimum_time_interval: 0.0

    # Loop closure
    loop_search_maximum_distance: 2.0
    loop_search_minimum_chain_size: 10
    loop_search_maximum_link_distance: 1.0
    do_loop_closing: true

    # Solver
    solver_plugin: "solver_plugins::CeresSolver"
    ceres_linear_solver: "SPARSE_NORMAL_CHOLESKY"
    ceres_preconditioner: "SCHUR_JACOBI"

    # General
    # use_sim_time: false
```

# 🧩 11. UNO Q MCU firmware

```cpp
#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LIDARLite.h>

/* ---------------- PCA9685 Setup ---------------- */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50

uint16_t USMIN[] = {950, 625};
uint16_t USMAX[] = {1870, 2275};

#define MAP_ROUND(x, in_min, in_max, out_min, out_max) \
  ( ((int64_t)(x) - (int64_t)(in_min)) * ((int64_t)(out_max) - (int64_t)(out_min)) + \
    (((int64_t)(x) - (int64_t)(in_min)) >= 0 ? ((int64_t)(in_max) - (int64_t)(in_min)) / 2 : -((int64_t)(in_max) - (int64_t)(in_min)) / 2) ) \
    / ((int64_t)(in_max) - (int64_t)(in_min)) + (int64_t)(out_min)

int angle2us(uint8_t servonum, uint16_t angle) {
  return MAP_ROUND(angle, 0, 180, USMIN[servonum], USMAX[servonum]);
}

/* ---------------- LIDAR Setup ---------------- */
LIDARLite lidar;

/* ---------------- RouterBridge API ---------------- */
float get_distance_m();
void set_servo_angle(float angle_deg);

void setup() {
  Wire.begin();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  pwm.writeMicroseconds(0, angle2us(0, 0));

  lidar.begin(0, true);
  lidar.configure(0);

  Bridge.begin();

  Bridge.provide("set_servo_angle", set_servo_angle);
  Bridge.provide("get_distance_m", get_distance_m);
}

void loop() {
  // Let the main loop run so safe handlers execute
  delay(1);
}

void set_servo_angle(float angle_deg) {
  if (angle_deg < 0) angle_deg = 0;
  if (angle_deg > 180) angle_deg = 180;

  uint16_t us = angle2us(0, (uint16_t)angle_deg);
  pwm.writeMicroseconds(0, us);
}

float get_distance_m() {
  int cm = lidar.distance();
  if (cm <= 0) return -1.0f;
  return float(cm / 100.0f);
}
```

## 12. Installation

- [ ] Copy the repo to the Arduino Uno Q MPU.
- [ ] Copy the ros-jazzy-base image
- [ ] Build the ros-jazzy-base image using:  
 ```text
 docker build -f Dockerfile.ros_jazzy_base_nano -t ros_jazzy_base .
```
- [ ] Run the following prune command to clear the build cache
```text
docker builder prune -f
```
- [ ] To have enough room on the docker drive you will have to remove the EI-models image - sorry no way around that.  However, after building and testing the slam image you can delete the ros2 base image.  The EI-models image will be restored when you run an app in applab requiring it.
```text
To remove the ei-models image
docker rmi .........

Build the ros2 slam docker image
docker build --no-cache -f Dockerfile.ros_sonar_slam -t ros-sonar-slam .
```

## 🧪 13. Testing and Verification

### 13.1 Start the container

```bash
docker run -it --rm --net=host -v /var/run/arduino-router.sock:/var/run/arduino-router.sock ros-sonar-slam:latest bash
```

Inside:

```bash
source install/setup.bash
```

---

### 13.2 Verify packages

```bash
ros2 pkg list | grep -E "sonar|slam_toolbox"
```

You should see `ros_sonar` and `slam_toolbox`.

---

### 13.3 Run sonar only

```bash
ros2 launch ros_sonar sonar.launch.py
```

Check:

```bash
ros2 topic echo /scan --once
```

---

### 13.4 Run sonar + SLAM Toolbox

```bash
ros2 launch ros_sonar slam_launch.launch.py
```

---

### 13.5 Validate `LaserScan`

```bash
ros2 topic echo /scan --once
```

Must include:

- `header.frame_id: "sonar_link"`
- valid `angle_min`, `angle_max`, `angle_increment`
- valid `ranges`

---

### 13.6 Validate TF

```bash
ros2 topic echo /tf_static
```

Expect:

- `base_link → sonar_link`  
- `odom → base_link`  

Check TF chain:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

(after SLAM Toolbox starts publishing `map → odom`)

---

## 13.7 Check map

```bash
ros2 topic echo /map --once
```

---

## 14. Q -> PC (WSL2) Rviz
### 14.1 Arduino Q Config network communication to the PC
The `ros-jazzy-base` image pre-configures to use fastDDS:  
```xml
RUN printf "<profiles>\n\
  <transport_descriptors>\n\
    <transport_descriptor>\n\
      <transport_id>udp_transport</transport_id>\n\
      <type>UDPv4</type>\n\
    </transport_descriptor>\n\
  </transport_descriptors>\n\
  <participant profile_name=\"unicast_profile\" is_default_profile=\"true\">\n\
    <rtps>\n\
      <builtin>\n\
        <initialPeersList>\n\
          <locator>\n\
            <udpv4>\n\
              <address>192.168.1.XXX</address>\n\
              <port>7400</port>\n\
            </udpv4>\n\
          </locator>\n\
        </initialPeersList>\n\
      </builtin>\n\
    </rtps>\n\
  </participant>\n\
</profiles>" > /root/.fastdds_uno.xml
```
Before building the base image make sure you change you pc address:  
```
              <address>192.168.1.XXX</address>\n\
 ```
 
 It also configure the environment variables:  
```text
# -------------------------------
# FastDDS environment variables
# -------------------------------
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.fastdds_uno.xml
ENV ROS_DOMAIN_ID=0
```
### 14.2 PC(WSL2) Network Configuration
See following for detailed instructions:
- [WSL2 Mirrored Networking Mode](https://github.com/mjs513/Arduino-Uno-Q-and-ROS2-Implementation/blob/main/Ros2%20Comm%20with%20WSL%20Ros2/%F0%9F%8C%90%20WSL2%20Mirrored%20Networking%20Mode%20%E2%80%94%20Full%20.md)
- [ROS 2 Jazzy + FastDDS Multi Machine Setup](https://github.com/mjs513/Arduino-Uno-Q-and-ROS2-Implementation/blob/main/Ros2%20Comm%20with%20WSL%20Ros2/README.md)
### 14.3 Rviz Coniguration
To keep from have to change Laserscan and Map msg names in Rviz

#### RViz2 Config File (save as sonar_slam.rviz) 
Create a file on your PC at:
```text
~/sonar_slam.rviz
```
Paste this inside:
```text
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map
      Topic: /map
      Alpha: 0.7
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: Scan
      Topic: /scan
      Size (m): 0.05
      Style: Points
    - Class: rviz_default_plugins/Pose
      Enabled: true
      Name: Pose
      Topic: /pose
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5.0
      Pitch: 0.785
      Yaw: 0.785
   ```
This config is intentionally simple and robust — nothing fancy, just the essentials for mapping.

#### Run the following command in wsl2 after sourcing your ros2 environment  
```text
source /opt/ros/jazzy/setup.bash
```

Rviz2 run command:
```text
rviz2 -d ~/sonar_slam.rviz
```

After the scan you will see your map!!!

## Other tests that I used  
Distance stress test:
```python
python3 - << 'EOF'
import time
from ros_sonar_py.app_utils import Bridge

print("=== distance-only stress test ===")
for i in range(5000):
    t0 = time.time()
    try:
        d = Bridge.call("get_distance_m")
    except Exception as e:
        print(f"[{i}] ERROR get_distance_m: {e}")
        break
    t1 = time.time()
    d = float(d) if d is not None else -1.0
    if i % 50 == 0:
        print(f"[{i}] dist={d:.3f} m  rpc={t1 - t0:6.3f}s")
    time.sleep(0.07)  # same ballpark as sweep step_total
EOF  
```

Sweep stress test:
```python  
python3 - << 'EOF'
import time
from ros_sonar_py.app_utils import Bridge

print("=== servo-only stress test ===")
angle = 0.0
step = 1.0
while True:
    t0 = time.time()
    try:
        Bridge.call("set_servo_angle", float(angle))
    except Exception as e:
        print(f"ERROR set_servo_angle({angle}): {e}")
        break
    t1 = time.time()
    print(f"angle={angle:6.1f} rpc={t1 - t0:6.3f}s")
    angle += step
    if angle > 180.0:
        angle = 0.0
    time.sleep(0.07)
EOF  
```

These tests are run once you start the conatiner.

To get timings of the sweep and for me LidarLite v3 instead of a ping sensor such as:  
```text  
Angle  175.0 deg | servo_rpc= 0.009s | settle=0.150s | dist_rpc= 0.012s | step_total= 0.171s
Angle  176.0 deg | servo_rpc= 0.009s | settle=0.150s | dist_rpc= 0.012s | step_total= 0.171s
Angle  177.0 deg | servo_rpc= 0.009s | settle=0.150s | dist_rpc= 0.012s | step_total= 0.172s
Angle  178.0 deg | servo_rpc= 0.009s | settle=0.150s | dist_rpc= 0.012s | step_total= 0.171s
Angle  179.0 deg | servo_rpc= 0.009s | settle=0.150s | dist_rpc= 0.011s | step_total= 0.171s
Angle  180.0 deg | servo_rpc= 0.009s | settle=0.150s | dist_rpc= 0.012s | step_total= 0.171s
=== Sweep complete: 33.978 seconds ===
```
You will need to modify the `sonar.py` file and uncomment the commented out code, i.e.,  
```python  
    def sweep(self):
        angles = []
        distances = []

        #sweep_start = time.time()
        #print("\n=== Starting sweep ===")

        for angle in self._frange(self.min_angle, self.max_angle, self.step):
            step_start = time.time()

            # ----------------------------------------------------
            # 1. Send servo command
            # ----------------------------------------------------
            #t0 = time.time()
            try:
                self.bridge.call("set_servo_angle", float(angle))
            except Exception as e:
                print(f"[ERROR] set_servo_angle({angle}) failed: {e}")
                print("[WARN] Aborting sweep early due to servo timeout")
                distances.append(float("inf"))
                #continue
                break
            #t1 = time.time()

            # Special case for angle 0
            if angle == 0:
                time.sleep(1.0)

            # Servo settle time
            time.sleep(self.settle_sec)

            # ----------------------------------------------------
            # 2. Read distance
            # ----------------------------------------------------
            #t2 = time.time()
            try:
                dist = self.bridge.call("get_distance_m")
            except Exception as e:
                print(f"[ERROR] get_distance_m() failed at angle {angle}: {e}")
                print("[WARN] Aborting sweep early due to distance timeout")
                break
                #dist = -1.0
            #t3 = time.time()

            # Convert to float
            dist = float(dist) if dist is not None else -1.0

            # ----------------------------------------------------
            # 3. Log timing for this step
            # ----------------------------------------------------
            #print(
            #    f"Angle {angle:6.1f} deg | "
            #    f"servo_rpc={t1 - t0:6.3f}s | "
            #    f"settle={self.settle_sec:5.3f}s | "
            #    f"dist_rpc={t3 - t2:6.3f}s | "
            #    f"step_total={t3 - step_start:6.3f}s"
            #)

            angles.append(angle)
            distances.append(dist)

            # Small pacing delay to avoid RouterBridge queue overload
            time.sleep(0.01)

        sweep_end = time.time()
        #print(f"=== Sweep complete: {sweep_end - sweep_start:.3f} seconds ===\n")

        return angles, distances
```

Just as a reminder if you dont want to use `tmux` you can open a second shell to the Q and then run:  
```text
docker exec -it **** base
```
where `***` is the container id of the running `ros-sonar-slam` image.

Remember you can do a `docker ps -a` to see the list of containers.

  