# ROS 2 Jazzy – Arduino UNO Q Workspace Container

This repository contains a two‑layer Docker setup for running ROS 2 Jazzy directly on the Arduino UNO Q MPU (Qualcomm Linux). The container builds a unified ROS workspace containing:

- The `ros_led` package (LED control via RPC to the STM32)
- Your Python ROS 2 package(s)

Everything is built inside `/opt/ros_ws`.

This is based on the effort of miguelgonrod - https://github.com/miguelgonrod/ros_arduino_uno_Q which carries a BSD license

---

## Directory Structure

```
python/
└── src/
     ├── ros_led/
     ├── <your python ROS package>
     ├── resource/
     ├── test/
     ├── package.xml
     ├── setup.cfg
     └── setup.py
python/requirements.txt
docker/
├── Dockerfile.base
├── Dockerfile.led_ws
```

---

## Images

### 1. Base Image (`ros2-base:jazzy`)
Contains:
- Ubuntu 24.04
- ROS 2 Jazzy (ros-base)
- colcon, rosdep, vcstool
- tmux + config
- Python + pip

No workspace is built here.

### 2. Workspace Image (`ros2-led-ws:latest`)
Contains:
- Everything from the base image
- Your merged ROS workspace
- Python dependencies
- rosdep install
- colcon build
- Auto‑sourcing of `/opt/ros_ws/install/setup.bash`

---

## Build

From the repo root:

🧱 build.sh
bash

```
#!/bin/bash
set -e

echo "Building base image..."
docker build -t ros2-base:jazzy -f docker/Dockerfile.base .

echo "Building merged workspace image..."
docker build -t ros2-led-ws:latest -f docker/Dockerfile.led_ws .

echo "Build complete."
```
Make executable:
bash
```
chmod +x build.sh
```

🧱 run.sh
bash
```
#!/bin/bash

docker run -it --net=host --name ros2_uno_q_dev --rm -v $(pwd)/src:/workspace/src -v /var/run/arduino-router.sock:/var/run/arduino-router.sock ros2-led-ws:latest

```

Make executable:
bash
```
chmod +x run.sh
```

This builds:
ros2-base:jazzy
ros2-led-ws:latest

bash
```
./run.sh
```

You will enter a shell where:

ROS 2 Jazzy is sourced

Your workspace is sourced

All nodes are ready to run

## Notes
The workspace is fully merged: Python packages + ros_led build together.

This is the recommended structure for UNO Q MPU deployments.

You can add more packages under python/src and they will be built automatically.
