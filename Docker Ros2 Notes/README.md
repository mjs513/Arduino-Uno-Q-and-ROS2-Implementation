# 1.  Installing Nodes in a Running Container
---

When testing the Docker image I had attempted to run the examples

```
ros2 run demo_nodes_cpp talker
```
and
```
ros2 run demo_nodes_py listener
```
and recieved an error messages about demo_nodes not installed.

You’ve got a couple of clean ways to install the demo nodes **inside the running container** without touching your Dockerfile. This is actually a great workflow when you want to test something quickly without rebuilding the whole image.

Here are the three approaches.

---

## 🟢 **Option 1 — Install demo nodes interactively inside the container**

Start your container:

```bash
docker run -it ros2_jazzy
```

Then inside the container:

```bash
apt update
apt install -y ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py
```

That’s it.  
You can now run:

```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

**Important:**  
These installs live *only inside this container instance*.  
If you exit and re‑run the container, they won’t be there unless you commit the container or rebuild the image.

---

## 🟦 **Option 2 — Install demo nodes in a running container and save it**

If you want to keep the changes without editing the Dockerfile:

1. Run the container:

   ```bash
   docker run -it ros2_jazzy
   ```

2. Install the demo nodes inside:

   ```bash
   apt update
   apt install -y ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py
   ```

3. Open a second terminal and commit the container:

   ```bash
   docker ps   # find the container ID
   docker commit <container_id> mpu_ros2_jazzy_with_demos
   ```

Now you have a new image that includes the demo nodes, without ever touching the Dockerfile.

---

## 🟩 **Option 3 — Use a temporary container just for testing**

If you don’t want to save anything:

```bash
docker run -it --net=host --rm ros2_jazzy bash
```

Then install:

```bash
apt update
apt install -y ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py
```

When you exit, the container disappears and your base image stays clean.

---

# 2.  Build the Docker Container with Nodes preinstalled

## 🟢 If you want the demo nodes (recommended for testing)
Just install them:
```
RUN apt update && apt install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
```
    
Then rebuild the image.

---

# 3. Two Ways to Run Multiple ROS2 Windows

## 🟢 Step 1 — Start your container normally
Run your dev image:

bash
```
docker run -it --net=host --name ros2_dev os2_jazzy
```

The important part is --name ros2_dev  
This gives the container a predictable name so you don’t have to look up its ID later.

You’ll now be inside the first shell.

## 🟦 Step 2 — Open a second terminal window on your host
In that second terminal, run:

bash
```
docker exec -it ros2_dev bash
```

Boom — you now have two independent shells inside the same running container.

Window 1:

Code
`ros2 run demo_nodes_cpp talker`
Window 2:

Code
```
ros2 run demo_nodes_py listener
```

They’ll discover each other instantly because you used --net=host.

## 🟩 Alternative: If you forgot to name the container
List running containers:

bash
```
docker ps
```

You’ll see something like:

Code
```
CONTAINER ID   IMAGE             COMMAND   ...   NAMES
abc123def456   mpu_ros2_jazzy    "bash"          hopeful_brown
```
Then exec into it:

bash
`docker exec -it abc123def456 bash`
or:

bash
`docker exec -it hopeful_brown bash`

## 🟣 Bonus: tmux is still the cleanest inside the container
Since your dev image includes tmux, you can also do:

bash
`tmux`
Split:

Code
`Ctrl+a  |`
Left pane → talker
Right pane → listener

This avoids needing two host terminals.
---
## In Lieu Of Creating 2 Ros2 Sessions use tmux

One‑command tmux dev session (recommended)
Inside your container, create a script:

Code
`nano /usr/local/bin/dev_session`
Paste this:

Code
```
#!/bin/bash

tmux new-session -d -s dev "cd /opt/ros_ws && bash" \; \
  split-window -h "cd /opt/ros_ws && bash" \; \
  split-window -v "cd /opt/ros_ws && bash" \; \
  select-pane -t 0 \; \
  split-window -v "cd /opt/ros_ws && bash" \; \
  attach-session -t dev
```

Make it executable:
Code
`chmod +x /usr/local/bin/dev_session`

Now you can launch your full dev environment with:
Code
`dev_session`
What you get:
Code
```
+----------------------+----------------------+
|        Pane 0        |        Pane 1        |
|   build workspace    |   run nodes/launch   |
+----------------------+----------------------+
|        Pane 2        |        Pane 3        |
|   logs / echo / top  |   free interactive   |
+----------------------+----------------------+
```

You can assign each pane a role:

Pane 0:
Code
```
colcon build --symlink-install
```

Pane 1:
Code
```
ros2 run <your_pkg> <your_node>
```
Pane 2:
Code
```
ros2 topic echo /imu/data
```

Pane 3:
Code
```
htop
```

This is the cleanest dev workflow for ROS 2 inside a container.

##  A Second and Minimal inline tmux command (no script)
If you don’t want a script, you can launch the layout directly:
Code
```
tmux new-session \; \
  split-window -h \; \
  split-window -v \; \
  select-pane -t 0 \; \
  split-window -v
```
Then manually run your commands in each pane.


