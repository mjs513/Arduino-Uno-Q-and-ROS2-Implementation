# 📘 ROS 2 Jazzy Docker Environment  
*A minimal, developer‑friendly container for ROS 2 Jazzy on Ubuntu 24.04*

This document explains every command in the Dockerfile and provides clear instructions for building and running the container.

---

# 🧱 1. Dockerfile Breakdown (Line‑by‑Line)

## **Base Image**
```dockerfile
FROM ubuntu:24.04
```
Uses Ubuntu 24.04 (Noble), the official target platform for ROS 2 Jazzy.

---

## **Non‑interactive APT**
```dockerfile
ENV DEBIAN_FRONTEND=noninteractive
```
Prevents APT from asking interactive questions during installation.

---

## **System Setup**
```dockerfile
RUN apt update && apt install -y \
    locales curl gnupg2 lsb-release git \
    && locale-gen en_US en_US.UTF-8
```
- Updates package lists  
- Installs essential utilities:
  - `locales` → UTF‑8 support  
  - `curl` → downloading files  
  - `gnupg2` → key management  
  - `lsb-release` → OS metadata  
  - `git` → version control  
- Generates the `en_US.UTF-8` locale

---

## **Locale Environment Variables**
```dockerfile
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
```
Ensures all tools run with UTF‑8 encoding.

---

## **ROS 2 APT Key**
```dockerfile
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Downloads and installs the official ROS 2 package signing key.

---

## **ROS 2 Repository**
```dockerfile
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list
```
Adds the ROS 2 Jazzy repository for Ubuntu Noble.

---

## **Install ROS 2 + Dev Tools**
```dockerfile
RUN apt update && apt install -y \
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
```
Installs:
- `ros-jazzy-ros-base` → core ROS 2 without GUI tools  
- `colcon` → ROS 2 build system  
- `rosdep` → dependency manager  
- `vcstool` → multi-repo management  
- `build-essential` → compilers and build tools  
- Cleans APT cache to reduce image size

---

## **Initialize rosdep**
```dockerfile
RUN rosdep init || true
```
Initializes rosdep.  
`|| true` prevents failure if it’s already initialized.

---

## **Install tmux**
```dockerfile
RUN apt update && apt install -y tmux
```
Installs tmux for terminal multiplexing.

---

## **tmux Configuration**
```dockerfile
RUN mkdir -p /etc/tmux && \
    printf "%s\n" \
    ...config lines...
    > /etc/tmux/tmux.conf
```
Creates a global tmux config with:
- Prefix changed to `Ctrl+A`  
- Mouse mode enabled  
- Pane navigation shortcuts  
- Better status bar  
- Vi-style copy mode  

---

## **Alias tmux to use the config**
```dockerfile
RUN echo "alias tmux='tmux -f /etc/tmux/tmux.conf'" >> /etc/bash.bashrc
```
Ensures every tmux session uses your custom config.

---

## **Create Workspace**
```dockerfile
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws
```
Creates a standard ROS 2 workspace at `/opt/ros_ws`.

---

## **Auto‑source ROS + Workspace**
```dockerfile
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ros_ws/install/setup.bash" >> /etc/bash.bashrc
```
Automatically sources:
- ROS 2 environment  
- Workspace overlay (once built)

Every new shell is ROS‑ready.

---

## **Default Command**
```dockerfile
CMD ["/bin/bash"]
```
Starts an interactive bash shell when the container runs.

---

# 🏗️ 2. Building the Docker Image

From the directory containing the Dockerfile:

```bash
docker build -t ros2-jazzy .
```

- `-t ros2-jazzy` → names the image  
- `.` → build context is the current directory

---

# 🚀 3. Running the Container

## **Basic interactive shell**
```bash
docker run -it ros2-jazzy
```

---

## **Run with workspace mounted from host**
Recommended for development:

```bash
docker run -it \
    -v ~/my_ws:/opt/ros_ws \
    ros2-jazzy
```

Your host workspace replaces the container’s workspace.

---

## **Run with X11 GUI support (if GUI tools are added later)**

Linux:

```bash
xhost +local:docker
docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros2-jazzy
```

---

## **Run with persistent bash history**
```bash
docker run -it \
    -v ~/.bash_history:/root/.bash_history \
    ros2-jazzy
```

---

# 🔧 4. Building Your Workspace Inside the Container

Once inside:

```bash
cd /opt/ros_ws
colcon build --symlink-install
```

Then open a new shell so auto-sourcing takes effect:

```bash
bash
```

---

# 🎉 Done
