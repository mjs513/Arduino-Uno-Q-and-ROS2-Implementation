# ROS 2 Jazzy + FastDDS Multi Machine Setup (PC ↔ UNO Q)
## Overview
This project provides a clean, reproducible ROS 2 Jazzy environment designed for multi machine communication between:
- PC (WSL2, Ubuntu 24.04)
- UNO Q (Docker container running Ubuntu 24.04)

Communication uses FastDDS with unicast discovery, ensuring reliable ROS graph visibility even on networks where multicast is restricted.

The system includes:
- A custom ROS 2 Jazzy base image
- FastDDS configuration baked into the container
- Developer friendly tools (tmux, nano with syntax highlighting)
- A clean workflow for building and running ROS nodes across machines

## 🟦 1. Networking Assumptions
| Device    | IP            | Purpose                |
| :-------- | :------------ | :--------------------- |
| PC (WSL2) | 192.168.1.YY  | FastDDS peer for UNO Q |
| UNO Q     | 192.168.1.xxx | FastDDS peer for PC    |

### Find your IPs using:
PC (Windows):
```
ipconfig
```
UNO Q:
```
ip addr
```
If the ip addresses for the PC(WSL) and the UNO Q are not on the same network, i.e., 192.168.1.---- then you will need to configure WSL2 for Mirrored netowrk mode:
🌐 WSL2 Mirrored Networking Mode — Full 

[🌐 WSL2 Mirrored Networking Mode — Full ](docs/CONTRIBUTING.md)

## 🟩 2. PC (WSL2) Setup
### 2.1 Install demo nodes
```
sudo apt update
sudo apt install -y ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py
```

### 2.2 Create FastDDS XML file
his tells the PC to explicitly look for the UNO Q.
Replace 192.168.1.XXX with the UNO Q’s LAN IP.
```
cat <<EOF > ~/.fastdds.xml
<profiles>
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="unicast_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>192.168.1.XXX</address>
              <port>7400</port>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF
```

### 2.3 Add FastDDS environment variables
Append to `~/.bashrc`:
```
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.fastdds.xml' >> ~/.bashrc
export ROS_DOMAIN_ID=0
```
Reload:
```
source ~/.bashrc
```

### 2.4 Restart ROS daemon
```
ros2 daemon stop
ros2 daemon start
```

## 🟧 3. Arduino FastDDS configuration
This step was made permanent in the Docker Base Image for Ros2, but for reference and sake of my memory.

### 3.1 FastDDS XML baked into the image
Create File: /root/.fastdds_uno.xml
Replace 192.168.1.YY with the PC’s IP.
```
<profiles>
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="unicast_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>192.168.1.36</address>
              <port>7400</port>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

### 3.2 Environment variables
```
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.fastdds_uno.xml
ENV ROS_DOMAIN_ID=0
```

### 🟦 4. Build and Run the UNO Q Container
Dockerfiles can be created and run from anywhere in the file system.  In this case I created a ROS2 directory under the arduino directory on MPU.  

The file structure I implemented is as showing below:
```
|--- home
   |--- arduino
     |--- ROS2
       |--- Ros2_Docker_base
            Contains the docker file to create the base image
            --- Dockerfile.ros_jazzy_base
       |--- ros2_docker_talker_test
       	     Contains the talker test images
       	     --- Dockerfile.Dockerfile.unoq_talker_auto
       	     --- Dockerfile.unoq_talker_man
       |--- ros2_led_node
            Contains the directory and files for the ros2 led example.
```

The Dockerfile.ros_jazzy_base image supports:
- ROS 2 Jazzy
- FastDDS RMW
- FastDDS XML baked in
- tmux
- nano with syntax highlighting
- ROS sourcing
- Developer tools

Create directory:  
```
mkdir -p Ros2_Docker_base
cd Ros2_Docker_base
```
Create the dockerfile
```
sudo nano Dockerfile.ros_jazzy_base
```
Paste the following into the file and save:
```
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------
# Base system setup
# -------------------------------
RUN apt update && apt install -y --no-install-recommends \
    locales curl gnupg2 lsb-release git ca-certificates \
    && update-ca-certificates \
    && locale-gen en_US en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# -------------------------------
# ROS 2 Jazzy APT key + repo
# -------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

# -------------------------------
# ROS 2 base + FastDDS RMW
# -------------------------------
RUN apt update && apt install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-fastrtps \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# ROS dev tools
# -------------------------------
RUN apt update && apt install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# Extra utilities (tmux, pip)
# -------------------------------
RUN apt update && apt install -y --no-install-recommends \
    tmux python3-pip \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# tmux config
# -------------------------------
RUN mkdir -p /etc/tmux && \
    printf "%s\n" \
"unbind C-b" \
"set -g prefix C-a" \
"bind C-a send-prefix" \
"set -g mouse on" \
"set -g default-terminal \"screen-256color\"" \
"bind | split-window -h" \
"bind - split-window -v" \
"bind -n M-Left select-pane -L" \
"bind -n M-Right select-pane -R" \
"bind -n M-Up select-pane -U" \
"bind -n M-Down select-pane -D" \
"set -g history-limit 10000" \
"set -g status-bg black" \
"set -g status-fg white" \
"set -g status-left \" #[bold]tmux \"" \
"set -g status-right \"#(date '+%H:%M') \"" \
"setw -g mode-keys vi" \
> /etc/tmux/tmux.conf

RUN echo "alias tmux='tmux -f /etc/tmux/tmux.conf'" >> /etc/bash.bashrc


# -------------------------------
# Nano editor with syntax highlighting
# -------------------------------
RUN apt update && apt install -y --no-install-recommends nano \
    && rm -rf /var/lib/apt/lists/*

# Create a clean nano config with highlighting + QoL settings
RUN printf "%s\n" \
"## Enable syntax highlighting" \
"include \"/usr/share/nano/*.nanorc\"" \
"" \
"## Editor quality-of-life settings" \
"set linenumbers" \
"set softwrap" \
"set tabsize 2" \
"set autoindent" \
> /root/.nanorc


# -------------------------------
# rosdep init
# -------------------------------
RUN rosdep init || true && rosdep update

# -------------------------------
# FastDDS XML config (replace IP with your PC's IP)
# -------------------------------
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
              <address>192.168.1.YY</address>\n\
              <port>7400</port>\n\
            </udpv4>\n\
          </locator>\n\
        </initialPeersList>\n\
      </builtin>\n\
    </rtps>\n\
  </participant>\n\
</profiles>" > /root/.fastdds_uno.xml

# -------------------------------
# FastDDS environment variables
# -------------------------------
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.fastdds_uno.xml
ENV ROS_DOMAIN_ID=0

# -------------------------------
# Auto-source ROS
# -------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]

```
**Important remember to change the PC address in the docker file before building:**
```
<address>192.168.1.YY</address>\n\
```

## 4.1 Build
```
docker build -f Dockerfile.ros_jazzy_base -t ros-jazzy-base .
```
This creates a docker image ros-jazzy-base.

## 4.2 Run
```
docker run --net=host -it --rm ros-jazzy-base bash
```

## 4.3 Create, build and run the talker image
       |--- ros2_docker_talker_test
       	     Contains the talker test images
       	     --- Dockerfile.Dockerfile.unoq_talker_auto
       	     --- Dockerfile.unoq_talker_man
### 4.3.1 Create the talker image
Create directory:  
```
mkdir -p ros2_docker_talker_test
cd ros2_docker_talker_test
```
Create the dockerfile - note in this version you will have to manually run the ros2 noode:
```
sudo nano Dockerfile.unoq_talker_man
```
Paste the following into the file and save:
```
# ============================================================
# Base image: Your custom ROS 2 Jazzy + FastDDS base
# ============================================================
FROM ros-jazzy-base

# ------------------------------------------------------------
# Install demo talker node
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-demo-nodes-cpp \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Default command: interactive shell
# ------------------------------------------------------------
CMD ["bash"]

```

Build the image
```
docker build -f Dockerfile.unoq_talker_man -t unoq_talker_man .
```
Now when you are ready you can run the image with
```
docker run --net=host -it --rm unoq_talker_man
```
With manual talker image you will have to tell ros2 to run talker once you are inside the container  
```
ros2 run demo_nodes_cpp talker
```

In a similar manner you could make the image automatically run the node:
1.  Create the docker file
```
sudo nano Dockerfile.unoq_talker_man
```
2. Paste the following into the file:  
```
# ============================================================
# Base image: Your custom ROS 2 Jazzy + FastDDS base
# ============================================================
FROM ros-jazzy-base

# ------------------------------------------------------------
# Install demo talker node (if not already in base)
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-demo-nodes-cpp \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Default command: run the talker
# ------------------------------------------------------------
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp talker"]

```
3. to build and run:
```
Dockerfile.unoq_talker_auto -t unoq_talker_auto .
docker run --net=host --rm unoq_talker_auto
```


## 🟩 5. Testing the Setup
### 5.1 On the UNO Q container
```
ros2 run demo_nodes_cpp talker
```
Leave it running.

### 5.2 On the PC
Restart daemon:
```
ros2 daemon stop
ros2 daemon start
```
Check nodes:
```
ros2 node list
```
Expected:
```
/talker
```
Run listener:
```
ros2 run demo_nodes_cpp listener
```
You should see:
```
I heard: [Hello World: N]
```

## 🟧 6. Troubleshooting
**XML file not found**
Check:
```
echo $FASTRTPS_DEFAULT_PROFILES_FILE
```
No nodes appear

Verify:
1. Both machines use FastDDS
2. Both XML files point to each other’s IP
3. Both are on the same subnet
4. Docker uses --net=host

Listener works but XML errors appear
Recreate the XML file using a clean heredoc block.

## 🟩 7. Credits
“Technical guidance provided with the help of Microsoft Copilot.”
