# 📘 ROS 2 Jazzy Docker Environment  
*A minimal, developer‑friendly container for ROS 2 Jazzy on Ubuntu 24.04*

This document explains every command in the Dockerfile and provides clear instructions for building and running the container and was generated with CoPilot

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

# USING TMUX COMMANDS



# 🧩 **Prefix Key: Switching to `Ctrl+a`**
```
unbind C-b
set -g prefix C-a
bind C-a send-prefix
```

### What it does
- Removes the default prefix (`Ctrl+b`)
- Sets your new prefix to **Ctrl+a** — faster, closer to home row
- Allows sending `Ctrl+a` to programs inside tmux when needed

### How to use it
Every tmux command starts with:

```
Ctrl+a
```

Example: split a pane vertically → `Ctrl+a -`

---

# 🖱️ **Mouse Mode**
```
set -g mouse on
```

### What it does
Enables:
- Click to select panes
- Click to resize panes
- Scroll wheel to scroll history

### How to use it
Just use your mouse naturally.  
Scroll = scroll history  
Drag pane borders = resize  
Click pane = focus

---

# 🎨 **Terminal Colors**
```
set -g default-terminal "screen-256color"
```

### What it does
Ensures tmux supports 256‑color output.  
This fixes issues with ROS tools, colcon output, and vim/neovim themes.

---

# 🪟 **Pane Splitting**
```
bind | split-window -h
bind - split-window -v
```

### What it does
Adds intuitive shortcuts:

- `Ctrl+a |` → split **left/right** (horizontal split)
- `Ctrl+a -` → split **top/bottom** (vertical split)

### How to use it
Inside tmux:

```
Ctrl+a |
Ctrl+a -
```

This is perfect for your 3–4 pane ROS dev layout.

---

# 🔀 **Pane Navigation (Arrow Keys)**
```
bind -n M-Left  select-pane -L
bind -n M-Right select-pane -R
bind -n M-Up    select-pane -U
bind -n M-Down  select-pane -D
```

### What it does
Allows **instant pane switching** using:

- **Alt + Left**
- **Alt + Right**
- **Alt + Up**
- **Alt + Down**

No prefix required.

### How to use it
Just press:

```
Alt + ←
Alt + →
Alt + ↑
Alt + ↓
```

This is the fastest way to move between your build, run, and log panes.

---

# 📜 **Scrollback History**
```
set -g history-limit 10000
```

### What it does
Increases scrollback buffer to 10,000 lines.

### How to use it
Scroll with mouse wheel or:

```
Ctrl+a [
```

Then use:
- `k/j` to scroll  
- `q` to exit scroll mode

---

# 🟦 **Status Bar Styling**
```
set -g status-bg black
set -g status-fg white
set -g status-left " #[bold]tmux "
set -g status-right "#(date '+%H:%M') "
```

### What it does
- Black background, white text
- Left side shows “tmux”
- Right side shows current time (updates every minute)

### How to use it
Purely visual — no interaction needed.

---

# ⌨️ **Vi Mode for Copy/Paste**
```
setw -g mode-keys vi
```

### What it does
Enables vim-like navigation in copy mode.

### How to use it
Enter copy mode:

```
Ctrl+a [
```

Then:
- `h/j/k/l` → move  
- `v` → start selection  
- `y` → yank (copy)  
- `q` → exit  

Paste with:

```
Ctrl+a ]
```

---

# 🧭 **Putting It All Together: Your 4‑Pane ROS Dev Layout**

Here’s a workflow you’ll love:

### 1. Start tmux
```
tmux
```

### 2. Create panes
```
Ctrl+a |     # left/right split
Ctrl+a -     # split bottom pane
Alt+Down     # move to bottom
Ctrl+a |     # split bottom into two
```

### 3. Assign roles
- **Top-left:** build (`colcon build --symlink-install`)
- **Top-right:** run nodes (`ros2 run ...`)
- **Bottom-left:** logs (`ros2 topic echo ...`)
- **Bottom-right:** interactive shell

### 4. Navigate instantly
```
Alt + Arrow keys
```


# 🎉 Done
