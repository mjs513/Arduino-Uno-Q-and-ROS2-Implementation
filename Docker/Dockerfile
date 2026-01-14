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
# ROS 2 Base + dev tools
# -------------------------------
RUN apt update && apt install -y --no-install-recommends \
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# -------------------------------
# tmux + clean config
# -------------------------------
RUN apt update && apt install -y --no-install-recommends tmux \
    && rm -rf /var/lib/apt/lists/*

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
# Workspace
# -------------------------------
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws

# -------------------------------
# Auto-source ROS + workspace (safe, no warnings)
# -------------------------------
RUN printf "%s\n" \
"if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi" \
"if [ -f /opt/ros_ws/install/setup.bash ]; then source /opt/ros_ws/install/setup.bash; fi" \
>> /etc/bash.bashrc

CMD ["/bin/bash"]
