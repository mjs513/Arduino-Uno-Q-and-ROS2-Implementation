FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------
# Base system setup
# -------------------------------
RUN apt update && apt install -y \
    locales curl gnupg2 lsb-release git \
    && locale-gen en_US en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# -------------------------------
# ROS 2 Jazzy APT key + repo (corrected)
# -------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

# -------------------------------
# ROS 2 Desktop + dev tools + colcon + rosdep
# -------------------------------
RUN apt update && apt install -y \
# changed from desktop to base
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# -------------------------------
# X11 GUI support
# -------------------------------
#RUN apt update && apt install -y \
#    x11-apps \
#    mesa-utils \
#    libgl1-mesa-glx \
#    libx11-6

# -------------------------------
# tmux + clean config
# -------------------------------
RUN apt update && apt install -y tmux

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
# Pre-built workspace
# -------------------------------
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws

# Example repo - replace with your own
#RUN git clone https://github.com/ros2/examples src/examples

# Build workspace
#RUN . /opt/ros/jazzy/setup.sh && \
#    colcon build --symlink-install

# -------------------------------
# Auto-source ROS + workspace
# -------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ros_ws/install/setup.bash" >> /etc/bash.bashrc

CMD ["/bin/bash"]

