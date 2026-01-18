# 🧩 1. Install RViz2 (ROS 2 Jazzy Desktop)

RViz2 is included in the **ros-jazzy-desktop** meta‑package. If you already installed that earlier, you’re done. If not, install it now:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop
```

This installs:

- RViz2  
- rqt + plugins  
- Gazebo Harmonic  
- Visualization dependencies  
- Qt GUI stack  

You don’t need anything extra beyond this.

---

# 🖥️ 2. Enable GUI support in WSL2

WSL2 **does not** display Linux GUIs by itself. Windows 11 includes a built‑in Wayland/X11 server, so you don’t need to install VcXsrv or Xming anymore.

### ✔️ Confirm GUI support is active

Run this inside WSL2:

```bash
echo $DISPLAY
```

You should see something like:

```
:0
```

If `$DISPLAY` is empty, set it manually:

```bash
export DISPLAY=:0
```

To make it permanent:

```bash
echo 'export DISPLAY=:0' >> ~/.bashrc
```

### ✔️ Install WSLg dependencies (usually already present)

```bash
sudo apt install mesa-utils
```

Test GUI rendering:

```bash
glxinfo | grep OpenGL
```

If you see GPU info, you’re good.

---

# 🚀 3. Launch RViz2

Now source ROS 2 and run RViz2:

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

If everything is configured correctly, RViz2 will pop up on your Windows desktop.

---

# 🧠 4. Common pitfalls (and how to avoid them)

### ❗ RViz2 crashes immediately  
Usually caused by missing GPU libraries. Fix:

```bash
sudo apt install libgl1-mesa-glx libgl1-mesa-dri
```

### ❗ RViz2 opens but is blank  
Set this environment variable:

```bash
export LIBGL_ALWAYS_INDIRECT=0
```

Add to `~/.bashrc` if needed.

### ❗ “Could not initialize OpenGL”  
WSLg sometimes needs a reboot:

1. Close WSL  
2. Run in PowerShell:

```powershell
wsl --shutdown
```

3. Reopen Ubuntu

---

# 🌐 5. If you want RViz2 to visualize data from another machine

Since you mentioned running ROS 2 on the Q and visualizing on your PC:

- Both machines must be on the same Wi‑Fi network  
- Set matching ROS 2 environment variables:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

- Ensure both use the same RMW (Fast DDS is default):

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Then RViz2 in WSL2 will discover topics published by the Q.

---

