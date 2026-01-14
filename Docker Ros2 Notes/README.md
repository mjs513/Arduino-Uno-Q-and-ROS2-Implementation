# Installing Nodes in a Running Container
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

# Build the Docker Container with Nodes preinstalled

## 🟢 If you want the demo nodes (recommended for testing)
Just install them:
RUN apt update && apt install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
    
Then rebuild the image.






