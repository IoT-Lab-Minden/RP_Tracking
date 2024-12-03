# Software installation

### 1. Prerequisites

This software is based on ROS 1 and therefore requires a functioning ROS 1 installation. The software was tested using ROS Noetic on Ubuntu 20.04. Besides the ROS installation, the following additional package needs to be installed:

- jsk_recognition_msgs

For ROS Noetic, you can install the package using the following command:

```bash
sudo apt install ros-noetic-jsk-recognition-msgs
```

#### 2. Installation of rp_tracking

To install the robotic platform tracking, it needs to be installed into a catkin workspace. When inside the *src* folder, you can clone the repository using the following command:

```bash
git clone https://github.com/IoT-Lab-Minden/rp_tracking
```

Then, execute the command *catkin_make* in the main workspace folder to build the package.
