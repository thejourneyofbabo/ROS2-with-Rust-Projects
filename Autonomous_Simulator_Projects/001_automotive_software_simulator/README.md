# 202420HY10228_Automotive_Software

## Getting started

### 0. Environments
* OS: Ubuntu 22.04    
* ROS2: [ROS2 humble](https://docs.ros.org/en/humble/index.html) version

### 1. Install ROS messages and other libraries
* for Ubuntu 22.04 (humble)
```bash
sudo apt install terminator ros-humble-rviz-2d-overlay-* -y
```

### 2. Build and run application
* Please clone the project in your `${HOME}/git/` directory!
```bash
$ mkdir -p ~/git && cd ~/git
$ git clone http://KUAilab.synology.me:30000/class/202420hy10228_automotive_software.git
$ cd 202420hy10228_automotive_software
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
