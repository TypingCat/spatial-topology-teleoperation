# Waffle
## Simple Usage
``` bash
ros2 launch waffle wakeup.launch.py
ros2 launch waffle getup.launch.py
ros2 launch waffle play.launch.py
```


## Setup
- Ubuntu 18.04
- ROS2 dashing
    - [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
    - [rplidar_ros](https://github.com/allenh1/rplidar_ros.git) 2.0.0(submodule)
    - [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)
    - [cartographer_ros](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation)
    - [image_transport](https://index.ros.org/p/image_transport/github-ros-perception-image_common/#dashing)

``` bash
git clone https://github.com/finiel/waffle.git
cd waffle
colcon build --symlink-install
source install/local_setup.bash
```
