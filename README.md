# Waffle
ROS2 package that develops wheeled mobile robot for semantic teleoperation


## Goals
The main purpose of this package is to service semantic commands such as 'turn left' and 'go to table'. There is no need to prepare a map in advance, because topology is extracted in real time. This topology contains spatial structure and things of interest. See issue [#3](https://github.com/finiel/waffle/issues/3) for details.

The following is a simple demo video of the [local topology extraction](https://github.com/finiel/waffle/issues/6). This robot is moving from the hallway to the room, and extracting topology in real time. Note that intersections are represented by yellow points.

![extract_topology](https://user-images.githubusercontent.com/16618451/105449922-a6fae680-5cbc-11eb-8043-47b890e61912.gif)


## Simple Usage
- Control a mobile robot
    ``` bash
    ros2 launch waffle wakeup.launch.py
    ros2 launch waffle getup.launch.py
    ros2 launch waffle play.launch.py
    ```
- Generate topology
    ``` bash
    ros2 run waffle_topology deproject_scan
    ros2 run waffle_topology generate_topology
    ```


## Setup
This package is being developed with [modified Turtlebot3](https://github.com/finiel/waffle/issues/8).

![hardware](https://user-images.githubusercontent.com/16618451/105457365-0ca19f80-5cca-11eb-8dd6-8a64d6c7ac5a.png)

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
