# Waffle
ROS2 package that develops wheeled mobile robot for semantic teleoperation

The main purpose of this package is to service semantic commands such as 'turn left' and 'go to table'. The topology that extracted at 5Hz will make it possible. This topology basically represents the spatial structure and is ready to add things of interest. The robot can use it to guide teleoperation user or make decisions.

The following is a simple demo video: This robot is moving from the hallway to the room. Black is the movable area detected by the LiDAR. Spatial structure is represented by graph. Yellow nodes are intersections, and red nodes are ends. The edges represent movability.

![extract_topology](https://user-images.githubusercontent.com/16618451/105449922-a6fae680-5cbc-11eb-8043-47b890e61912.gif)


## Algorithms
### [Local Topology Extraction](https://github.com/TypingCat/waffle/issues/12)
Mobile robots should be able to explore the environment itself. This algorithm helps the robot to recognize the spatial structure of the environment. It receives observations and returns a graph. This graph is already a topology that shows movability, but the information quality can be further improved by adding things of interest.

### [Topology Merge](https://github.com/TypingCat/waffle/issues/11)
Mobile robots should be able to remember the environment. Information will be volatilized without the process of connecting local topologies. However, the spatial structure is difficult to estimate because it has coordinates even though there is no substance. As a result of the test [#7](https://github.com/TypingCat/waffle/issues/13), intersection clustering approach was adopted. This approach detects probabilistically significant intersections.


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
This package is being developed with modified Turtlebot3. See issue [#8](https://github.com/TypingCat/waffle/issues/8) for details.

![hardware](https://user-images.githubusercontent.com/16618451/105457365-0ca19f80-5cca-11eb-8dd6-8a64d6c7ac5a.png)

- Ubuntu 18.04
- ROS2 dashing
    - [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
    - [rplidar_ros](https://github.com/allenh1/rplidar_ros.git) 2.0.0(submodule)
    - [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)
    - [cartographer_ros](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation)
    - [image_transport](https://index.ros.org/p/image_transport/github-ros-perception-image_common/#dashing)

``` bash
git clone https://github.com/TypingCat/waffle.git
cd waffle
colcon build --symlink-install
source install/local_setup.bash
```
