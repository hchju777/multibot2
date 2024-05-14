# multibot2
This repository provides the multi-robot ROS2 package, which creates a local trajectory for multiple mobile robots. Robots create subgoals to avoid collisions and deadlocks between robots, and then optimize trajectories to those subtgoals.

## 1. Concept
To generate proper subgoals, [PIBT](https://kei18.github.io/pibt2/) and [V-RVO](https://arxiv.org/abs/2102.13281) are used. More details are as follows.

[![Video Label](http://img.youtube.com/vi/BsfKGs3H9ww/0.jpg)](https://youtu.be/BsfKGs3H9ww)

## 2. Installation
**Dependencies:** This software is built on the Robotic Operating System ([ROS](https://www.ros.org/)). We assume that the followings are installed.
- Ubuntu (Tested on 20.04) 
- ROS2 (Tested on [ROS Foxy](https://docs.ros.org/en/foxy/))
- [CGAL](https://www.cgal.org/index.html) library
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) library
- [costmap_converter](https://wiki.ros.org/costmap_converter) package

For installation of **`CGAL`**, use the following commands:
```
sudo apt install libcgal-dev
```

For installation of **`yaml-cpp`**, use the following commands:
```
sudo apt install libyaml-cpp-dev
```

**Build:** In order to install the `multibot2` package, clone the latest version from this repository and compile the package.
  ```
  cd ~/{your-ros-workspace}/src
  git clone https://github.com/hchju777/multibot2.git
  cd ..
  colcon build
  source install/setup.bash
  ```

## 3. Basic Usage
### 3-1. Launch multibot2_server
```
ros2 launch multibot2_server multibot2_server_simul_launch.py 
```
### 3-2. Launch multibot2_robot
#### Spawn robots at once
```
ros2 launch multibot2_server multibot2_robots_sim_launch.py 
```
#### Spawn a robot
```
ros2 launch multibot2_robot robot1_sim_launch.py 
```

## 4. Results
![result](https://github.com/hchju777/multibot2/assets/169625948/bd0fcb0a-85c2-408b-ac94-e05f095ff398)

## 5. Experiment
[![Video Label](http://img.youtube.com/vi/UCHZ8Q0e030/0.jpg)](https://youtu.be/UCHZ8Q0e030)
[![Video Label](http://img.youtube.com/vi/mAPAo0MlVRQ/0.jpg)](https://youtu.be/mAPAo0MlVRQ)
[![Video Label](http://img.youtube.com/vi/F9dSxDVsyl4/0.jpg)](https://youtu.be/F9dSxDVsyl4)