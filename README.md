# multibot2
This repository provides the multi-robot ROS2 package, which creates a local trajectory for multiple mobile robots. Robots create subgoals to avoid collisions and deadlocks between robots, and then optimize trajectories to those subtgoals.

## 1. Concept
To generate proper subgoals, [PIBT](https://kei18.github.io/pibt2/) and [V-RVO](https://arxiv.org/abs/2102.13281) are used. More details are as follows.
[![Video Label](http://img.youtube.com/vi/59USvjy2toI/0.jpg)](https://youtu.be/59USvjy2toI)