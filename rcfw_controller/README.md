# rcfw_controller

A small ROS Python package scaffold for the Diffbot project.

Contains a starter node `rcfw_controller_node.py` that publishes a heartbeat message to `rcfw_heartbeat`.

How to build (catkin workspace root):

```bash
catkin_make
source devel/setup.bash
rosrun rcfw_controller rcfw_controller_node.py
```
