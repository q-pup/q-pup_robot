# q-pup_robot
ROS Package for q-pup


# Workspace Initialization

```
cd <workspace>/src
git clone https://github.com/q-pup/q_pup_robot.git

# Install all dependencies
rosinstall --catkin . q_pup_robot/q_pup_robot.rosinstall
rosdep install --from-paths . --ignore-src -r -y
```
