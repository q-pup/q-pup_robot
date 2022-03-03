# q-pup_robot
ROS Package for q-pup

# Prereqs
1) Install CUDA: https://developer.nvidia.com/cuda-downloads

2) Install the ZED SDK
```
wget https://download.stereolabs.com/zedsdk/3.7/cu115/ubuntu20
chmod +x ubuntu20
./ubuntu20
```


# Workspace Initialization

```
cd <workspace>/src
git clone https://github.com/q-pup/qpup_robot.git

# Install all dependencies
rosinstall --catkin . qpup_robot/qpup_robot.rosinstall
rosdep install --from-paths . --ignore-src -r -y
```
