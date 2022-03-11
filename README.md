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

# Hardware Initialization
[CAN](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9)
```
sudo apt install busybox
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

### Hard Realtime Loop

`qpup.launch` makes the assumption that you are running a linux kernel will realtime capabilities (like our NVIDIA
Jetsons where we have enabled the PREEMPT_RT kernel patch). If this assumption is not true(ex. you are running nodes on
your own development machines), you must run `qpup.launch` with `use_realtime_kernel:=false`

On machines with a realtime kernel, make sure you have permissions to lock unlimited memory for processes.

To check:

```
ulimit -l
```

If it does not return `unlimited`, you must adjust your memory locking permissions:

1. Open `/etc/security/limits.conf` as sudo in a text editor (ex. gedit, nano, vim)
2. Add the following line:
   ```
   <your username>    -   memlock   -1
   ```
3. Reboot your computer
4. Check that `ulimit -l` returns `unlimited`

# Bringup

```
rosrun qpup_bringup bringup_can.sh

roslaunch qpup_bringup qpup.launch
```
