# SRMAUV Workspace for ROS

This is a workspace for working on AUV using ros.
To complete the setup, first of all clone this repo in the `$HOME` directory of your system.

> :warning: **Warning:** Make sure you have <a href="https://docs.ros.org/en/humble/Installation.html">ROS2 (Humble)</a> and <a href="https://gazebosim.org/docs/harmonic/install_ubuntu/">Gazebo Harmonic</a> installed in your system.

### Then install the required ROS2 plugins with the given commands:
ROS2 with Gazebo Harmonic:
```
sudo apt-get install ros-humble-ros-gzharmonic
```
Transforms:
```
sudo apt install ros-humble-tf2-tools
```
URDF:
```
sudo apt install ros-humble-urdf-tutorial
```

And finally add these lines to your `~/.bashrc` file by using the cmd `gedit ~/.bashrc`
```
source /opt/ros/humble/setup.bash
source ~/auv_ws/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
> :bulb: **Tip:** We need to export the `GZ_SIM_RESOURCE_PATH` and the path will be available by using the cmd `echo $GZ_SIM_RESOURCE_PATH` and add the path in `export GZ_SIM_RESOURCE_PATH=$HOME/{path}`
<br><br>

Finally your setup is complete use the commands given below every time before running the code:

```
source ~/.bashrc
colcon build
```
