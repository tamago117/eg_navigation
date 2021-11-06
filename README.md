# eg_navigation
ROS meta package for robot 2d/3d navigation

# Requirments

- ROS Melodic
- Gazebo(ver9.4.0) (for simulation) ->[update gazebo](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/gazebo_upgrade.md)

# install
```
cd ~/catkin_ws/src
git clone https://github.com/tamago117/eg_navigation
git clone https://github.com/tamago117/control_panel_plugin
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# demo
```
#2d
roslaunch eg_navigation navigation_2d.launch
```
![Screenshot from eg_navigation_2d_safety-2021-11-06_17 52 45 webm](https://user-images.githubusercontent.com/38370926/140604530-01eee6b0-c831-4618-b33a-4c3bcf2c967c.png)

# packages

## eg_mixed
mixed node for robot navigation

## eg_navigation
sample launch

## eg_planner
planner

## eg_safety
safety and recover

## eg_track
tracking

## eg_wptool
way point tools
