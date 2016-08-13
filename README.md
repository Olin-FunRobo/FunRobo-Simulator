# Olin O Virtual Simulation

Fundamentals of Robotics 2016-2017

Virtual environment and template 4-wheeled robot

For software testing purposes

# Installation

Requires a maximum storage space of 680 MB available.

(tested in a bare chroot environment)

```bash
cd ~/catkin_ws/src/
git clone https://github.com/yycho0108/olin_funrobo.git
cd ~/catkin_ws
catkin_make
rosdep install olin_control olin_description olin_gazebo
```

# Launching

```bash
roslaunch olin_gazebo all.launch
```
