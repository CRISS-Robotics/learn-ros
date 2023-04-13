# learn-ros
Simple example to learn ROS basics

Instructions:

1. Download dependencies

```sh
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
```

2. Make and move into catkin workspace

```sh
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
```

3. Create a package for mobile manipulator

```sh
catkin_create_pkg mobile_manipulator_body std_msgs roscpp rospy
```

```sh
cd ~/catkin_ws/
```

```sh
catkin_make --only-pkg-with-deps mobile_manipulator_body
```

```sh
source devel/setup.bash
```

4. Create folders for project

```sh
roscd mobile_manipulator_body
```

```sh
mkdir config launch urdf
```

5. Download meshes

```sh
git clone -b meshes https://github.com/CRISS-Robotics/learn-ros.git
```

```sh
mv learn-ros/ meshes/
```

6. Create URDF

```sh
cd ..
```

```sh
cd urdf
```

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/main/robot_base.urdf
```

7. Check robot in RVIZ

```sh
cd ~/catkin_ws/
```

```sh
catkin_make
```

```sh
source devel/setup.bash
```

```sh
roscd mobile_manipulator_body/urdf/
```

```sh
roslaunch urdf_tutorial display.launch model:=robot_base.urdf
```

8. Control your robot

```sh
roscd mobile_manipulator_body/config/
```

```sh
https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/main/control.yaml
```

9. Launch in gazebo

```sh
roscd mobile_manipulator_body/launch/
```

```sh
jj
```

```sh
cd ~/catkin_ws/
```

```sh
catkin_make
```

```sh
source devel/setup.bash
```

```sh
roslaunch mobile_manipulator_body base_gazebo_control.launch
```
