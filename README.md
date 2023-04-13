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
mkdir config launch meshes urdf
```

```sh
cd meshes
```

