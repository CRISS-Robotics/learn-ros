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
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/main/control.yaml
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

10. Move robot

```sh
sudo apt install ros-noetic-rqt-robot-steering
```

```sh
rosrun rqt_robot_steering rqt_robot_steering
```

```sh
rostopic list
```

```sh
/robot_base_velocity_controller/cmd_vel
```

```sh
rostopic echo /robot_base_velocity_controller/cmd_vel
```

11. Make robot arm urdf

```sh
roscd mobile_manipulator_body/urdf/
```

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/main/robot_arm.urdf
```

12. Test the arm

```sh
cd ~/catkin_ws/src
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
roslaunch urdf_tutorial display.launch model:=robot_arm.urdf
```

```sh
Change the Fixed Frame to world
```

13. Add control for arm

```sh
roscd mobile_manipulator_body/config/
```

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/arm_control.yaml
```

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/joint_state_controller.yaml
```

14. Add arm launch code

```sh
roscd mobile_manipulator_body/launch/
```

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/main/arm_gazebo_control.launch
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
roslaunch mobile_manipulator_body arm_gazebo_control.launch
```

15. Test arm

```sh
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["arm_base_joint","shoulder_joint", "bottom_wrist_joint", "elbow_joint","top_wrist_joint"], points: [{positions: [-0.1, 0.5, 0.02, 0, 0], time_from_start: [1,0]}]}' -1
```

```sh
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["arm_base_joint","shoulder_joint", "bottom_wrist_joint", "elbow_joint","top_wrist_joint"], points: [{positions: [0, 0, 0, 0, 0], time_from_start: [1,0]}]}' -1
```

16. Combine Arm and Body

```sh
roscd mobile_manipulator_body/urdf/
```

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/main/mobile_manipulator.urdf
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
roscd mobile_manipulator_body/urdf/
```

```sh
roslaunch urdf_tutorial display.launch model:=mobile_manipulator.urdf
```

17. Control the robot

```sh
roscd mobile_manipulator_body/launch/
```

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/mobile_manipulator_gazebo.launch
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
roslaunch mobile_manipulator_body mobile_manipulator_gazebo.launch
```

18. Place this in a world

```sh
wget https://raw.githubusercontent.com/CRISS-Robotics/learn-ros/main/postoffice.world
```

19. Add this code to launch file

```sh
  <!-- Gazebo post office environment -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="/home/test/nav_ws/src/mobile_manipulator/worlds/postoffice.world"/>
  </include>
```
