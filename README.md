
# diff_drive_bot

This ROS package implements SLAM on a 2 wheeled differential drive robot to map an unknown environment. A joystick is used to teleoperate the robot in Gazebo. The map generated is then used for autonomous navigation using the ROS Navigation stack.

[Demo Video](https://youtu.be/jbd2p1llsqA) \
[Blog Post](https://devanshdhrafani.github.io/blog/2020/11/01/diffdrive.html)

## Installation
1. Build package from source: navigate to the source folder of your catkin workspace and build this package using:
	```
	$ git clone https://github.com/devanshdhrafani/diff_drive_bot.git
	$ cd ..
	$ catkin_make
	```
2. Install Required dependencies:
	```
	$ sudo apt-get install ros-melodic-dwa-local-planner
	$ sudo apt-get install ros-melodic-joy
	```

## Simultaneous Localization And Mapping (SLAM)

The package uses [slam_gmapping](http://wiki.ros.org/slam_gmapping) to map the environment. For the purpose of this demonstration, we use the Gazebo simulation environment to move around the robot. 

![SLAM Screenshot](https://github.com/devanshdhrafani/diff_drive_bot/raw/master/screenshots/slam_gmapping_resized.gif)

1. Load the robot in the Gazebo environment. Default model is the turtlebot3_house. You can change this from ```/worlds/mybot.world```. To continue with default model:
	```
	$ roslaunch diff_drive_bot gazebo.launch 
	```
2. Launch the **slam_gmapping** node. This will also start **rviz** where you can visualize the map being created:
	```
	$ roslaunch diff_drive_bot gmapping.launch
	```
3. Move the robot around. If you have a Joystick, use:
	 ```
	 $ roslaunch diff_drive_bot joy_teleop_launch.launch
	 ```
	 OR 
	 teleop using keyboard:
	 ```
	 $ rosrun diff_drive_bot keyboard_teleop.py 
	 ```
4. Move the robot in your environment till a satisfactory map is created. 
5. Save the map using:
	```
	$ rosrun map_server map_saver -f ~/test_map
	```
6. Copy the map file to ```~/diff_drive_bot/maps/``` directory and edit the .yaml file to match the path. 
	
## Autonomous Navigation
This package uses the [ROS Navigation stack](http://wiki.ros.org/navigation) to autonomously navigate through the map created using gmapping. 

![Navigation](https://raw.githubusercontent.com/devanshdhrafani/diff_drive_bot/master/screenshots/autonomous_navigation.png)
  
0. To use your generated map, edit ```/launch/amcl_move_base.launch``` and add map .yaml location and name to map_server node launch.
1. Load the robot in gazebo environment:
	```
	$ roslaunch diff_drive_bot gazebo.launch 
	```
2. Start the **amcl**, **move_base** and **rviz** nodes:
	```
	$ roslaunch diff_drive_bot amcl_move_base.launch
	```
3. In rviz, click on ***2D Pose Estimate*** and set initial pose estimate of the robot.
4. To move to a goal, click on ***2D Nav Goal*** to set your goal location and pose.  

##  [Optional] Joystick Configuration 

To make it easier to map environments, I added a joystick_teleop node to control the robot movement using my xbox controller. If you are using some other controller, you can easily map your buttons:

1. Install the ROS [joy](http://wiki.ros.org/joy) package:
	``` $ sudo apt-get install ros-melodic-joy``` 
2. Connect your Jotstick to your machine and check if its detected:
	```	$ ls /dev/input/```
3. If everything worked, your joystick should show up as jsX. In my case, it showed up as js1.
4. Go to ```/launch/joy_teleop_launch.launch``` and edit the dev parameter value to ```/dev/input/jsX```.
5. Open the ```joy_teleop.py``` script in the ```/scripts/``` folder.
6.  Uncomment the print statements in the ```joyCallback()``` function.
7. Save and run the script using:
	```$ roslaunch diff_drive_robot joy_teleop_launch.launch ```
8. You will see 2 arrays corresponding to the axes and buttons of your Joystick. Press each button/stick and find the index of your controls. Change the ```joy_teleop.py``` script with your respective axes.
