# solder_paste_workspace

## Build and Source Instructions

First things first you need to run `catkin_make` in the root of the ROS Workspace (Also the root of the git repo)

This will add two folders `build` and `devel`. In `devel` there are three files called `setup.bash setup.sh setup.zsh`. These are the setup files in the syntax of three common shells. As I'm pretty sure I'm the only guy hipster enough to use `zsh` for my shell, I'll explain this setup for `bash` but the method is the same for all three. 

In your `.bashrc` file you need to add the line `source /path/to/solder_paste_workspace/devel/setup.bash` anywhere in the file. `/path/to` being the path to wherever you placed the workspace. 

I would also reccomend adding this line:

`alias bashupdate="source ~/.bashrc"`

This allows you to run the command `bashupdate` which will re-source your `.bashrc` file. This is especially important when you make new packages or executables, otherwise they won't show up in `rosrun` or `roslaunch` without first closing and reopening your terminal. This just lets you skip that step.


## Creating Packages

[ROS Tutorials for how to create ROS Packages](http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage)

Make sure to know what your package will rely on. If you are editing a known package you can set the dependencies to match the listed dependencies on that packages ROS page. Otherwise if you keep track of what your stuff relys on you just have to put the name of the packages in the depends arguments.

For example. If I am writing a node that used the [vision_opencv](http://wiki.ros.org/vision_opencv) package (I use it for the `image_transport` package mainly) I would create my package by `catkin_create_package MY_PACKAGE_NAME vision_opencv roscpp rospy std_msgs ETC_DEPENDENCIES` 


## Dependences
1. moveit!
2. ROS industrial
3. ROS Serial
4. USB Camera

## Simulation using Gazebo
1. Launch gazebo environment for abb_irb120 (more objects to be added)
roslaunch abb_irb120_gazebo irb120_gazebo2.launch

2. Launch move_group with rviz for visualization
roslaunch abb_irb120_moveit_config move_group.launch 

3. Run motion control programe which calls move_group_interface
rosrun moveit_tutorials move_group_interface_tutorial


## Simulation using Robot Studio
1. Follow the official tutorial to set up
http://wiki.ros.org/abb/Tutorials

2. Connect Robot Studio with ROS
roslaunch abb_irb120_support robot_interface_download_irb120.launch robot_ip:=ip_address
my address is 192.168.1.102

3. Launch moveit planning execution and don't run the following commands if you just want to see how it works
roslaunch abb_irb120_moveit_config moveit_planning_execution.launch

4. Or Launch move group if you want to control the robot by your code
roslaunch abb_irb120_moveit_config move_group.launch

5. Run the robot
rosrun moveit_nodes move_group_interface
In rviz, choose Key Tool and click 'n' in keyboard for next step.
After running this, you can see the robot moves to several poses.

## Work with real Robot
1. Connect Robot with Ethernet cable and build a connection between Ubuntu and the robot

2. Connect ROS to the robot
roslaunch abb_irb120_support robot_interface_download_irb120.launch robot_ip:=192.168.125.1

3. Follow the steps in 5.3

## Run camera using usb_camera in ROS
rosrun usb_cam usb_cam_node _video_device:=/dev/video1 _camera_name:=cam1

## Final usage guide
1. Connect Robot with Ethernet cable and build a connection between Ubuntu and the robot

2. Connect ROS to the robot
roslaunch abb_irb120_support robot_interface_download_irb120.launch robot_ip:=192.168.125.1

3. Run gerber import server
rosrun gerber_import gerber_import

4. Run pneumatic control server
rosrun paste_arduino PneumaticControl.py 

5. Run trajectory generation 
rosrun abb_control trajectory_generator.py

6. Run soldering paste task
rosrun abb_control solder_paste.py 

