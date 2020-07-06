# gazebo_ros_demo
Gazebo simulation demo of a robot chasing a white ball. 
 
![](doc/demo.gif)
 
The robot is driven by a pair of [differential wheels](src/ball_chaser/src/drive_bot.cpp). And a simple [recognition lib](src/ball_chaser/src/process_image.cpp) was written to process the raw image from the camera feed & control the robot accordingly. A simple crosshair is also added to show the location of the recognized ball.

The robot is controlled by differential drive on the back wheels, while the front wheels are programmed to be relatively slippery so they can allow the robot to turn. This is because gazebo_ros_diff_drive does not allow skid steering. In the future I might replace it with ROS [diff_driver_controller](http://wiki.ros.org/diff_drive_controller) to address this.

The visual recognition logic is very naive and relies on the color of the ball being perfectly white, since this is not a computer vision demo. But improvements with OpenCV are possible.

# Usage

Build:

`cd src; catkin_init_workspace; cd..`

`catkin_make`

`source devel/setup.sh`

Launch the simulation:

`roslaunch my_robot world.launch`

Launch the ball chaser:

`roslaunch ball_chaser ball_chaser.launch`

After startup the robot will try to locate the white ball with the camera and move towards the ball. You could drag the white ball in Gazebo editor to change its location.

# Dependencies

I created this project with the help of tutorials in Udacity [ND209](https://www.udacity.com/course/robotics-software-engineer--nd209) course, plus some personal spice. Dependencies I've used:

* libgazebo_ros_camera.so
* libgazebo_ros_diff_drive.so
* Plenty of world object models from [Gazebo model online database](http://github.com/osrf/gazebo_models)

(Hokuyo Lidar meshes and libgazebo_ros_laser.so is also included for future convenience but not used now in this demo)
