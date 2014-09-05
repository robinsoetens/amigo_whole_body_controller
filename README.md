amigo_whole_body_controller
===========================
Whole-Body Control Framework for the Amigo service robot at the TU/e
---------------------------

Installation
------------
If you have a [TU/e installation](http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=PC_Configuration), just use the following command, that should pull all the dependencies.
```
tue-get install amigo_whole_body_controller
```
If you prefer a standard ROS installation, take the following steps.

1. install [ROS](http://www.ros.org/wiki/ROS/Installation)

2. clone the stack repos into your catkin workspace.
   - `$ cd ~/catkin_ws`
   - `$ wstool set amigo_whole_body_controller --git`
   - `$ wstool up`

3. build it using catkin
   - `$ catkin_make`

Usage
-----
To run the controller, use the `start.launch` launchfile.
```
$ roslaunch amigo_whole_body_controller start.launch
```
