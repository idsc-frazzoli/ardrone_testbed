# AR drone workspace
This git repo contains the ROS packages for running the AR drone autonomously

## Code Style Suggestions
class names: MyClass
class object names: myObject
variable names: my_variable
file_names: my_header.h, my_source_file.cpp

All member variables in a class should be private with commented functions for accessing (or modifying) those variables.

## Packages
--ardrone_autonomy
This is the driver that lets us communicate with the drone through ROS. It is more or less identical to the repo here:

[ARdrone driver repo](https://github.com/AutonomyLab/ardrone_autonomy) 

--ardrone_controller
This will ultimately contain the controller node that interprets the motion plan and determince actual control inputs

ORB_SLAM2: This is the monocular slam code taken from here:

[Monocular slam repo](https://github.com/raulmur/ORB_SLAM2) 

You will have to follow the setup instructions in the readme for this package VERY CAREFULLY. 

--ardrone_orb: This package simply contains some launch files and the settings file for the slam code. The ardrone camera calibratio parameters have been incorporated into settings.yaml


##Catkin Setup
1) You should create a local catkin workspace for the packages. Wherever you will have the top level directory run:

```
mkdir ardrone_ws && cd ardrone_ws && mkdir src 
``` 
2) Go into the src folder and clone the project

```
cd ardrone_ws/src
git clone git@github.mit.edu:bapaden/ardrone_glc.git
```

3) Go to the ORB_SLAM folder:

```
cd ardrone_ws/src/ardrone_glc/ORB_SLAM
```

4) Install the dependancies described here: [Monocular slam repo](https://github.com/raulmur/ORB_SLAM2)

5) Compile ORB_SLAM. In the ORB_SLAM directory:

```
build_ros.sh
```

6) create a catkin_workspace

```
cd ardrone_ws/src/
catkin_init_workspace
``` 

7) Run catkin_make to compile everything in the project

```
cd ardrone_ws/
catkin_make
```

