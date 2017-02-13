# AR drone workspace
This git repo contains the ROS packages for running the AR drone autonomously

##Packages
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
1) You should create a local catking workspace for the packages:

```
mkdir ardrone_ws && cd ardrone_ws && mkdir src 
``` 
2) Clone the packages in the src folder

```
cd ardrone_ws/src
git clone git@github.mit.edu:bapaden/ardrone_glc.git
```

3) Build ORB_SLAM without catkin since it uses the old ROS build system. Follow the directions on the orb slam git repo carefully.

4) Initialize the catkin workspace

```
cd ardrone_ws/ardrone_glc/src
catkin_init_workspace
cd ..
catkin_make
```
 



