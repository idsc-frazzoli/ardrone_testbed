## AR drone workspace
This git repo contains the ROS packages for running the AR drone autonomously

#Packages
--ardrone_autonomy
This is the driver that lets us communicate with the drone through ROS. It is more or less identical to the repo here:

[ARdrone driver repo](https://github.com/AutonomyLab/ardrone_autonomy) 

--ardrone_controller
This will ultimately contain the controller node that interprets the motion plan and determince actual control inputs

ORB_SLAM2: This is the monocular slam code taken from here:

[Monocular slam repo](https://github.com/raulmur/ORB_SLAM2) 

You will have to follow the setup instructions in the readme for this package VERY CAREFULLY. 

--ardrone_orb: This package simply contains some launch files and the settings file for the slam code. The ardrone camera calibratio parameters have been incorporated into settings.yaml


#Catkin Setup
Clone the 

'''
git clone 
'''
