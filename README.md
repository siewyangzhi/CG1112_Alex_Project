# Alex Project - CG1112

## by Wong Li Ping, Siew Yang Zhi, Christopher Langton, Falicia Ong


## Table of Content
[Introduction to Alex Project](#introduction-to-alex-project)  
[Instructions on LiDAR](#instructions-on-lidar)  


### Introduction to Alex Project
This project builds a robotic vehicle, Alex, with search and rescue functionalities. Alex will be tele‐operated from your laptop. 
Alex functionalities include its ability to precisely map out and manoeuvre through the obstacle course's environment, colour detection and parking into a 30 by 30 cm slot.


### Instructions on LiDAR

##### ENSURE THE LIDAR IS PLUCK IN

In raspberry pi,  

source ~/Desktop/slam/devel/setup.bash  
export ROS_MASTER_URI=http://192.168.43.66:1311 [ip of PI, read from the top of VNC Viewer]  
export ROS_IP=192.168.43.66 [ip of PI]  

In your laptop,  
run the script using the command `./ros_start.sh`  

The script will run the following: {  
source ~/Desktop/slam/devel/setup.bash  
export ROS_MASTER_URI=http://192.168.43.200:1311 [this will always be the ip of the PI]  
export ROS_IP=192.168.43.212 [ip from LAPTOP]  
}  

OR   

simply type  
source ~/Desktop/slam/devel/setup.bash  
export ROS_MASTER_URI=http://192.168.43.200:1311 [this will always be the ip of the PI]  
export ROS_IP=192.168.43.212 [ip from LAPTOP]  

In raspberry pi,  
cd Desktop/slam  
roslaunch rplidar_ros rplidar.launch  


In your laptop,  
cd Desktop/slam  
rosrun rplidar_ros rplidarNodeClient  


The next step will open rviz map:  
In your laptop,  
roslaunch rplidar_ros view_slam.launch  


#### To configure Alex's object on Rviz map  

Click add > by topic > pose  

To configure the red arrow properly to front, back and left:  
Alpha: 1  
Shaft Length: 0,9  
Shaft Radius: 0  
Head Length: 0,3  
Head Radius: 0,1  

Change the red arrow to yellow colour  

The TF represents the right of the robot [green in colour].  

MakerScale: 0,4  
Show Axes: TICK, everything else untick  
Laser: TICK  


