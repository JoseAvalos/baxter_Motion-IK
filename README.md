# Baxter Motion Inverse Kinematic
The steps to reproduce this project will be explain. The Baxter robot could follow human motion using Inverse Kinematic (IK). In this work, we are working to develop to obtain human motion position and reproduce them in the Baxter. The project is develop for Ubuntu 14.04 using ROS. 


<a href="http://www.youtube.com/watch?feature=player_embedded&v=1cECGg1YLfo
" target="_blank"><img src="http://img.youtube.com/vi/1cECGg1YLfo/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="360"  align="center" border="10" /></a>


##0-Dependence
###Workspace
It necessary to get the defult workspace for the Baxter. In http://sdk.rethinkrobotics.com/wiki/Workstation_Setup are the step to get it. 
###Libraries
In source_lib directory there is a osik-full.zip file. You must decompress it. After that:
```
cd ext
./install-ext   ~/devel/install -a
cd ...
mkdir build
cd build
cmake ..
make
```
##1.-Ros Packages
This git contain the files neccesary to work as another packages. You could edit the "CMakeLists.txt" file to modify the name of the project. After that you could compile the whole workspace.
```
roscd 
cd ..
catkin_make
```

Also you need to get the packages https://github.com/JoseAvalos/baxter_kinect360. In order, to process the data from the kinect.

##2-Robot Conection


Before to execute avahi is recommedable eneable networking in the computer.
```
roscore
```




```
sud avadi-autoipd eth0
```
It will set a IP for the baxter. Go to the baxter-workspace and edit baxter.sh (default name) file. You have to change the variable your_ip. After that, execute.


```
./baxter.sh
```


To prove the connection, we enable the robot
```
rosrun baxter_tools enable_robot.py -e
```

You will se the direcction in the terminal. It means that you could interact with the robot in just this windows. 

##3.-Image Processing
The camera will detect a person using the kinect. In order to transfer this information with the workspace of the baxter, we  have to open the ./baxter.sh before to :
```
Terminal_1:
roslaunch openni_launch openni.launch

Terminal_2:
rosrun openni_tracker openni_tracker
```
As recomendation one person has to be working in the laptop and another in fton of the kinect. Avoid more than one person because the topic will generate som much noise. In the terminal will apper user 1, if you see user 2, yoyu have to reset the program. 

##4.-Reproduce
Finally, We could reproduce the real motion in the baxter. We have the option of work with two hands.

```
rosrun baxter_cpp OSIK_baxter 
```

