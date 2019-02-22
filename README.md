# Autonomous-Go-Kart
These files are for the decision making algorithm using a LIDAR and Sonar sensors for an autonomous go-kart project.

# Folder Files

The "Arduino Master" folder consists of the arduino code to control and retrieve data from the ultrasonic sensors, 
the stepper motor control, and the potentiometer.
The "Catkin Workspace" folder consists of all of the ROS files. This consists of code to control and retrieve data from the RPLIDAR.
The data retreived from the arduino also gets sent here and everything is evaluated for the master decision making algorithm.
Also within this folder, multiple different versions of this code can be found (i.e. "Master_critsub.cpp" and "Master_MStone1.cpp"),
this was for the various milestones/demonstrations for the project. 


There exists one ultrasonic sensor on the right side of the go-kart vehicle and one on the left side of the vehicle.
The ultrasonic sensors are used to send flags if the vehicle sides are "too close" to a wall or obstacle. There is a "warning zone"
that if thrown will dampen the optimal wheel angle the vehicle should turn to in order to avoid hitting the wall/obstacle.
There is also a "danger zone" that replaces the optimal wheel angle with either a "-5" or "+5" degrees to begin moving away from the
wall/obstacle.

The RPLIDAR is mounted on the center front of the vehicle. The decision making process flowchart can be found in a file titled
"Flowchart_Logic" for more information.



# How to instantiate your ROS program

- Before runnin programs using ROS, you must start the ROS Core process. Simply open up a terminal and type/enter the following:
$ roscore

(The terminal window will then show you a couple of pieces of useful information, namely the distribution and version of ROS)

- Open a new terminal and change your working directory to the ROS project folder:
$ cd "your folder path"

- To run programs, you must source the devel.bash file by using the line"
$ . ~/catkin_ws/devel/setup.bash

(This must be done for every terminal you will run programs in)

- You can now run programs using the following syntax:
$ rosrun "package_name" "program_name"

(i.e. $ rosrun cpp_programs Master)

# To Compile Programs

- Change directory back to catkin_ws
Type and run the following:
$ catkin_make

NOTE: You must make sure your program is listed in the package's "Cmakelists" text file with the proper file directory.
