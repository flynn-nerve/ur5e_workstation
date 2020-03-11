# NERVE Center @ UMASS Lowell: UR5e Workstation Package

This package is currently a work in progress and a new, more general package will be created for the industrial arm robots at the NERVE center once all basic requirements are complete for the UR5e robot.

This package serves to hold all of the launch, urdf, dae, etc. files necessary for the operation of the UR5e system at the NERVE Center with the 2 finger robotiq gripper and realsense d435 camera mouted to the end of the arm. Some old files remain as a point of reference as development moves forward. Files from the urdf folder were used to generate a moveit config package for the robot as well as an IKFast solver for operation of the actual robot.

## Precursory note:

The setup scripts and instructions contained within this README/package all expect that the user is using catkin build to build their development space rather than catkin\_make. 

Catkin\_tools (the package that gives you the catkin build utility among other tools) gives the user more context info when building and setting up a workspace and it the newer of the build options. It is suggested that the user switch to using the catkin\_tools package if they are not already.

## You can set up catkin_tools by heading over to their site or following these instructions:

*https://catkin-tools.readthedocs.io/en/latest/installing.html*

*generally you would have this installed first, but in case you don't I am leaving directions* 

1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
2. wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
3. sudo apt-get update
4. sudo apt-get install python-catkin-tools

###### Get your workspace initialized and ready:

1. cd ~/<your_ws>
2. catkin init
3. catkin build

That's it, you're ready to go.
Their main website has more guides for those interested in exploring additional utilities

## Setup of package and necessary components

A setup script for installing all required packages is located within ur5e_workstation/scripts. The script takes in one argument: the name of your workspace. 

*example:*

I want to install this package and its other required packages in my workspace called catkin_ws:
1. cd ~/catkin_ws/src
2. git clone -b devel https://github.com/flynn-nerve/ur5e_workstation.git
3. cd ..
4. catkin build
5. source ./devel/setup.bash
6. cd src/ur5e_workstation/scripts
7. ./install\_workspace catkin\_ws

_You will need to enter your password for the sudo apt-get... instructions in the beginning, so don't walk away until you have completed that step (probably about 15 seconds after running the script). Installation will take a while after entering your password, so don't forget to do that (~20 minutes if you have none of the libraries installed or haven't set up a realsense camera already)_

## Use of package

Currently, the package is used to run a specific setup; the UR5e industrial arm with a mounting structure that includes a 3d-printed realsense camera mount, gripper quick-change adapter, and a robotiq 2f\_85 gripper. The moveit config package can be used in simulation, but you will want to make your own custom moveit config package if you have a robot with a different hardware setup.

So long as you have installed the other packages (manually or by the script) you can launch a simulation of the robot with the following command:
ur5e_workstation ur5e\_workstation\_v3.launch sim:=true 

Alternatively, you can launch the real robot by providing the ip address as an argument instead if you know it. Once again, this is strongly advised against since you will likely need to make a custom moveit! config package for your specific hardware configuration to ensure that you do not harm the robot, its surroundings, or any users nearby.
ur5e\_workstation ur5e\_workstation_v3.launch ip:=<your.robots.ip.address>


