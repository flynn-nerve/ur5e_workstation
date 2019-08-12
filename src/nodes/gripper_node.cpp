// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// gripper_node.cpp
//
// manipulation_class node
// ********************************************************************************************

#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <array>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <moveit/move_group/capability_names.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <std_msgs/Int8.h>

class Gripper
{
  public:

  // Members  
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;

  // Publishers and Subscribers
  ros::Publisher gripper_command;
  ros::Subscriber gripper_instruction;

  // Functions
  Gripper(ros::NodeHandle nodeHandle)
  {
    this->gripper_command = nodeHandle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);
    this->gripper_instruction = nodeHandle.subscribe("gripper_instruction", 1, &Gripper::gripper_instruction_callback, this);
  }
  void activate_gripper()
  {
    this->command.rACT = 1;
    this->command.rGTO = 1;
    this->command.rSP = 255;
    this->command.rFR = 150;
  }
  void gripper_open()
  {
    this->command.rPR = 0;
  }
  void gripper_close()
  {
    this->command.rPR = 255;
  }
  void gripper_instruction_callback(const std_msgs::Int8 msg)
  {
    if(msg.data == 0)
    {
      this->gripper_open();
      this->gripper_command.publish(this->command);
    }
    else if(msg.data == 1);
    {
      this->gripper_close();
      this->gripper_command.publish(this->command);
    }
  }

};

int main(int argc, char** argv)
{
  // ros initialization
  ros::init(argc,argv,"gripper_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Gripper gripper(nh);

  gripper.activate_gripper();
  gripper.gripper_command.publish(gripper.command);
  ros::Duration(1.5).sleep();
  gripper.gripper_open();
  gripper.gripper_command.publish(gripper.command);
  ros::Duration(1.5).sleep();
  
  while(ros::ok())
  {
    //gripper.gripper_command.publish(gripper.command);
    //ros::Duration(0.1).sleep();
  }

  ros::waitForShutdown();
  return 0;
}
