// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// grasp_cluster_class.hpp
// ********************************************************************************************

#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

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

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class Manipulation
{
  public:

    //Constructor
    Manipulation()
    {
      PLANNING_GROUP = "manipulator";
    }

    //Members
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    MoveGroupPtr move_group_ptr;
    PlanningScenePtr planning_scene_ptr;
    std::string PLANNING_GROUP;
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state;
    tf2::Quaternion q;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Vector3 orientation;
    bool pose_success;
    double x_pos;
    double y_pos;
    double z_pos;
    std_msgs::Int8 instruction;
    moveit_msgs::CollisionObject collision_object;
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;

    //Publisher
    ros::Publisher gripper_instruction;
    ros::Publisher gripper_command;

    //Functions
    Manipulation(ros::NodeHandle nodeHandle)
    {
      PLANNING_GROUP = "manipulator";
      this->gripper_command = nodeHandle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);
    }
    void init_gripper_utils(ros::NodeHandle nodeHandle);
    void move_to_left();
    void move_to_right();
    void move_to_top();
    void move_to_front();
    void move_to_wait_position();
    void pick_and_place();
    void pick_and_place_raw();
    void create_object();
    void generate_workspace();
    void set_target_pose();
    void set_dropoff_pose();
    void plan_pose_goal();
    void move_to_pose_goal();
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
};  

#endif // MANIPULATION_CLASS
