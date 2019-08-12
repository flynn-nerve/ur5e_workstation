// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// manipulation_node.cpp
//
// manipulation_class & perception_class node
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char** argv)
{
  // ros initialization
  ros::init(argc,argv,"manipulation_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // SETUP
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // Create manipulation and perception objects
  Manipulation manipulation(nh);
  Perception perception(nh);

  // Wait for spinner to start
  ros::Duration(1.0).sleep();

  // Transform listener
  perception.transform_listener_ptr = TransformListenerPtr(
      new tf::TransformListener());
  perception.init_subscriber(nh);

  // Planning scene interface
  manipulation.planning_scene_ptr = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface());

  // Moveit interface
  manipulation.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));
  
  // Set useful variables before robot manipulation begins
  manipulation.move_group_ptr->setPlanningTime(45.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);
  //manipulation.move_group_ptr->setPlannerId("RRTConnectkConfigDefault");
  
  manipulation.move_group_ptr->setEndEffector("endeffector");
  manipulation.move_group_ptr->setPoseReferenceFrame("world");
  manipulation.move_group_ptr->setPlannerId("RRTConnectkConfigDefault");
  
  // Add object(s) to planning scene
  manipulation.generate_workspace();

  // Wait for objects to initialize
  ros::Duration(1.0).sleep();

  // Move robot into starting position and wait a moment
  manipulation.move_to_wait_position();
  ros::Duration(1).sleep();

  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // publish gripper close instruction
  manipulation.activate_gripper();
  manipulation.gripper_command.publish(manipulation.command);
  // Wait for gripper to activate
  ros::Duration(1.5).sleep();
  
  while(ros::ok())
  {
    // Move into each position and halt for a moment to capture pointcloud snapshot
    //***********************************************
    manipulation.move_to_top();
    ros::Duration(1).sleep();
    perception.take_snapshot_top();
    ros::Duration(1).sleep();

    manipulation.move_to_left();
    ros::Duration(1).sleep();
    perception.take_snapshot_left();
    ros::Duration(1).sleep();

    manipulation.move_to_right();
    ros::Duration(1).sleep();
    perception.take_snapshot_right();
    ros::Duration(1).sleep();

    manipulation.move_to_wait_position();

    //manipulation.move_to_front();
    //ros::Duration(1).sleep();
    //perception.take_snapshot_front();
    //ros::Duration(1).sleep();

    // Concatenate the pointclouds and run filters on them
    //perception.concatenate_clouds();

    // Publish concatenated cloud
    //perception.publish_combined_cloud();

    //***********************************************

    // create a cylinder object for testing
    manipulation.create_object();

    // Wait for objects to initialize
    ros::Duration(1.0).sleep();

    // set pose target to pickup, plan and move
    manipulation.set_target_pose();
    manipulation.plan_pose_goal();
    manipulation.move_to_pose_goal();

    // publish gripper close instruction
    manipulation.gripper_close();
    manipulation.gripper_command.publish(manipulation.command);
    // Wait for gripper to close
    ros::Duration(1.5).sleep();

    manipulation.move_group_ptr->attachObject("object", "tcp_link");

    manipulation.move_to_wait_position();

    // set pose target to dropoff, plan and move
    manipulation.set_dropoff_pose();
    manipulation.plan_pose_goal();
    manipulation.move_to_pose_goal();

    // publish gripper open instruction
    manipulation.gripper_open();
    manipulation.gripper_command.publish(manipulation.command);
    // Wait for gripper to open
    ros::Duration(1.5).sleep();
    
    manipulation.move_group_ptr->detachObject("object");    

  }  

  ros::waitForShutdown();
  return 0;

}
