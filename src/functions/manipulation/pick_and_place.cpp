// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// pick_and_placet.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"

void Manipulation::pick_and_place()
{
  this->move_group_ptr->setGoalPositionTolerance(0.001);
  this->move_group_ptr->setGoalOrientationTolerance(0.002);
  //this->move_group_ptr->move();

  // Create a vector of grasps to be attempted, currently only creating single grasp.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  grasps[0].grasp_pose.header.frame_id = "robot_base_link";
  this->q.setRPY(this->orientation.x - 3.14, this->orientation.y, this->orientation.z);
  this->q.normalize();  
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(this->q);
  grasps[0].grasp_pose.pose.position.x = this->pose_sample.x;
  grasps[0].grasp_pose.pose.position.y = this->pose_sample.y;
  grasps[0].grasp_pose.pose.position.z = this->pose_sample.z;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "robot_tcp_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0;
  grasps[0].pre_grasp_approach.desired_distance = 0.05;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "robot_tcp_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = -1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.0;
  grasps[0].post_grasp_retreat.desired_distance = 0.05;

  // Setting posture of eef before grasp
  //openGripper(grasps[0].pre_grasp_posture);

  /* Add both finger joints*/
  grasps[0].pre_grasp_posture.joint_names.resize(2);
  grasps[0].pre_grasp_posture.joint_names[0] = "gripper_finger_joint";
  grasps[0].pre_grasp_posture.joint_names[1] = "gripper_right_outer_knuckle_joint";

  /* Set them as open, wide enough for the object to fit. */
  grasps[0].pre_grasp_posture.points.resize(1);
  grasps[0].pre_grasp_posture.points[0].positions.resize(2);
  grasps[0].pre_grasp_posture.points[0].positions[0] = 0.03;
  grasps[0].pre_grasp_posture.points[0].positions[1] = 0.03;
  grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

  //closedGripper(grasps[0].grasp_posture);

  /* Add both finger joints*/
  grasps[0].grasp_posture.joint_names.resize(2);
  grasps[0].grasp_posture.joint_names[0] = "gripper_finger_joint";
  grasps[0].grasp_posture.joint_names[1] = "gripper_right_outer_knuckle_joint";

  /* Set them as closed. */
  grasps[0].grasp_posture.points.resize(1);
  grasps[0].grasp_posture.points[0].positions.resize(2);
  grasps[0].grasp_posture.points[0].positions[0] = 0.3;
  grasps[0].grasp_posture.points[0].positions[1] = 0.3;
  grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

  // Set support surface as table1.
  //move_group.setSupportSurfaceName("table1");

  // Call pick to pick up the object using the grasps given
  this->move_group_ptr->pick("object", grasps);
}
