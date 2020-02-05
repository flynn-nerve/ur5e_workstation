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
  //this->move_group_ptr->setEndEffector("endeffector");
  moveit_msgs::Grasp grasp;
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  this->move_group_ptr->setEndEffectorLink("tcp_link");

  grasp.grasp_pose.header.frame_id = "base_link";
  this->q.setRPY(this->orientation.x - 3.14, this->orientation.y, this->orientation.z);
  this->q.normalize();
  grasp.grasp_pose.pose.orientation = tf2::toMsg(this->q);
  grasp.grasp_pose.pose.position.x = this->pose_sample.x;
  grasp.grasp_pose.pose.position.y = this->pose_sample.y;
  grasp.grasp_pose.pose.position.z = this->pose_sample.z;

  grasp.pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasp.pre_grasp_approach.direction.vector.x = 1.0;
  grasp.pre_grasp_approach.min_distance = 0.05;
  grasp.pre_grasp_approach.desired_distance = 0.07;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasp.post_grasp_retreat.direction.vector.x = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.desired_distance = 0.07;

  //============================================================================================================================
  // GRASP POSTURE SETTINGS
  //============================================================================================================================
/*
  // pre grasp posture
  // +++++++++++++++++
  grasp.pre_grasp_posture.joint_names.resize(2);
  grasp.pre_grasp_posture.joint_names[0] = "robotiq_2f_85_body_to_robotiq_2f_85_left_finger_joint";
  grasp.pre_grasp_posture.joint_names[1] = "robotiq_2f_85_body_to_robotiq_2f_85_right_finger_joint";

  // Set them as open, wide enough for the object to fit.
  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.resize(2);
  grasp.pre_grasp_posture.points[0].positions[0] = 0.0;
  grasp.pre_grasp_posture.points[0].positions[1] = 0.0;
  grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);
  
  // post grasp posture
  // +++++++++++++++++
  grasp.grasp_posture.joint_names.resize(2);
  grasp.grasp_posture.joint_names[0] = "robotiq_2f_85_body_to_robotiq_2f_85_left_finger_joint";
  grasp.grasp_posture.joint_names[1] = "robotiq_2f_85_body_to_robotiq_2f_85_right_finger_joint";

  // Set them as open, wide enough for the object to fit.
  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.resize(2);
  grasp.grasp_posture.points[0].positions[0] = 0.0;
  grasp.grasp_posture.points[0].positions[1] = 0.0;
  grasp.grasp_posture.points[0].time_from_start = ros::Duration(0.5);
*/
  //============================================================================================================================

  ROS_INFO("at pick stage");
  this->move_group_ptr->setSupportSurfaceName("workstation");
  this->move_group_ptr->pick("object");  //, grasp);
  //this->move_group_ptr->attachObject("object", "tcp_link");

  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation2;
  orientation2.setRPY(-3.14, 0, 1.57);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation2);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = -0.35;
  place_location[0].place_pose.pose.position.y = -0.665;
  place_location[0].place_pose.pose.position.z = 0.09;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.x = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.03;
  place_location[0].pre_place_approach.desired_distance = 0.07;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.x = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.03;
  place_location[0].post_place_retreat.desired_distance = 0.07;

  ROS_INFO("at place stage");
  this->move_group_ptr->setSupportSurfaceName("dropoffbox");
  this->move_group_ptr->place("object", place_location);
  //this->move_group_ptr->detachObject("object");
}
