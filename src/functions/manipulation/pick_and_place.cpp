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

  this->move_group_ptr->setMaxVelocityScalingFactor(0.05);
  this->move_group_ptr->setMaxAccelerationScalingFactor(0.05);
  this->move_group_ptr->setEndEffectorLink("tool0");
  //this->move_group_ptr->setEndEffector("endeffector");
  
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0.7, 0.7, 0);							//(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = this->x_pos;				//-0.35;
  grasps[0].grasp_pose.pose.position.y = this->y_pos;				//0;
  grasps[0].grasp_pose.pose.position.z = 0.3;				//0.15;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // pre grasp posture
  // +++++++++++++++++
  //grasps[0].pre_grasp_posture.joint_names.resize(2);
  //grasps[0].pre_grasp_posture.joint_names[0] = "robotiq_2f_85_body_to_robotiq_2f_85_left_finger_joint";
  //grasps[0].pre_grasp_posture.joint_names[1] = "robotiq_2f_85_body_to_robotiq_2f_85_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  //grasps[0].pre_grasp_posture.points.resize(1);
  //grasps[0].pre_grasp_posture.points[0].positions.resize(2);
  //grasps[0].pre_grasp_posture.points[0].positions[0] = 0.0;
  //grasps[0].pre_grasp_posture.points[0].positions[1] = 0.0;
  //grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);
  
  // post grasp posture
  // +++++++++++++++++
  //grasps[0].grasp_posture.joint_names.resize(2);
  //grasps[0].grasp_posture.joint_names[0] = "robotiq_2f_85_body_to_robotiq_2f_85_left_finger_joint";
  //grasps[0].grasp_posture.joint_names[1] = "robotiq_2f_85_body_to_robotiq_2f_85_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  //grasps[0].grasp_posture.points.resize(1);
  //grasps[0].grasp_posture.points[0].positions.resize(2);
  //grasps[0].grasp_posture.points[0].positions[0] = 0.0;
  //grasps[0].grasp_posture.points[0].positions[1] = 0.0;
  //grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

  this->move_group_ptr->setSupportSurfaceName("workstation");
  this->move_group_ptr->pick("object", grasps);

  this->move_group_ptr->attachObject("object", "tcp_link");

  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation2;
  orientation2.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation2);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = -0.35;
  place_location[0].place_pose.pose.position.y = -0.65;
  place_location[0].place_pose.pose.position.z = 0.09;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  this->move_group_ptr->setSupportSurfaceName("dropoffbox");
  this->move_group_ptr->place("object", place_location);
}
