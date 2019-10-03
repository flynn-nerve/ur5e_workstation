// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// pose_goal.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"

// set_target_pose function
// --------------------------------
// Using stored pose and orientation variables;
// determine quaternion using desired orientation values and 
// update target_pose for current goal
void Manipulation::set_target_pose()
{
  float pi = 3.14;
  this->q.setRPY(-pi, 0, pi/2);
  this->q.normalize();
  this->target_pose.orientation = tf2::toMsg(this->q);
  this->target_pose.position.x = this->x_pos;
  this->target_pose.position.y = this->y_pos;
  this->target_pose.position.z = 0.24; //this->z_pos;
}

void Manipulation::set_dropoff_pose()
{
  this->q.setRPY(-3.14, 0, 1.57);
  this->q.normalize();
  this->target_pose.orientation = tf2::toMsg(this->q);
  this->target_pose.position.x = -0.35;
  this->target_pose.position.y = -0.65;
  this->target_pose.position.z = 0.24;		//.0025; //this->z_pos;
}

// plan_pose_goal function
// --------------------------------
// Using stored variables; plan movement to target_pose
void Manipulation::plan_pose_goal()
{
  this->move_group_ptr->setPoseTarget(this->target_pose);
  this->move_group_ptr->setGoalPositionTolerance(0.002);
  this->move_group_ptr->setGoalOrientationTolerance(0.005);
  this->move_group_ptr->setPlanningTime(5);
  this->move_group_ptr->setNumPlanningAttempts(30);
  this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

// move_to_pose_goal function
// --------------------------------
// Using stored variables; move to current jointValueTarget
void Manipulation::move_to_pose_goal()
{
  const robot_state::JointModelGroup* joint_model_group =
    this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->move_group_ptr->setGoalPositionTolerance(0.002);
  this->move_group_ptr->setGoalOrientationTolerance(0.005);
  this->move_group_ptr->move();
}
