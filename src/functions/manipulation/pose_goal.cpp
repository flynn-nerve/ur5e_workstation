// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// pose_goal.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"


// store_gpd_vals() function
// --------------------------------
// take in grasp candidates produced by gpd
// save as member variables to be used internally
void Manipulation::store_gpd_vals(gpd::GraspConfigList msg)
{
  ROS_INFO("storing gpd vals");
  this->candidates = msg;
  if(candidates.grasps.size() == 0)
  {
    ROS_INFO("error: grasp list is empty");
  }
}

// plan_paths() function;
// --------------------------------
// Capture grasp candidates "snapshot" and iterate through list,
// performing the first possible grasp since gpd outputs
// grasp candidates in a list in  a descending order 
// according to probability of success
void Manipulation::select_and_plan_path() 
{
  int count = sizeof(this->candidates);
  this->score = candidates.grasps[0].score; 

  int i = 0;
  for(i=0; ((this->score.data > -150) && i<10); i++)
  {
    // Store grasp candidate "i" and separate into relevant variables for planning evaluation
    this->grasp = this->candidates.grasps[i];
    this->pose_top = this->grasp.top;
    this->pose_bottom = this->grasp.bottom;
    this->pose_center.x = this->pose_top.x - this->pose_bottom.x;
    this->pose_center.y = this->pose_top.y - this->pose_bottom.y;
    this->pose_center.z = this->pose_top.z - this->pose_bottom.z;
    this->pose_sample = this->grasp.sample;
    this->orientation = this->grasp.approach;
    this->score = this->grasp.score;
    ROS_INFO("this grasp score: %f", this->score.data);
    
    // Set target_pose
    this->set_target_pose();  
    this->plan_pose_goal();
      
    if (this->pose_success) 
    {
      // create a cylinder object for testing
      //this->create_object();
      //this->pick_and_place();
      this->move_to_pose_goal();
      ROS_INFO("plan success");
      break;
    }
  }
}

// set_target_pose function
// --------------------------------
// Using stored pose and orientation variables;
// determine quaternion using desired orientation values and 
// update target_pose for current goal
void Manipulation::set_target_pose()
{
  this->q.setRPY(this->orientation.x - 3.14, this->orientation.y, this->orientation.z);	// -2*pi on either y or x, will test both out, both seem to work for now
  this->q.normalize();
  this->target_pose.orientation = tf2::toMsg(this->q);
  this->target_pose.position.x = this->pose_sample.x;
  this->target_pose.position.y = this->pose_sample.y;
  this->target_pose.position.z = this->pose_sample.z;
}

void Manipulation::set_dropoff_pose()
{
  this->q.setRPY(-1.57, 1.57, 0);
  this->q.normalize();
  this->target_pose.orientation = tf2::toMsg(this->q);
  this->target_pose.position.x = -0.35;
  this->target_pose.position.y = -0.665;
  this->target_pose.position.z = 0.15;
}

// plan_pose_goal function
// --------------------------------
// Using stored variables; plan movement to target_pose
void Manipulation::plan_pose_goal()
{
  this->move_group_ptr->setPoseTarget(this->target_pose);
  this->move_group_ptr->setGoalPositionTolerance(0.001);
  this->move_group_ptr->setGoalOrientationTolerance(0.002);
  this->move_group_ptr->setPlanningTime(5);
  this->move_group_ptr->setNumPlanningAttempts(30);
  this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

// move_to_pose_goal function
// --------------------------------
// Using stored variables; move to current jointValueTarget
void Manipulation::move_to_pose_goal()
{
  //const robot_state::JointModelGroup* joint_model_group =
    //this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->move_group_ptr->setGoalPositionTolerance(0.001);
  this->move_group_ptr->setGoalOrientationTolerance(0.002);
  this->move_group_ptr->move();
}
