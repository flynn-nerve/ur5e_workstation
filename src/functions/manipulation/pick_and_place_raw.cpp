// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// pick_and_placet.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"

void Manipulation::pick_and_place_raw()
{
  //const robot_state::JointModelGroup* joint_model_group =
  //  this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->move_group_ptr->setEndEffector("endeffector");
  //this->move_group_ptr->setEndEffectorLink("tool0");
  this->move_group_ptr->setSupportSurfaceName("workstation");
  this->move_group_ptr->planGraspsAndPick("object");
  this->move_group_ptr->setSupportSurfaceName("dropoffbox");
  this->move_group_ptr->place("object");
}
