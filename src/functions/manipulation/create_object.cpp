// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// create_object.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"

void Manipulation::create_object() {
  this->x_pos = this->pose_sample.x;
  this->y_pos = this->pose_sample.y;
  this->z_pos = this->pose_sample.z;
  
  // Create a vector to hold collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add workstation surface
  // ************************************************************************************************ 
  // 0.192 (distance from robot center to edge of workstation)
  collision_objects[0].id = "object";
  collision_objects[0].header.frame_id = "world";

  // Define primitives, dimensions and position
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[0].primitives[0].dimensions.resize(2);
  collision_objects[0].primitives[0].dimensions[0] = 0.01;	//0.15;	// height
  collision_objects[0].primitives[0].dimensions[1] = 0.01;	// radius
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = x_pos;
  collision_objects[0].primitive_poses[0].position.y = y_pos;
  collision_objects[0].primitive_poses[0].position.z = z_pos; //0.09;
  // ************************************************************************************************

  collision_objects[0].operation = collision_objects[0].ADD;
  this->planning_scene_ptr->applyCollisionObjects(collision_objects);

  this->collision_object = collision_objects[0];
}
