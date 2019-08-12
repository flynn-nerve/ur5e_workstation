// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// create_object.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"

void Manipulation::create_object() {

  double xpos_rand, ypos_rand, xpos_m, ypos_m, xpos, ypos;

  srand (time(NULL));

  xpos_rand = (rand() % 30) + 1;
  ypos_rand = (rand() % 30) + 1;
  xpos_m = (xpos_rand - 12.5) / 100;
  ypos_m = (ypos_rand - 12.5) / 100;
  xpos = -0.642 + xpos_m;
  ypos = ypos_m;

  this->x_pos = xpos;
  this->y_pos = ypos;
  this->z_pos = 0.025; //0.075;
  
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
  collision_objects[0].primitives[0].dimensions[0] = 0.035;	//0.15;	// height
  collision_objects[0].primitives[0].dimensions[1] = 0.035;	// radius
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = xpos;
  collision_objects[0].primitive_poses[0].position.y = ypos;
  collision_objects[0].primitive_poses[0].position.z = 0.05; //0.09;
  // ************************************************************************************************

  collision_objects[0].operation = collision_objects[0].ADD;
  this->planning_scene_ptr->applyCollisionObjects(collision_objects);

  this->collision_object = collision_objects[0];
}
