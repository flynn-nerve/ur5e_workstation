// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// perception_class.cpp
//
// Implements a Perception class to handle visualization for operating a robotic manipulator
// utilizing an Intel Realsense D435i sensor
//
// ********************************************************************************************

#include "perception_class.hpp"

// ********************************************************************************************
// Private Functions
// ********************************************************************************************

// WRIST CAMERA CALLBACK FUNCTION
// initialize cloud right with the information from wrist_camera
// converts pointcloud from sensor_msgs::PointCloud2 to pcl::PointXYZRGB
void Perception::wrist_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, this->current_cloud);
 
    ros::Time stamp = ros::Time(0);
    pcl_conversions::toPCL(stamp, this->current_cloud.header.stamp);
    this->transform_listener_ptr->waitForTransform("/world", this->current_cloud.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->current_cloud, this->current_cloud, *this->transform_listener_ptr);
}

// ********************************************************************************************
// Public Functions
// ********************************************************************************************

// INITIALIZE SUBSCRIBER FUNCTION
// Seperate "constructor" for initialization of subscriber due to passing shared pointers as arguments before creating them
void Perception::init_subscriber(ros::NodeHandle nodeHandle)
{
  this->wrist_camera_sub = nodeHandle.subscribe("/pcl_filters/wrist_camera_xyz_filter/output", 1, &Perception::wrist_camera_callback, this);
}

// PERCEPTION CLASS CONSTRUCTOR
// Create instance of Perception class and instantiate publisher for combined cloud 
// and subscriber for pointcloud from wrist camera sensor
Perception::Perception(ros::NodeHandle handle)
{
  this->combined_cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
  this->points_not_found = true;
}

// PUBLISH COMBINED CLOUD FUNCTION
// Publish combined (concatenated) point cloud
// Convert combined_cloud (pcl::PointXYZRGB) to sensor_msgs::PointCloud2
// and then publish
void Perception::publish_combined_cloud()
{

  sensor_msgs::PointCloud2 cloud;
  toROSMsg(this->combined_cloud, cloud);

  this->combined_cloud_pub.publish(cloud);
}
