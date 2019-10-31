// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// concatenate_cloud.cpp
//
// perception_class function
// ********************************************************************************************

#include "perception_class.hpp"

// CONCATENATE CLOUDS FUNCTION
// concatenates the points of all the cloud members into the combined pointcloud
// then performs downsampling (voxel_filter) and noise reduction (move_least_squares)
// to clean up resulting published pointcloud
void Perception::concatenate_clouds() 
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    
  *temp_cloud = this->left_cloud;
  *temp_cloud+= this->right_cloud;
  *temp_cloud+= this->top_cloud;

  // Uncomment to save concatenated pointcloud if desired
  // pcl::io::savePCDFileASCII("single_workstation_object_sample.pcd", *temp_cloud);

  // Passthrough filter to limit to work area
  PassThrough<PointXYZRGB> pass_w;
  pass_w.setInputCloud (temp_cloud);
  pass_w.setFilterFieldName ("x");
  pass_w.setFilterLimits (-0.9, -0.3);
  pass_w.filter(*temp_cloud);

  PassThrough<PointXYZRGB> pass_y;
  pass_y.setInputCloud (temp_cloud);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.45, 0.45);
  pass_y.filter(*temp_cloud);
  
  PassThrough<PointXYZRGB> pass_z;
  pass_z.setInputCloud (temp_cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.0, 0.3);
  pass_z.filter(*temp_cloud);

  //*****************************************************************

  PointCloud<Normal>::Ptr temp_normals(new PointCloud<Normal>);
  this->computeNormals(temp_cloud, temp_normals);

  PointIndices::Ptr inliers_plane(new PointIndices);
  *temp_cloud = this->sac_segmentation(temp_cloud, inliers_plane);
  this->combined_cloud = *temp_cloud;

  this->extractNormals(temp_normals, inliers_plane);
  
  ModelCoefficients::Ptr coefficients_cylinder(new ModelCoefficients);
  this->extractCylinder(temp_cloud, coefficients_cylinder, temp_normals);

  //*****************************************************************

  if (temp_cloud->points.empty())
  {
    ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylindrical component.");
    return;
  }

  /* Store the radius of the cylinder. */
  this->radius = coefficients_cylinder->values[6];
  /* Store direction vector of z-axis of cylinder. */
  this->direction_vec[0] = coefficients_cylinder->values[3];
  this->direction_vec[1] = coefficients_cylinder->values[4];
  this->direction_vec[2] = coefficients_cylinder->values[5];
  
  // Extracting Location and Height
  // Compute the center point of the cylinder using standard geometry
  extractLocationHeight(temp_cloud);
  addCylinder();

}

void Perception::extractCylinder(PointCloud<PointXYZRGB>::Ptr cloud, ModelCoefficients::Ptr coefficients_cylinder, PointCloud<Normal>::Ptr cloud_normals)
{
  SACSegmentationFromNormals<PointXYZRGB,Normal> segmentor;
  PointIndices::Ptr inliers_cylinder(new PointIndices);
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_CYLINDER);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  segmentor.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  segmentor.setMaxIterations(1000);
  // tolerance for variation from model
  segmentor.setDistanceThreshold(0.05);
  // min max values of radius in meters to consider
  segmentor.setRadiusLimits(0, 1);
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

  // Extract the cylinder inliers from the input cloud
  ExtractIndices<PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  extract.filter(*cloud);
}

void Perception::extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  double max_angle_y = 0.0;
  double min_angle_y = std::numeric_limits<double>::infinity();

  double lowest_point[3];
  double highest_point[3];
    // BEGIN_SUB_TUTORIAL extract_location_height
    // Consider a point inside the point cloud and imagine that point is formed on a XY plane where the perpendicular
    // distance from the plane to the camera is Z. |br|
    // The perpendicular drawn from the camera to the plane hits at center of the XY plane. |br|
    // We have the x and y coordinate of the point which is formed on the XY plane. |br|
    // X is the horizontal axis and Y is the vertical axis. |br|
    // C is the center of the plane which is Z meter away from the center of camera and A is any point on the plane.
    // |br|
    // Now we know Z is the perpendicular distance from the point to the camera. |br|
    // If you need to find the  actual distance d from the point to the camera, you should calculate the hypotenuse-
    // |code_start| hypot(point.z, point.x);\ |code_end| |br|
    // angle the point made horizontally- |code_start| atan2(point.z,point.x);\ |code_end| |br|
    // angle the point made Vertically- |code_start| atan2(point.z, point.y);\ |code_end| |br|
    // Loop over the entire pointcloud.
  for (auto const point : cloud->points)
  {
    /* Find the coordinates of the highest point */
    if (atan2(point.z, point.y) < min_angle_y)
    {
      min_angle_y = atan2(point.z, point.y);
      lowest_point[0] = point.x;
      lowest_point[1] = point.y;
      lowest_point[2] = point.z;
    }
    /* Find the coordinates of the lowest point */
    else if (atan2(point.z, point.y) > max_angle_y)
    {
      max_angle_y = atan2(point.z, point.y);
      highest_point[0] = point.x;
      highest_point[1] = point.y;
      highest_point[2] = point.z;
    }
  }
  /* Store the center point of cylinder */
  this->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
  this->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
  this->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
  /* Store the height of cylinder */
  this->height =
      sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
           pow((lowest_point[2] - highest_point[2]), 2));
}

void Perception::addCylinder()
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // BEGIN_SUB_TUTORIAL add_cylinder
  //
  // Adding Cylinder to Planning Scene
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "camera_rgb_optical_frame";
  collision_object.id = "cylinder";

  // Define a cylinder which will be added to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  /* Setting height of cylinder. */
  primitive.dimensions[0] = this->height;
  /* Setting radius of cylinder. */
  primitive.dimensions[1] = this->radius;

  // Define a pose for the cylinder (specified relative to frame_id).
  geometry_msgs::Pose cylinder_pose;
  /* Computing and setting quaternion from axis angle representation. */
  Eigen::Vector3d cylinder_z_direction(this->direction_vec[0], this->direction_vec[1],
                                         this->direction_vec[2]);
  Eigen::Vector3d origin_z_direction(0., 0., 1.);
  Eigen::Vector3d axis;
  axis = origin_z_direction.cross(cylinder_z_direction);
  axis.normalize();
  double angle = acos(cylinder_z_direction.dot(origin_z_direction));
  cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
  cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
  cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
  cylinder_pose.orientation.w = cos(angle / 2);

  // Setting the position of cylinder.
  cylinder_pose.position.x = this->center_pt[0];
  cylinder_pose.position.y = this->center_pt[1];
  cylinder_pose.position.z = this->center_pt[2];

  // Add cylinder as collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObject(collision_object);
}
