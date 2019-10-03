// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// pcl_filters.cpp
//
// Implements numerous pcl filtering functions as members of the custom Perception class
// Non-void outputs for passing of variables between classes/objects as necessary such
// as when generating collision objects from derived point coordinates
//
// Utilizes code originally created by Victoria Albanese from UMASS Lowell while working 
// at the NERVE Center on the Verizon 5G Challenge Project (Particularly voxel_filter, 
// concatenate_clouds, and move_least_squares functions <- thank you Victoria!)
//
// ********************************************************************************************

#include "perception_class.hpp"

// COMPUTE NORMALS FUNCTION
// TODO: add description
void Perception::computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals)
{
  search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

// EXTRACT NORMALS FUNCTION
// TODO: add description
void Perception::extractNormals(PointCloud<Normal>::Ptr cloud_normals, PointIndices::Ptr inliers_plane)
{
  ExtractIndices<Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

// VOXEL FILTER FUNCTION
// downsamples point cloud to make the resulting model cleaner
PointCloud<PointXYZRGB> Perception::voxelgrid_filter(PointCloud<PointXYZRGB>::Ptr cloud)
{
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);

  VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01, 0.01, 0.01); 
  sor.filter(*filtered_cloud);

  return *filtered_cloud;
}

// SAC (PLANAR) SEGMENTATION FUNCTION
// remove largest planar surface from pointcloud
PointCloud<PointXYZRGB> Perception::sac_segmentation(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr inliers_plane)
{
  // Create the segmentation object
  SACSegmentation<PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud);
  ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
  seg.segment (*inliers_plane, *coefficients_plane);

  // Create the filtering object
  ExtractIndices<PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);

  return *cloud;
}

// MOVE LEAST SQUARES FUNCTION
// aligns the surface normals to eliminate noise
PointCloud<PointNormal> Perception::move_least_squares(PointCloud<PointXYZRGB>::Ptr cloud)
{
  search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
  PointCloud<PointNormal> mls_points;
  MovingLeastSquares<PointXYZRGB, PointNormal> mls;
 
  mls.setComputeNormals(true);
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(4);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.1);
  mls.process(mls_points);

  return mls_points;
}

