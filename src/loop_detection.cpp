#include "loop_detection.hpp"

LoopDetection::LoopDetection()
{
  point_cloud_in_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
  keyframe_counter_ = 0;
  timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &LoopDetection::timerCallback, this);
  // /cloud_registered
  cloud_in_sub_ = n_.subscribe("/cloud_registered", 200000, &LoopDetection::cloud_cb, this);
}

void LoopDetection::timerCallback(const ros::TimerEvent &)
{
  ROS_INFO("In timer!");
}

void LoopDetection::cloud_cb(const sensor_msgs::PointCloud2 &input)
{
  if (keyframe_counter_ > NUM_FRAMES_PER_KEYFRAME)
  {
    pcl::fromROSMsg(input, *point_cloud_in_);
    keyframe_counter_ = keyframe_counter_ % NUM_FRAMES_PER_KEYFRAME;
    t_ = input.header.stamp.toSec();
    calculateNormals();
    calculateFPFH();
    ROS_INFO("Point cloud point: %f, %f, %f", point_cloud_in_->points[0].x, point_cloud_in_->points[0].y, point_cloud_in_->points[0].z);
  }
  ++keyframe_counter_;
}

void LoopDetection::calculateNormals()
{
  // Create the normal estimation class, and pass the input dataset to it
  ne_.setInputCloud(point_cloud_in_);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne_.setSearchMethod(tree);

  // Use all neighbors in a sphere of radius 3cm
  ne_.setRadiusSearch(NORMALS_DISTANCE);

  // Compute the features
  ne_.compute(*normals_);
}

void LoopDetection::calculateFPFH()
{
  fpfh_.setInputCloud(point_cloud_in_);
  fpfh_.setInputNormals(normals_);
  // alternatively, if cloud is of tpe PointNormal, do fpfh_.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  fpfh_.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh_.setRadiusSearch(FPFH_RADIUS);

  // Compute the features
  fpfh_.compute(*fpfhs);

  keyframe_fpfhs_.insert({t_, fpfhs});
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loop_detection_node");
  ROS_INFO("Heyyy");
  LoopDetection loop_detector_;
  ros::spin();
  return 0;
}