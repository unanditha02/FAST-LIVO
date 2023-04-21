#include "loop_detection.hpp"

LoopDetection::LoopDetection()
{
  point_cloud_in_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
  keyframe_counter_ = NUM_FRAMES_PER_KEYFRAME + 1;
  timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &LoopDetection::timerCallback, this);
  // /cloud_registered
  cloud_in_sub_ = n_.subscribe("/cloud_registered", 200000, &LoopDetection::cloud_cb, this);
}

void LoopDetection::timerCallback(const ros::TimerEvent &)
{
  // ROS_INFO("In timer!");
}

void LoopDetection::cloud_cb(const sensor_msgs::PointCloud2 &input)
{
  if (keyframe_counter_ > NUM_FRAMES_PER_KEYFRAME)
  {
    pcl::fromROSMsg(input, *point_cloud_in_);
    keyframe_counter_ = keyframe_counter_ % NUM_FRAMES_PER_KEYFRAME;
    t_ = input.header.stamp.toSec();
    calculateNormals();
    calculateFPFH(); // Calculates FPFH for every point, normalizes, then sums to represent keyframe, normalizes
    //Compare against previous keyframe FPFHs
    ROS_INFO("Number of points: %ld", point_cloud_in_->points.size());
    // ROS_INFO("Point cloud point: %f, %f, %f", point_cloud_in_->points[0].x, point_cloud_in_->points[0].y, point_cloud_in_->points[0].z);
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

  auto norms_it = normals_->begin();
  auto pc_it = point_cloud_in_->begin();
  for (; norms_it != normals_->end(); ++norms_it, ++pc_it)
  {
    if (!pcl::isFinite<pcl::Normal>(*norms_it))
    {
      normals_->erase(norms_it--);
      point_cloud_in_->erase(pc_it--);
      // ROS_INFO("Erased point");
    }
  }
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

  for (int i = 0; i < normals_->size(); i++)
  {
    if (!pcl::isFinite<pcl::Normal>((*normals_)[i]))
    {
      PCL_WARN("normals[%d] is not finite\n", i);
    }
  }

  // Compute the features
  fpfh_.compute(*fpfhs);
  // ROS_INFO("Printing histogram");

  // Normalizing the FPFHs of all points
  for (auto p_it = fpfhs->begin(); p_it != fpfhs->end(); ++p_it)
  {
    float sum = 0;
    for (float f : p_it->histogram)
    {
      sum += f;
    }
    if (!isfinite(sum) || sum == 0)
    {
      // ROS_INFO("Sum is not finite!!!");
      fpfhs->erase(p_it--);
      continue;
    }
    for (int i = 0; i < p_it->descriptorSize(); ++i)
    {
      p_it->histogram[i] =  p_it->histogram[i] / sum;
    }
  }
  
  // Declaring summed FPFH to store FPFH of entire keyframe
  pcl::FPFHSignature33 summed_fpfh;
  for (int i = 0; i < summed_fpfh.descriptorSize(); ++i)
  {
    summed_fpfh.histogram[i] = 0;
  }

  // Summing FPFHs for Keyframe
  for (int j = 0; j < fpfhs->points.size(); ++j)
  {
    for (int i = 0; i < fpfhs->points[j].descriptorSize(); ++i)
    {
      summed_fpfh.histogram[i] += fpfhs->points[j].histogram[i];
    }
  }

  // Normalizing Keyframe FPFH
  float sum = 0;
  for (int i = 0; i < summed_fpfh.descriptorSize(); ++i)
  {
    // ROS_INFO("hist value: %f", summed_fpfh.histogram[i]);
    sum += summed_fpfh.histogram[i];
  }
  // ROS_INFO("Sum: %f", sum);
  for (int i = 0; i < summed_fpfh.descriptorSize(); ++i)
  {
    summed_fpfh.histogram[i] = summed_fpfh.histogram[i] / sum;
  }

  
  std::vector<double> old_keyframe_times;
  std::vector<pcl::FPFHSignature33> old_keyframes;
  for (auto key_fpfh : summed_keyframe_fpfhs_)
  {
    old_keyframe_times.push_back(key_fpfh.first);
    old_keyframes.push_back(key_fpfh.second);
  }

  // #ifdef MP_EN
  //   ROS_INFO("Using OMP");
  //   omp_set_num_threads(MP_PROC_NUM);
  //   #pragma omp parallel for
  // #endif
  for (int i = 0; i < old_keyframes.size(); ++i)
  {
    double similarity = compareFPFHs(summed_fpfh, old_keyframes[i]);
    ROS_INFO("Similarity at t=%f: %f", old_keyframe_times[i], similarity);
  }
  // keyframe_fpfhs_.insert({t_, fpfhs});
  summed_keyframe_fpfhs_.insert({t_, summed_fpfh});
}

double LoopDetection::compareFPFHs(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2)
{
  double mean = 1.0 / fpfh1.descriptorSize();
  double sum1 = 0;
  double sum2 = 0;
  for (int i = 0; i < fpfh1.descriptorSize(); ++i)
  {
    sum1 += powf(fpfh1.histogram[i] - mean, 2);
    sum2 += powf(fpfh2.histogram[i] - mean, 2);
    // ROS_INFO("fpfh1: %f", fpfh1.histogram[i]);
    // ROS_INFO("fpfh2: %f", fpfh2.histogram[i]);
  }
  double denominator = sqrt(sum1 * sum2);
  // ROS_INFO("Denom: %f", denominator);
  // ROS_INFO("sum1: %f", sum1);
  // ROS_INFO("sum2: %f", sum2);
  // ROS_INFO("mean: %f", mean);
  double numerator = 0;
  for (int i = 0; i < fpfh1.descriptorSize(); ++i)
  {
    numerator += (fpfh1.histogram[i] - mean) * (fpfh2.histogram[i] - mean);
  }
  return numerator/denominator;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loop_detection_node");
  ROS_INFO("Heyyy");
  LoopDetection loop_detector_;
  ros::spin();
  return 0;
}