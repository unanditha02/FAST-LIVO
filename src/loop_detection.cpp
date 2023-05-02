#include "loop_detection.hpp"

LoopDetection::LoopDetection()
{
  point_cloud_in_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
  keyframe_counter_ = NUM_FRAMES_PER_KEYFRAME + 1;
  timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &LoopDetection::timerCallback, this);
  // /cloud_registered
  cloud_in_sub_ = n_.subscribe("/cloud_registered", 200000, &LoopDetection::cloud_cb, this);
  path_in_sub_ = n_.subscribe("/path",200000, &LoopDetection::path_cb, this);
  client = n_.serviceClient<fast_livo::global_registration>("global_registration");
  pose_graph_constraint_pub_ = n_.advertise<std_msgs::String>("pose_graph_constraint", 1000);
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

void LoopDetection::path_cb(const nav_msgs::Path &input)
{
  path_ = input;
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
  bool loop_detected = false;
  double best_similarity_score = 0;
  double best_similarity_time = 0;
  int best_old_pose_idx;
  int best_new_pose_idx;
  geometry_msgs::Pose best_old_pose;
  geometry_msgs::Pose best_new_pose;
  for (int i = 0; i < old_keyframes.size(); ++i)
  {
    double similarity = compareFPFHs(summed_fpfh, old_keyframes[i]);
    ROS_INFO("Similarity at t=%f: %f", old_keyframe_times[i], similarity);
    // If similarity above a certain threshold
    if (similarity > 0.985)
    {
      if(fabs(old_keyframe_times[i] - t_) < 10.0) continue; // THANKS SASSY
      // ROS_INFO("Similarity above threshold");
      geometry_msgs::Pose old_pose;
      geometry_msgs::Pose new_pose;
      int old_pose_idx;
      int new_pose_idx;
      // Determine pose at old timestep
      bool new_pose_flag = false;
      for (int j = 0; j < path_.poses.size(); ++j)
      {
        // ROS_INFO("%f, %f", path_.poses[j].header.stamp.toSec(), old_keyframe_times[i]);
        // ROS_INFO("%f, %f", round(path_.poses[j].header.stamp.toSec()), round(old_keyframe_times[i]));
        if (round(path_.poses[j].header.stamp.toSec()) == round(old_keyframe_times[i]))
        {
          old_pose = path_.poses[j].pose;
          old_pose_idx = j;
          // ROS_INFO("Found old pose");
          
        }
        if (round(path_.poses[j].header.stamp.toSec()) == round(t_))
        {
          new_pose = path_.poses[j].pose;
          new_pose_idx = j;
          new_pose_flag = true;
          // ROS_INFO("Found new pose");s
        }
      }
      if(!new_pose_flag){
        continue;
      }
      double distancethreshold = 0;
      double distance = 0;
      for (int j = old_pose_idx+1; j < new_pose_idx+1; ++j)
      {
        distancethreshold += sqrt(pow(path_.poses[j].pose.position.x - path_.poses[j-1].pose.position.x, 2) + 
                          pow(path_.poses[j].pose.position.y - path_.poses[j-1].pose.position.y, 2) + 
                          pow(path_.poses[j].pose.position.z - path_.poses[j-1].pose.position.z, 2)); // abs(Old pose - new pose)
      }
      // ROS_INFO("WEE WOO %f", distance);
      // Calculate distance
      distance = sqrt(pow(old_pose.position.x - new_pose.position.x, 2) + 
                          pow(old_pose.position.y - new_pose.position.y, 2) + 
                          pow(old_pose.position.z - new_pose.position.z, 2)); // abs(Old pose - new pose)
      // If drift is within a certain distance threshold based on difference in time
      if (distance < distancethreshold*0.25)
      {
      //   // Print drift over time in coordinate frame values 
        ROS_INFO("Distance Threshold: %f", distancethreshold);
        ROS_INFO("Loop Detected at t = %f, Euclidean drift calculated: %f", old_keyframe_times[i], distance);
        loop_detected = true;
        if (similarity > best_similarity_score)
        {
          best_similarity_score = similarity;
          best_similarity_time = old_keyframe_times[i];
          best_old_pose_idx = old_pose_idx;
          best_old_pose = path_.poses[best_old_pose_idx].pose;
          best_new_pose_idx = new_pose_idx;
          best_new_pose = path_.poses[best_new_pose_idx].pose;
        }
      //   //Return over a rostopic the drift and old pose graph time
      //   break;
      }
    }
  }

  if (loop_detected)
  {
    fast_livo::global_registration srv;
    pcl::toROSMsg(keyframe_pointclouds_[best_similarity_time], srv.request.pcl1);
    pcl::toROSMsg(*point_cloud_in_, srv.request.pcl2);
    if (client.call(srv))
    {
      ROS_INFO("CALLED SUCCESSFULLY!");
      // srv.response.transform
      tf::Transform world_transform;
      world_transform.setOrigin(tf::Vector3(srv.response.transform.translation.x, srv.response.transform.translation.y, srv.response.transform.translation.z));
      world_transform.setRotation(tf::Quaternion(srv.response.transform.rotation.x, srv.response.transform.rotation.y, srv.response.transform.rotation.z, srv.response.transform.rotation.w));
      tf::Transform prev_pose;
      prev_pose.setOrigin(tf::Vector3(best_old_pose.position.x, best_old_pose.position.y, best_old_pose.position.z));
      prev_pose.setRotation(tf::Quaternion(best_old_pose.orientation.x, best_old_pose.orientation.y, best_old_pose.orientation.z, best_old_pose.orientation.w));
      tf::Transform current_pose;
      current_pose.setOrigin(tf::Vector3(best_new_pose.position.x, best_new_pose.position.y, best_new_pose.position.z));
      current_pose.setRotation(tf::Quaternion(best_new_pose.orientation.x, best_new_pose.orientation.y, best_new_pose.orientation.z, best_new_pose.orientation.w));
      tf::Transform relative_transform = prev_pose.inverseTimes(world_transform.inverseTimes(current_pose));
      std_msgs::String msg;
      msg.data = "EDGE_SE3:QUAT " + std::to_string(best_old_pose_idx) + " " + std::to_string(best_new_pose_idx)
                                                + " " + std::to_string(relative_transform.getOrigin().getX())
                                                + " " + std::to_string(relative_transform.getOrigin().getY())
                                                + " " + std::to_string(relative_transform.getOrigin().getZ())
                                                + " " + std::to_string(relative_transform.getRotation().getX())
                                                + " " + std::to_string(relative_transform.getRotation().getY())
                                                + " " + std::to_string(relative_transform.getRotation().getZ())
                                                + " " + std::to_string(relative_transform.getRotation().getW())
                                                + " " + std::to_string(1)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(1)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(1)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(1)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(1)
                                                + " " + std::to_string(0)
                                                + " " + std::to_string(1);
      ROS_INFO("Sending constraint!");
      ROS_INFO("%s", msg.data.c_str());
      pose_graph_constraint_pub_.publish(msg);
    }
    else
    {
      ROS_ERROR("CALLED UNSUCCESSFULLY!");
    }
  }
  
  // keyframe_fpfhs_.insert({t_, fpfhs});
  keyframe_pointclouds_.insert({t_, pcl::PointCloud<pcl::PointXYZ>(*point_cloud_in_)});
  summed_keyframe_fpfhs_.insert({t_, summed_fpfh});
}

double LoopDetection::round(double in)
{
  return static_cast<long int>(in * 100.0) / 100.0;
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