#pragma once

#include "ros/ros.h"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <unordered_map>
#include <map>
#include <nav_msgs/Path.h>


#define PLANNER_RATE 10

class LoopDetection
{
    public:
    LoopDetection();


    private:
    // Hyper parameters
    unsigned int NUM_FRAMES_PER_KEYFRAME = 50;
    float NORMALS_DISTANCE = 0.3;
    float FPFH_RADIUS = 5.0;

    void timerCallback(const ros::TimerEvent&);
    void cloud_cb(const sensor_msgs::PointCloud2 &input);
    void path_cb(const nav_msgs::Path &input);
    void calculateNormals();
    void calculateFPFH();
    double compareFPFHs(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2);
    double round(double in);

    ros::NodeHandle n_;
    ros::Timer timer_;
    double t_;
    ros::Subscriber cloud_in_sub_;
    ros::Subscriber path_in_sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_in_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_;
    std::unordered_map<double, pcl::PointCloud<pcl::FPFHSignature33>::Ptr> keyframe_fpfhs_;
    std::map<double, pcl::FPFHSignature33> summed_keyframe_fpfhs_;
    nav_msgs::Path path_;

    unsigned int keyframe_counter_;
};

