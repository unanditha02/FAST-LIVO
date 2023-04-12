#pragma once

#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <unordered_map>


#define PLANNER_RATE 10

class LoopDetection
{
    public:
    LoopDetection();


    private:
    // Hyper parameters
    unsigned int NUM_FRAMES_PER_KEYFRAME = 10;
    float NORMALS_DISTANCE = 0.3;
    float FPFH_RADIUS = 1.0;

    void timerCallback(const ros::TimerEvent&);
    void cloud_cb(const sensor_msgs::PointCloud2 &input);
    void calculateNormals();
    void calculateFPFH();
    double compareFPFHs(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh1, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh2);

    ros::NodeHandle n_;
    ros::Timer timer_;
    double t_;
    ros::Subscriber cloud_in_sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_in_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_;
    std::unordered_map<double, pcl::PointCloud<pcl::FPFHSignature33>::Ptr> keyframe_fpfhs_;
    std::unordered_map<double, pcl::FPFHSignature33> summed_keyframe_fpfhs_;

    unsigned int keyframe_counter_;
};

