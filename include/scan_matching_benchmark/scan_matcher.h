#ifndef SCAN_MATCHER_H_
#define SCAN_MATCHER_H_

#include <cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h>
#include <cartographer/sensor/point_cloud.h>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <cstdio>
#include <ctime>
#include <chrono>


struct ScanMatcherConfig
{
public:
  bool publish_cloud = true;
  bool verbose = true;
  bool multi_res_probability_grid = false;
  float resolution = 0.1;
  float truncation_distance = 0.4;
  float esdf_distance = 30.0;
  float interpolation_map_min_x = -10.1;
  float interpolation_map_min_y = -10.1;
  float interpolation_map_min_z = 0.0;
  float interpolation_map_max_x = 10.1;
  float interpolation_map_max_y = 10.1;
  float interpolation_map_max_z = 0.0;
  bool boundary_extrapolation = false;
  bool cubic_interpolation = false;
};

class ScanMatcher
{
public:
  //virtual ScanMatcher(ros::NodeHandle &nh, ScanMatcherConfig config) = 0;
  virtual void evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                                   const cartographer::transform::Rigid3d& initial_pose_estimate,
                                   cartographer::transform::Rigid3d& matched_pose_estimate,
                                   double& time_map_update, //seconds
                                   double& time_scan_matching, //seconds
                                   ceres::Solver::Summary& summary) = 0;
  void publishClouds();

protected:
  ros::Publisher map_pointcloud_publisher_;
  ros::Publisher interpolated_map_pointcloud_publisher_;
  ros::Publisher gradient_x_publisher_;
  ros::Publisher gradient_y_publisher_;
  pcl::PointCloud<pcl::PointXYZI> map_cloud_;
  pcl::PointCloud<pcl::PointXYZI> interpolated_map_cloud_;
  pcl::PointCloud<pcl::PointXYZI> gradient_x_;
  pcl::PointCloud<pcl::PointXYZI> gradient_y_;
  ScanMatcherConfig config_;
};


#endif //SCAN_MATCHER_H_
