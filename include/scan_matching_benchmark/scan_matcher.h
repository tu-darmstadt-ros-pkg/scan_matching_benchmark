#ifndef SCAN_MATCHER_H_
#define SCAN_MATCHER_H_

#include <cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h>
#include <cartographer/sensor/point_cloud.h>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <voxblox/core/tsdf_map.h>


struct ScanMatcherConfig
{
public:
  bool publish_cloud = true;
  bool verbose = true;
  float resolution = 0.05;
  float truncation_distance = 0.4;
  float esdf_distance = 1.0;
  float interpolation_map_min_x = -5.0;
  float interpolation_map_min_y = -5.0;
  float interpolation_map_min_z = 0.0;
  float interpolation_map_max_x = 5.0;
  float interpolation_map_max_y = 5.0;
  float interpolation_map_max_z = 0.01;
};

class ScanMatcher
{
public:
  //virtual ScanMatcher(ros::NodeHandle &nh, ScanMatcherConfig config) = 0;
  virtual void evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                                   const cartographer::transform::Rigid3d& initial_pose_estimate,
                                   cartographer::transform::Rigid3d& matched_pose_estimate,
                                   ceres::Solver::Summary& summary) = 0;
  void publishClouds();

protected:
  ros::Publisher map_pointcloud_publisher_;
  ros::Publisher interpolated_map_pointcloud_publisher_;
  pcl::PointCloud<pcl::PointXYZI> map_cloud_;
  pcl::PointCloud<pcl::PointXYZI> interpolated_map_cloud_;
  ScanMatcherConfig config_;
};


#endif //SCAN_MATCHER_H_
