#ifndef SCAN_MATCHING_BENCHMARK_H_
#define SCAN_MATCHING_BENCHMARK_H_

#include <cartographer/sensor/point_cloud.h>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <voxblox/core/tsdf_map.h>



class ScanMatchingBenchmark
{
public:
  ScanMatchingBenchmark(ros::NodeHandle &nh);

private:

  void evaluateChiselTSDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                      const cartographer::transform::Rigid3d& initial_pose_estimate,
                                      cartographer::transform::Rigid3d& matched_pose_estimate,
                                      pcl::PointCloud<pcl::PointXYZI>& chisel_tsdf_cloud,
                                      pcl::PointCloud<pcl::PointXYZI>& interpolated_chisel_tsdf_cloud,
                                      ceres::Solver::Summary& summary);

  void evaluateVoxbloxTSDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                      const cartographer::transform::Rigid3d& initial_pose_estimate,
                                      cartographer::transform::Rigid3d& matched_pose_estimate,
                                      pcl::PointCloud<pcl::PointXYZI>& voxblox_tsdf_cloud,
                                      pcl::PointCloud<pcl::PointXYZI>& interpolated_voxblox_tsdf_cloud,
                                      ceres::Solver::Summary& summary);

  void evaluateVoxbloxESDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                      const cartographer::transform::Rigid3d& initial_pose_estimate,
                                      cartographer::transform::Rigid3d& matched_pose_estimate,
                                      pcl::PointCloud<pcl::PointXYZI>& voxblox_esdf_cloud,
                                      pcl::PointCloud<pcl::PointXYZI>& interpolated_voxblox_esdf_cloud,
                                      ceres::Solver::Summary& summary);

  void evaluateProbabilityGridScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                          const cartographer::transform::Rigid3d& initial_pose_estimate,
                                          cartographer::transform::Rigid3d& matched_pose_estimate,
                                          pcl::PointCloud<pcl::PointXYZI>& probability_grid,
                                          pcl::PointCloud<pcl::PointXYZI>& interpolated_probability_grid_cloud,
                                          ceres::Solver::Summary& summary);



};

#endif //SCAN_MATCHING_BENCHMARK_H_
