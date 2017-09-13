#include "scan_matching_benchmark.h"

#include "test_set_generator.h"
#include "scan_matcher.h"
#include "chisel_tsdf_scan_matcher.h"
#include "probability_grid_scan_matcher.h"
#include "voxblox_esdf_scan_matcher.h"
#include "voxblox_tsdf_scan_matcher.h"

#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_esdf_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/range_data_inserter.h>
#include <cartographer/mapping_3d/proto/range_data_inserter_options.pb.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_voxblox_tsdf.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_tsdf.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_voxblox_esdf.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxblox/interpolator/interpolator.h>

#include <iostream>

ScanMatchingBenchmark::ScanMatchingBenchmark(ros::NodeHandle &nh)
{
  cartographer::sensor::PointCloud pointcloud;

  TestSetGenerator generator(0.021);
  generator.generateCuboid(pointcloud, 2.025, 5.025, 2.025);

  const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d::Translation({0.05,0.1,0.2});
  cartographer::transform::Rigid3d matched_pose_estimate;
  ceres::Solver::Summary summary;
  ros::Publisher benchmark_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("benchmark_pointcloud", 1, true);

  ros::spinOnce();
  ScanMatcherConfig scan_matcher_config;
  ProbabilityGridScanMatcher probability_grid_scan_matcher(nh, scan_matcher_config);
  probability_grid_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, summary);
  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;

  ChiselTSDFScanMatcher chisel_tsdf_scan_matcher(nh, scan_matcher_config);
  chisel_tsdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, summary);
  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;

  VoxbloxTSDFScanMatcher voxblox_tsdf_scan_matcher(nh, scan_matcher_config);
  voxblox_tsdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, summary);
  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;

  VoxbloxESDFScanMatcher voxblox_esdf_scan_matcher(nh, scan_matcher_config);
  voxblox_esdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, summary);
  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;


  sensor_msgs::PointCloud2 benchmark_pointcloud_msg;
  pcl::PointCloud<pcl::PointXYZ> benchmark_pointcloud_pcl;
  for(Eigen::Vector3f& p : pointcloud)
    benchmark_pointcloud_pcl.points.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
  pcl::toROSMsg(benchmark_pointcloud_pcl, benchmark_pointcloud_msg);
  benchmark_pointcloud_msg.header.stamp = ros::Time::now();
  benchmark_pointcloud_msg.header.frame_id = "world";
  benchmark_pointcloud_publisher.publish(benchmark_pointcloud_msg);

  ros::spin();
}
