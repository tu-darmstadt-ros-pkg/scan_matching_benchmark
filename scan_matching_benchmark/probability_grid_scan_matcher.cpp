#include "probability_grid_scan_matcher.h"

#include <cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_grid.h>
#include <cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h>
#include <cartographer/mapping_3d/range_data_inserter.h>

#include <sensor_msgs/PointCloud2.h>



ProbabilityGridScanMatcher::ProbabilityGridScanMatcher(ros::NodeHandle &nh, ScanMatcherConfig config)
{
  config_ = config;

  if(config.publish_cloud) {
    map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("hybrid_grid_pointcloud", 1, true);
    interpolated_map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("hybrid_grid_interpolated_pointcloud", 1, true);
    ros::spinOnce();
  }

}
void ProbabilityGridScanMatcher::evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                                                     const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                     cartographer::transform::Rigid3d& matched_pose_estimate,
                                                     ceres::Solver::Summary& summary) {
  std::cout << "ProbabilityGridScanMatcher\n";
  map_cloud_.clear();
  interpolated_map_cloud_.clear();
  cartographer::mapping_3d::HybridGrid hybrid_grid_high_res(config_.resolution);
  cartographer::mapping_3d::proto::RangeDataInserterOptions range_data_inserter_options;
  range_data_inserter_options.set_hit_probability(0.56);
  range_data_inserter_options.set_miss_probability(0.44);
  range_data_inserter_options.set_num_free_space_voxels(5);
  cartographer::mapping_3d::RangeDataInserter range_data_inserter(range_data_inserter_options);

  cartographer::sensor::RangeData range_data;
  range_data.returns = scan_cloud;
  range_data.origin = Eigen::Vector3f{0,0,0};

  range_data_inserter.Insert(range_data, &hybrid_grid_high_res);

  const cartographer::transform::Rigid3d previous_pose;

  cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions ceres_scan_matcher_options;
  ceres_scan_matcher_options.set_translation_weight(0.0000001);
  ceres_scan_matcher_options.set_rotation_weight(0.0000000001);
  ceres_scan_matcher_options.add_occupied_space_weight(10.0);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_max_num_iterations(300);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_num_threads(1);
  cartographer::mapping_3d::scan_matching::CeresScanMatcher scan_matcher(ceres_scan_matcher_options);
  scan_matcher.Match(previous_pose,
                     initial_pose_estimate,
  {{&scan_cloud, &hybrid_grid_high_res}},
                     &matched_pose_estimate,
                     &summary);

  if(config_.publish_cloud){
  for(auto& p : hybrid_grid_high_res)
  {
    float probability =  hybrid_grid_high_res.GetProbability(p.first);
    Eigen::Vector3f cell_center = hybrid_grid_high_res.GetCenterOfCell(p.first);
    pcl::PointXYZI point;
    point.x = cell_center[0];
    point.y = cell_center[1];
    point.z = cell_center[2];
    point.intensity = probability;
    map_cloud_.push_back(point);
  }

  cartographer::mapping_3d::scan_matching::InterpolatedGrid interpolated_grid(hybrid_grid_high_res);
  float min_x = config_.interpolation_map_min_x;
  float min_y = config_.interpolation_map_min_y;
  float min_z = config_.interpolation_map_min_z;
  float max_x = config_.interpolation_map_max_x;
  float max_y = config_.interpolation_map_max_y;
  float max_z = config_.interpolation_map_max_z;
  for(float x = min_x; x <= max_x; x += 0.01) {
    for(float y = min_y; y <= max_y; y += 0.01) {
      for(float z = min_z; z <= max_z; z += 0.01) {
        float q = std::abs(interpolated_grid.GetProbability((double)x,(double)y,(double)z));
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = q;
        interpolated_map_cloud_.push_back(p);
      }
    }
  }

    publishClouds();
  }

}
