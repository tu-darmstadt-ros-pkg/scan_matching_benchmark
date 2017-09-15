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
    if(config_.multi_res_probability_grid) {
      map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("multi_res_hybrid_grid_pointcloud", 1, true);
      interpolated_map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("multi_res_hybrid_grid_interpolated_pointcloud", 1, true);
    }
    else {
      map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("hybrid_grid_pointcloud", 1, true);
      interpolated_map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("hybrid_grid_interpolated_pointcloud", 1, true);
    }
    ros::spinOnce();
  }

}
void ProbabilityGridScanMatcher::evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                                                     const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                     cartographer::transform::Rigid3d& matched_pose_estimate,
                                                     double &time_map_update, //seconds
                                                     double &time_scan_matching, //seconds
                                                     ceres::Solver::Summary& summary) {
  if(config_.multi_res_probability_grid)
    std::cout << "MultiResProbabilityGridScanMatcher\n";
  else
    std::cout << "ProbabilityGridScanMatcher\n";

  map_cloud_.clear();
  interpolated_map_cloud_.clear();
  cartographer::mapping_3d::HybridGrid hybrid_grid_high_res(config_.resolution);
  cartographer::mapping_3d::HybridGrid hybrid_grid_low_res(config_.resolution*8.0);
  cartographer::mapping_3d::proto::RangeDataInserterOptions range_data_inserter_options;
  range_data_inserter_options.set_hit_probability(0.56);
  range_data_inserter_options.set_miss_probability(0.44);
  range_data_inserter_options.set_num_free_space_voxels(5);
  cartographer::mapping_3d::RangeDataInserter range_data_inserter(range_data_inserter_options);

  cartographer::sensor::RangeData range_data;
  range_data.returns = scan_cloud;
  range_data.origin = Eigen::Vector3f{0,0,0};


  std::clock_t start_map_update = std::clock();
  range_data_inserter.Insert(range_data, &hybrid_grid_high_res);
  if(config_.multi_res_probability_grid)
    range_data_inserter.Insert(range_data, &hybrid_grid_low_res);
  time_map_update = (std::clock() - start_map_update) / (double)CLOCKS_PER_SEC;

  const cartographer::transform::Rigid3d previous_pose;

  cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions ceres_scan_matcher_options;
  ceres_scan_matcher_options.set_translation_weight(0.0);
  ceres_scan_matcher_options.set_rotation_weight(0.0);
  ceres_scan_matcher_options.add_occupied_space_weight(4.);
  if(config_.multi_res_probability_grid)
    ceres_scan_matcher_options.add_occupied_space_weight(1.);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_max_num_iterations(300);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_num_threads(1);
  cartographer::mapping_3d::scan_matching::CeresScanMatcher scan_matcher(ceres_scan_matcher_options);
  std::vector<cartographer::mapping_3d::scan_matching::PointCloudAndHybridGridPointers> pointcloud_and_grid;
  if(config_.multi_res_probability_grid)
    pointcloud_and_grid = {{&scan_cloud, &hybrid_grid_high_res},{&scan_cloud, &hybrid_grid_low_res}};
  else
    pointcloud_and_grid = {{&scan_cloud, &hybrid_grid_high_res}};

  std::clock_t start_scan_matching = std::clock();
  scan_matcher.Match(initial_pose_estimate,
                     initial_pose_estimate,
                     pointcloud_and_grid,
                     &matched_pose_estimate,
                     &summary);
  time_scan_matching = (std::clock() - start_scan_matching) / (double)CLOCKS_PER_SEC;

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

    cartographer::mapping_3d::scan_matching::InterpolatedGrid interpolated_grid_high_res(hybrid_grid_high_res);
    cartographer::mapping_3d::scan_matching::InterpolatedGrid interpolated_grid_low_res(hybrid_grid_low_res);
    float min_x = config_.interpolation_map_min_x;
    float min_y = config_.interpolation_map_min_y;
    float min_z = config_.interpolation_map_min_z;
    float max_x = config_.interpolation_map_max_x;
    float max_y = config_.interpolation_map_max_y;
    float max_z = config_.interpolation_map_max_z;
    for(float x = min_x; x <= max_x; x += 0.01) {
      for(float y = min_y; y <= max_y; y += 0.01) {
        for(float z = min_z; z <= max_z; z += 0.01) {
          float q;
          if(config_.multi_res_probability_grid)
            q = 0.8*std::abs(interpolated_grid_high_res.GetProbability((double)x,(double)y,(double)z))
                + 0.2*std::abs(interpolated_grid_low_res.GetProbability((double)x,(double)y,(double)z));
          else
            q = std::abs(interpolated_grid_high_res.GetProbability((double)x,(double)y,(double)z));

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
