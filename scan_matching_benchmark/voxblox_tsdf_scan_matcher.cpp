#include "voxblox_tsdf_scan_matcher.h"

#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_voxblox_tsdf.h>
#include <cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h>
#include <cartographer/mapping_3d/range_data_inserter.h>

#include <open_chisel/ChunkManager.h>
#include <open_chisel/truncation/ConstantTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include <open_chisel/Chisel.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxblox/interpolator/interpolator.h>

#include <sensor_msgs/PointCloud2.h>



VoxbloxTSDFScanMatcher::VoxbloxTSDFScanMatcher(ros::NodeHandle &nh, ScanMatcherConfig config)
{
  config_ = config;

  if(config.publish_cloud) {
    map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("voxblox_tsdf_pointcloud", 1, true);
    interpolated_map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("voxblox_tsdf_interpolated_pointcloud", 1, true);
    ros::spinOnce();
  }

}
void VoxbloxTSDFScanMatcher::evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                                                const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                cartographer::transform::Rigid3d& matched_pose_estimate,
                                                 double &time_map_update, //seconds
                                                 double &time_scan_matching, //seconds
                                                ceres::Solver::Summary& summary) {
  std::cout << "VOXBLOX_TSDF\n";
  voxblox::Transformation T_G_C;
  voxblox::Pointcloud points_C;
  voxblox::Colors colors;

  for(Eigen::Vector3f point : scan_cloud) {
    points_C.emplace_back(voxblox::Point(point));
    colors.emplace_back(voxblox::Color::Gray());
  }

  voxblox::TsdfMap::Config tsdf_config;
  tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(config_.resolution);
  std::shared_ptr<voxblox::TsdfMap> voxblox_tsdf;
  voxblox_tsdf.reset(new voxblox::TsdfMap(tsdf_config));

  voxblox::TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  integrator_config.default_truncation_distance = config_.truncation_distance;
  integrator_config.max_ray_length_m = 120.0;

  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator;
  tsdf_integrator.reset(new voxblox::SimpleTsdfIntegrator(
                          integrator_config, voxblox_tsdf->getTsdfLayerPtr())); //todo(kdaun) add config to choose between integrators

  std::clock_t start_map_update = std::clock();
  tsdf_integrator->integratePointCloud(T_G_C, points_C, colors);
  time_map_update = (std::clock() - start_map_update) / (double)CLOCKS_PER_SEC;

  float max_truncation_distance = integrator_config.default_truncation_distance;
  int coarsening_factor = 1;

  const cartographer::transform::Rigid3d previous_pose;

  cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions ceres_scan_matcher_options;
  ceres_scan_matcher_options.set_translation_weight(0.0);
  ceres_scan_matcher_options.set_rotation_weight(0.0);
  ceres_scan_matcher_options.add_occupied_space_weight(10.0);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_max_num_iterations(300);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_num_threads(1);

  cartographer::mapping_3d::scan_matching::CeresVoxbloxTSDFScanMatcher voxblox_scan_matcher(ceres_scan_matcher_options);
  std::clock_t start_scan_matching = std::clock();
  voxblox_scan_matcher.Match(initial_pose_estimate,
                             initial_pose_estimate,
  {{&scan_cloud, voxblox_tsdf}},
                             max_truncation_distance,
                             coarsening_factor,
                             &matched_pose_estimate,
                             &summary);
  time_scan_matching = (std::clock() - start_scan_matching) / (double)CLOCKS_PER_SEC;



  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_unfiltered(new pcl::PointCloud<pcl::PointXYZI>);
  createDistancePointcloudFromTsdfLayer(voxblox_tsdf->getTsdfLayer(), tsdf_unfiltered.get());
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (tsdf_unfiltered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.06);
  //pass.setFilterLimitsNegative (true);
  pass.filter (map_cloud_);
  for(auto& p : map_cloud_)
    p.intensity = std::abs(p.intensity);

  cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxTSDF interpolated_voxblox_tsdf(voxblox_tsdf, max_truncation_distance);
  float min_x = config_.interpolation_map_min_x;
  float min_y = config_.interpolation_map_min_y;
  float min_z = config_.interpolation_map_min_z;
  float max_x = config_.interpolation_map_max_x;
  float max_y = config_.interpolation_map_max_y;
  float max_z = config_.interpolation_map_max_z;
  for(double x = min_x; x <= max_x; x += 0.01) {
    for(double y = min_y; y <= max_y; y += 0.01) {
      for(double z = min_z; z <= max_z; z += 0.01) {
        double q = max_truncation_distance;
        q = interpolated_voxblox_tsdf.GetSDF((double)x,(double)y,(double)z,1);
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = std::abs(q);
        interpolated_map_cloud_.push_back(p);
      }
    }
  }

  if(config_.publish_cloud) {
    publishClouds();
  }
}
