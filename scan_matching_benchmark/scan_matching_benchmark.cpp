#include "include/scan_matching_benchmark/scan_matching_benchmark.h"

#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>

ScanMatchingBenchmark::ScanMatchingBenchmark(ros::NodeHandle &nh)
{
  const cartographer::sensor::PointCloud pointcloud;
  const std::shared_ptr<voxblox::TsdfMap> tsdf;

  voxblox::TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  integrator_config.default_truncation_distance = 0.2;
  integrator_config.max_ray_length_m = 30.0;

  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator;
  tsdf_integrator.reset(new voxblox::SimpleTsdfIntegrator(
      integrator_config, tsdf->getTsdfLayerPtr())); //todo(kdaun) add config to choose between integrators

   voxblox::Transformation T_G_C;
   voxblox::Pointcloud points_C;
   voxblox::Colors colors;

   float generator_resolution = 0.1;
   int generator_num_points = 1000;
   //square
   int face = 0;
   voxblox::Point(0,0,0);
   for(int i = 0; i < generator_num_points; ++i) {
     //todo generate data

   }

   tsdf_integrator->integratePointCloud(T_G_C, points_C, colors);

  cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions options;
  cartographer::mapping_3d::scan_matching::CeresVoxbloxTSDFScanMatcher voxblox_scan_matcher(options);
  const cartographer::transform::Rigid3d previous_pose;
  const cartographer::transform::Rigid3d initial_pose_estimate;
  const std::vector<cartographer::mapping_3d::scan_matching::PointCloudAndVoxbloxTSDFPointers>
      point_clouds_and_tsdfs;
  float max_truncation_distance = 0.5;
  int coarsening_factor = 1;
  cartographer::transform::Rigid3d pose_estimate;
  ceres::Solver::Summary summary;
  voxblox_scan_matcher.Match(previous_pose,
                             initial_pose_estimate,
                             point_clouds_and_tsdfs,
                             max_truncation_distance,
                             coarsening_factor,
                             &pose_estimate,
                             &summary);

}
