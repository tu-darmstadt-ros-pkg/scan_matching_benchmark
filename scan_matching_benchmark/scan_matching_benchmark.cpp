#include "scan_matching_benchmark.h"

#include "test_set_generator.h"

#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include <iostream>



ScanMatchingBenchmark::ScanMatchingBenchmark(ros::NodeHandle &nh)
{
  cartographer::sensor::PointCloud pointcloud;

  TestSetGenerator generator(0.02, 10.0);
  generator.generateCube(pointcloud);

  ceres_scan_matcher_options_.set_translation_weight(0.0000001);
  ceres_scan_matcher_options_.set_rotation_weight(0.0000000001);
  ceres_scan_matcher_options_.add_occupied_space_weight(10.0);
  ceres_scan_matcher_options_.mutable_ceres_solver_options()->set_max_num_iterations(100);
  ceres_scan_matcher_options_.mutable_ceres_solver_options()->set_num_threads(1);
  const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d::Translation({0,0,0.03});
  cartographer::transform::Rigid3d matched_pose_estimate;
  ceres::Solver::Summary summary;
  evaluateVoxbloxTSDFScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, summary);

  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;
}


void ScanMatchingBenchmark::evaluateVoxbloxTSDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                                const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                cartographer::transform::Rigid3d& matched_pose_estimate,
                                                ceres::Solver::Summary& summary) {
  voxblox::Transformation T_G_C;
  voxblox::Pointcloud points_C;
  voxblox::Colors colors;

  for(Eigen::Vector3f point : cloud) {
    points_C.emplace_back(voxblox::Point(point));
    colors.emplace_back(voxblox::Color::Gray());
  }

  voxblox::TsdfMap::Config tsdf_config;
  tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(0.05);
  std::shared_ptr<voxblox::TsdfMap> tsdf;
  tsdf.reset(new voxblox::TsdfMap(tsdf_config));

  voxblox::TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  integrator_config.default_truncation_distance = 0.2;
  integrator_config.max_ray_length_m = 30.0;

  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator;
  tsdf_integrator.reset(new voxblox::SimpleTsdfIntegrator(
                          integrator_config, tsdf->getTsdfLayerPtr())); //todo(kdaun) add config to choose between integrators

  tsdf_integrator->integratePointCloud(T_G_C, points_C, colors);


  float max_truncation_distance = 0.2;
  int coarsening_factor = 1;

  const cartographer::transform::Rigid3d previous_pose;
  cartographer::mapping_3d::scan_matching::CeresVoxbloxTSDFScanMatcher voxblox_scan_matcher(ceres_scan_matcher_options_);
  voxblox_scan_matcher.Match(previous_pose,
                             initial_pose_estimate,
                             {{&cloud, tsdf}},
                             max_truncation_distance,
                             coarsening_factor,
                             &matched_pose_estimate,
                             &summary);
}
