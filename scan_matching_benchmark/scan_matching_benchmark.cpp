#include "scan_matching_benchmark.h"

#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>
#include <glog/logging.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>



ScanMatchingBenchmark::ScanMatchingBenchmark(ros::NodeHandle &nh)
{

//google::InitGoogleLogging("ScanMatchingBenchmark");
    const cartographer::sensor::PointCloud pointcloud;

    LOG(INFO)<<"a";
    voxblox::TsdfMap::Config tsdf_config;
    tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(0.05);
    std::shared_ptr<voxblox::TsdfMap> tsdf;
    tsdf.reset(new voxblox::TsdfMap(tsdf_config));
    LOG(INFO)<<"b";

    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.voxel_carving_enabled = true;
    integrator_config.default_truncation_distance = 0.2;
    integrator_config.max_ray_length_m = 30.0;
    LOG(INFO)<<"c";

    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator;
    tsdf_integrator.reset(new voxblox::SimpleTsdfIntegrator(
                              integrator_config, tsdf->getTsdfLayerPtr())); //todo(kdaun) add config to choose between integrators

    LOG(INFO)<<"d";
    voxblox::Transformation T_G_C;
    voxblox::Pointcloud points_C;
    voxblox::Colors colors;
    cartographer::sensor::PointCloud points_carto;

    float generator_resolution = 0.1;
    int generator_num_points = 1000;
    //square
    float num_points_per_side = generator_num_points / 4;
    float start_offset = num_points_per_side * generator_resolution;
    voxblox::Point point(-start_offset,start_offset,0);
    voxblox::Point delta(0,0,0);
    for(int i_side = 0; i_side < 4; ++i_side)
    {
        switch (i_side) {
        case 0:
            delta = voxblox::Point(generator_resolution, 0, 0);
            break;
        case 1:
            delta = voxblox::Point(0, -generator_resolution, 0);
            break;
        case 2:
            delta = voxblox::Point(-generator_resolution, 0, 0);
            break;
        case 3:
            delta = voxblox::Point(0, generator_resolution, 0);
            break;
        default:
            break;
        }
        for(int i_point = 0; i_point < num_points_per_side; ++i_point) {
            point += delta;
            points_C.emplace_back(point);
            points_carto.emplace_back(Eigen::Vector3f(point[0], point[1], point[2]));
            colors.emplace_back(voxblox::Color::Gray());
        }
    }

    point = voxblox::Point(0, start_offset, -start_offset);
    for(int i_side = 0; i_side < 4; ++i_side)
    {
        switch (i_side) {
        case 0:
            delta = voxblox::Point(0, 0, generator_resolution);
            break;
        case 1:
            delta = voxblox::Point(0, -generator_resolution, 0);
            break;
        case 2:
            delta = voxblox::Point(0, 0, -generator_resolution);
            break;
        case 3:
            delta = voxblox::Point(0, generator_resolution, 0);
            break;
        default:
            break;
        }
        for(int i_point = 0; i_point < num_points_per_side; ++i_point) {
            point += delta;
            points_C.emplace_back(point);
            points_carto.emplace_back(Eigen::Vector3f(point[0], point[1], point[2]));
            colors.emplace_back(voxblox::Color::Gray());
        }
    }

    LOG(INFO)<<"e";
    tsdf_integrator->integratePointCloud(T_G_C, points_C, colors);

    LOG(INFO)<<"f";
    cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions options;
    options.set_translation_weight(0.0000001);
    options.set_rotation_weight(0.0000000001);
    options.add_occupied_space_weight(10.0);
    options.mutable_ceres_solver_options()->set_max_num_iterations(100);
    options.mutable_ceres_solver_options()->set_num_threads(1);
    cartographer::mapping_3d::scan_matching::CeresVoxbloxTSDFScanMatcher voxblox_scan_matcher(options);
    const cartographer::transform::Rigid3d previous_pose;
    const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d::Translation({0,0,0.1});
    const std::vector<cartographer::mapping_3d::scan_matching::PointCloudAndVoxbloxTSDFPointers>
            point_clouds_and_tsdfs;
    //cartographer::mapping_3d::scan_matching::PointCloudAndVoxbloxTSDFPointers<>
    float max_truncation_distance = 0.2;
    int coarsening_factor = 1;
    cartographer::transform::Rigid3d pose_estimate;
    ceres::Solver::Summary summary;
    LOG(INFO)<<"Start match";
    voxblox_scan_matcher.Match(previous_pose,
                               initial_pose_estimate,
                               point_clouds_and_tsdfs,
                               max_truncation_distance,
                               coarsening_factor,
                               &pose_estimate,
                               &summary);


    LOG(INFO)<<"Before "<<initial_pose_estimate;

    LOG(INFO)<<"After "<<pose_estimate;

    LOG(INFO)<<summary.BriefReport();

}
