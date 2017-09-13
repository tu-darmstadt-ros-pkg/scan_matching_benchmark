#include "scan_matching_benchmark.h"

#include "test_set_generator.h"

#include <cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_esdf_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/range_data_inserter.h>
#include <cartographer/mapping_3d/proto/range_data_inserter_options.pb.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_voxblox_tsdf.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_tsdf.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_grid.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_voxblox_esdf.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/truncation/ConstantTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include <open_chisel/Chisel.h>
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

enum ScanMatcherType { PROBABILITY_GRID, CHISEL_TSDF, VOXBLOX_TSDF };

ScanMatchingBenchmark::ScanMatchingBenchmark(ros::NodeHandle &nh)
{
  cartographer::sensor::PointCloud pointcloud;

  TestSetGenerator generator(0.021);
  generator.generateCuboid(pointcloud, 2.025, 12.025, 2.025);


  ceres_scan_matcher_options_.set_translation_weight(0.0000001);
  ceres_scan_matcher_options_.set_rotation_weight(0.0000000001);
  ceres_scan_matcher_options_.add_occupied_space_weight(10.0);
  //ceres_scan_matcher_options_.add_occupied_space_weight(10.0);
  ceres_scan_matcher_options_.mutable_ceres_solver_options()->set_max_num_iterations(300);
  ceres_scan_matcher_options_.mutable_ceres_solver_options()->set_num_threads(1);
  const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d::Translation({0.05,0.1,0.2});
  cartographer::transform::Rigid3d matched_pose_estimate;
  ceres::Solver::Summary summary;

  ros::Publisher benchmark_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("benchmark_pointcloud", 1, true);
  ros::Publisher hybrid_grid_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("hybrid_grid_pointcloud", 1, true);
  ros::Publisher hybrid_grid_interpolated_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("hybrid_grid_interpolated_pointcloud", 1, true);
  ros::Publisher voxblox_tsdf_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("voxblox_tsdf_pointcloud", 1, true);
  ros::Publisher voxblox_interpolated_tsdf_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("voxblox_interpolated_tsdf_pointcloud", 1, true);
  ros::Publisher voxblox_esdf_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("voxblox_esdf_pointcloud", 1, true);
  ros::Publisher voxblox_interpolated_esdf_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("voxblox_interpolated_esdf_pointcloud", 1, true);
  ros::Publisher chisel_tsdf_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("chisel_tsdf_pointcloud", 1, true);
  ros::Publisher chisel_interpolated_tsdf_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("chisel_interpolated_tsdf_pointcloud", 1, true);

  ros::spinOnce();

  ScanMatcherType scan_matcher_type = PROBABILITY_GRID;

  pcl::PointCloud<pcl::PointXYZI> hybrid_grid_cloud;
  pcl::PointCloud<pcl::PointXYZI> hybrid_grid_interpolated_cloud;
  pcl::PointCloud<pcl::PointXYZI> voxblox_esdf_cloud;
  pcl::PointCloud<pcl::PointXYZI> voxblox_interpolated_esdf_cloud;
  pcl::PointCloud<pcl::PointXYZI> voxblox_tsdf_cloud;
  pcl::PointCloud<pcl::PointXYZI> voxblox_interpolated_tsdf_cloud;
  pcl::PointCloud<pcl::PointXYZI> chisel_tsdf_cloud;
  pcl::PointCloud<pcl::PointXYZI> chisel_interpolated_tsdf_cloud;

  /*
  switch(scan_matcher_type)
  {
  case PROBABILITY_GRID:
    std::cout << "PROBABILITY_GRID\n";
    evaluateProbabilityGridScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, hybrid_grid_cloud, summary);
    break;
  case CHISEL_TSDF:
    std::cout << "green\n";
    break;
  case VOXBLOX_TSDF:
    std::cout << "VOXBLOX_TSDF\n";
    evaluateVoxbloxTSDFScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, voxblox_tsdf_cloud, summary);
    break;
  }
*/

  std::cout << "PROBABILITY_GRID\n";
  evaluateProbabilityGridScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, hybrid_grid_cloud,hybrid_grid_interpolated_cloud, summary);
  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;

  std::cout << "VOXBLOX_ESDF\n";
  evaluateVoxbloxESDFScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, voxblox_esdf_cloud, voxblox_interpolated_esdf_cloud, summary);
  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;

  std::cout << "VOXBLOX_TSDF\n";
  evaluateVoxbloxTSDFScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, voxblox_tsdf_cloud, voxblox_interpolated_tsdf_cloud, summary);
  std::cout<<"Before "<<initial_pose_estimate<<std::endl;
  std::cout<<"After "<<matched_pose_estimate<<std::endl;
  std::cout<<summary.BriefReport()<<std::endl;

  std::cout << "CHISEL_TSDF\n";
  evaluateChiselTSDFScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, chisel_tsdf_cloud, chisel_interpolated_tsdf_cloud, summary);
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

  sensor_msgs::PointCloud2 hybrid_grid_cloud_msg;
  pcl::toROSMsg(hybrid_grid_cloud, hybrid_grid_cloud_msg);
  hybrid_grid_cloud_msg.header.stamp = ros::Time::now();
  hybrid_grid_cloud_msg.header.frame_id = "world";
  hybrid_grid_pointcloud_publisher.publish(hybrid_grid_cloud_msg);

  sensor_msgs::PointCloud2 hybrid_grid_interpolated_cloud_msg;
  pcl::toROSMsg(hybrid_grid_interpolated_cloud, hybrid_grid_interpolated_cloud_msg);
  hybrid_grid_interpolated_cloud_msg.header.stamp = ros::Time::now();
  hybrid_grid_interpolated_cloud_msg.header.frame_id = "world";
  hybrid_grid_interpolated_pointcloud_publisher.publish(hybrid_grid_interpolated_cloud_msg);

  sensor_msgs::PointCloud2 voxblox_tsdf_cloud_msg;
  pcl::toROSMsg(voxblox_tsdf_cloud, voxblox_tsdf_cloud_msg);
  voxblox_tsdf_cloud_msg.header.stamp = ros::Time::now();
  voxblox_tsdf_cloud_msg.header.frame_id = "world";
  voxblox_tsdf_pointcloud_publisher.publish(voxblox_tsdf_cloud_msg);

  sensor_msgs::PointCloud2 voxblox_interpolated_tsdf_cloud_msg;
  pcl::toROSMsg(voxblox_interpolated_tsdf_cloud, voxblox_interpolated_tsdf_cloud_msg);
  voxblox_interpolated_tsdf_cloud_msg.header.stamp = ros::Time::now();
  voxblox_interpolated_tsdf_cloud_msg.header.frame_id = "world";
  voxblox_interpolated_tsdf_pointcloud_publisher.publish(voxblox_interpolated_tsdf_cloud_msg);

  sensor_msgs::PointCloud2 voxblox_esdf_cloud_msg;
  pcl::toROSMsg(voxblox_esdf_cloud, voxblox_esdf_cloud_msg);
  voxblox_esdf_cloud_msg.header.stamp = ros::Time::now();
  voxblox_esdf_cloud_msg.header.frame_id = "world";
  voxblox_esdf_pointcloud_publisher.publish(voxblox_esdf_cloud_msg);

  sensor_msgs::PointCloud2 voxblox_interpolated_esdf_cloud_msg;
  pcl::toROSMsg(voxblox_interpolated_esdf_cloud, voxblox_interpolated_esdf_cloud_msg);
  voxblox_interpolated_esdf_cloud_msg.header.stamp = ros::Time::now();
  voxblox_interpolated_esdf_cloud_msg.header.frame_id = "world";
  voxblox_interpolated_esdf_pointcloud_publisher.publish(voxblox_interpolated_esdf_cloud_msg);

  sensor_msgs::PointCloud2 chisel_tsdf_cloud_msg;
  pcl::toROSMsg(chisel_tsdf_cloud, chisel_tsdf_cloud_msg);
  chisel_tsdf_cloud_msg.header.stamp = ros::Time::now();
  chisel_tsdf_cloud_msg.header.frame_id = "world";
  chisel_tsdf_pointcloud_publisher.publish(chisel_tsdf_cloud_msg);

  sensor_msgs::PointCloud2 chisel_interpolated_tsdf_cloud_msg;
  pcl::toROSMsg(chisel_interpolated_tsdf_cloud, chisel_interpolated_tsdf_cloud_msg);
  chisel_interpolated_tsdf_cloud_msg.header.stamp = ros::Time::now();
  chisel_interpolated_tsdf_cloud_msg.header.frame_id = "world";
  chisel_interpolated_tsdf_pointcloud_publisher.publish(chisel_interpolated_tsdf_cloud_msg);

  ros::spin();



}

void ScanMatchingBenchmark::evaluateChiselTSDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                                          const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                          cartographer::transform::Rigid3d& matched_pose_estimate,
                                                          pcl::PointCloud<pcl::PointXYZI>& chisel_tsdf_cloud,
                                                          pcl::PointCloud<pcl::PointXYZI>& interpolated_chisel_tsdf_cloud,
                                                          ceres::Solver::Summary& summary)
{
  chisel::PointCloud cloudOut;
  cloudOut.GetMutablePoints().resize(cloud.size());

  size_t i = 0;
  for (const Eigen::Vector3f& pt : cloud)
  {
    chisel::Vec3& xyz =  cloudOut.GetMutablePoints().at(i);
    xyz(0) = pt(0);
    xyz(1) = pt(1);
    xyz(2) = pt(2);
    i++;
  }

  chisel::Vec3 chisel_pose;
  chisel_pose.x() = 0.;
  chisel_pose.y() = 0.;
  chisel_pose.z() = 0.;

  chisel::ChiselPtr<chisel::DistVoxel> chisel_tsdf;
  chisel_tsdf.reset(new chisel::Chisel<chisel::DistVoxel>({8,8,8}, 0.05, false, {0,0,0}));

  chisel::ProjectionIntegrator projection_integrator;
  projection_integrator.SetCentroids(chisel_tsdf->GetChunkManager().GetCentroids());
  projection_integrator.SetTruncator(chisel::TruncatorPtr(new chisel::ConstantTruncator(0.1, 1.0)));
  projection_integrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(1)));
  projection_integrator.SetCarvingDist(0.2);
  projection_integrator.SetCarvingEnabled(false);

  chisel_tsdf->GetMutableChunkManager().clearIncrementalChanges();
  //min and max dist are already filtered in the local trajectory builder
  chisel_tsdf->IntegratePointCloud(projection_integrator, cloudOut,
                                   chisel_pose, 0.0f, HUGE_VALF);
  chisel_tsdf->UpdateMeshes();


  const cartographer::transform::Rigid3d previous_pose;
  cartographer::mapping_3d::scan_matching::CeresTSDFScanMatcher chisel_scan_matcher(ceres_scan_matcher_options_);
  chisel_scan_matcher.Match(previous_pose,
                             initial_pose_estimate,
  {{&cloud, chisel_tsdf}},
                             0.12,
                             1,
                             &matched_pose_estimate,
                             &summary);



  cartographer::mapping_3d::scan_matching::InterpolatedTSDF interpolated_chisel_tsdf(chisel_tsdf, 0.12);
  double min_x = -1.2;
  double min_y = -51.2;
  double min_z = 0.001;
  double max_x = 1.2;
  double max_y = 51.2;
  double max_z = 0.001;
  for(double x = min_x; x <= max_x; x += 0.01) {
    for(double y = min_y; y <= max_y; y += 0.01) {
      for(double z = min_z; z <= max_z; z += 0.01) {

        double q = interpolated_chisel_tsdf.GetSDF((double)x,(double)y,(double)z,1);
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = std::abs(q);
        interpolated_chisel_tsdf_cloud.push_back(p);
      }
    }
  }

   min_x = -1.2;
   min_y = -6.2;
   min_z = 0.000;
   max_x = 1.2;
   max_y = 6.2;
   max_z = 0.001;
  for(double x = min_x; x <= max_x; x += 0.05) {
    for(double y = min_y; y <= max_y; y += 0.05) {
      for(double z = min_z; z <= max_z; z += 0.05) {
        const auto& chunk_manager = chisel_tsdf->GetChunkManager();
        const chisel::DistVoxel* voxel = chunk_manager.GetDistanceVoxel(chisel::Vec3(x,y,z));

        double sdf = NAN;
        bool valid = false;
        if(voxel) {
          if(voxel->IsValid()) {
            sdf = voxel->GetSDF();
            valid = true;
          }
        }
        if(std::abs(sdf) > 10.0 )
        {
          //LOG(WARNING)<<"q > max_truncation_distance "<< q <<" > "<< max_truncation_distance_;
          //s LOG(WARNING)<<"weight: "<< voxel->GetWeight();
          sdf = NAN;
          valid = false;
        }
        if(valid) {
          pcl::PointXYZI p;
          p.x = x;
          p.y = y;
          p.z = z;
          p.intensity = sdf;
          chisel_tsdf_cloud.push_back(p);
        }

      }
    }
  }


}

void ScanMatchingBenchmark::evaluateVoxbloxTSDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                                           const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                           cartographer::transform::Rigid3d& matched_pose_estimate,
                                                           pcl::PointCloud<pcl::PointXYZI>& voxblox_tsdf_cloud,
                                                           pcl::PointCloud<pcl::PointXYZI>& interpolated_voxblox_tsdf_cloud,
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

  voxblox_tsdf_.reset(new voxblox::TsdfMap(tsdf_config));

  voxblox::TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  integrator_config.default_truncation_distance = 0.5;
  integrator_config.max_ray_length_m = 120.0;

  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator;
  tsdf_integrator.reset(new voxblox::SimpleTsdfIntegrator(
                          integrator_config, voxblox_tsdf_->getTsdfLayerPtr())); //todo(kdaun) add config to choose between integrators

  tsdf_integrator->integratePointCloud(T_G_C, points_C, colors);

  float max_truncation_distance = integrator_config.default_truncation_distance;
  int coarsening_factor = 1;

  const cartographer::transform::Rigid3d previous_pose;
  cartographer::mapping_3d::scan_matching::CeresVoxbloxTSDFScanMatcher voxblox_scan_matcher(ceres_scan_matcher_options_);
  voxblox_scan_matcher.Match(previous_pose,
                             initial_pose_estimate,
  {{&cloud, voxblox_tsdf_}},
                             max_truncation_distance,
                             coarsening_factor,
                             &matched_pose_estimate,
                             &summary);



  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_unfiltered(new pcl::PointCloud<pcl::PointXYZI>);
  createDistancePointcloudFromTsdfLayer(voxblox_tsdf_->getTsdfLayer(), tsdf_unfiltered.get());
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (tsdf_unfiltered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.06);
  //pass.setFilterLimitsNegative (true);
  pass.filter (voxblox_tsdf_cloud);
  for(auto& p : voxblox_tsdf_cloud)
    p.intensity = std::abs(p.intensity);

  cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxTSDF interpolated_voxblox_tsdf(voxblox_tsdf_, max_truncation_distance);
  double min_x = -1.2;
  double min_y = -51.2;
  double min_z = 0.001;
  double max_x = 1.2;
  double max_y = 51.2;
  double max_z = 0.001;
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
        interpolated_voxblox_tsdf_cloud.push_back(p);
      }
    }
  }
}


void ScanMatchingBenchmark::evaluateVoxbloxESDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                                           const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                           cartographer::transform::Rigid3d& matched_pose_estimate,
                                                           pcl::PointCloud<pcl::PointXYZI>& voxblox_esdf_cloud,
                                                           pcl::PointCloud<pcl::PointXYZI>& interpolated_voxblox_esdf_cloud,
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

  voxblox_tsdf_.reset(new voxblox::TsdfMap(tsdf_config));

  voxblox::TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  integrator_config.default_truncation_distance = 0.5;
  integrator_config.max_ray_length_m = 120.0;

  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator;
  tsdf_integrator.reset(new voxblox::SimpleTsdfIntegrator(
                          integrator_config, voxblox_tsdf_->getTsdfLayerPtr())); //todo(kdaun) add config to choose between integrators

  voxblox::EsdfMap::Config esdf_config;
  esdf_config.esdf_voxel_size = static_cast<voxblox::FloatingPoint>(0.05);
  //esdf_config.esdf_voxels_per_side = config.tsdf_voxels_per_side; //todo(kdaun) set form config
  std::shared_ptr<voxblox::EsdfMap> voxblox_esdf(new voxblox::EsdfMap(esdf_config));

  voxblox::EsdfIntegrator::Config esdf_integrator_config;
  // Make sure that this is the same as the truncation distance OR SMALLER!
  esdf_integrator_config.min_distance_m =
      integrator_config.default_truncation_distance;
  esdf_integrator_config.max_distance_m = 2.0;
  esdf_integrator_config.default_distance_m = 2.0;
  //esdf_integrator_config.min_diff_m = 0.3;

  std::shared_ptr<voxblox::EsdfIntegrator> esdf_integrator;
  esdf_integrator.reset(new voxblox::EsdfIntegrator(esdf_integrator_config,
                                            voxblox_tsdf_->getTsdfLayerPtr(),
                                            voxblox_esdf->getEsdfLayerPtr()));


  tsdf_integrator->integratePointCloud(T_G_C, points_C, colors);

  const bool clear_esdf = true; //todo(kdaun) copied this from voxblox for now, should go to config
  if (clear_esdf) {
    esdf_integrator->updateFromTsdfLayerBatch();
  } else {
    const bool clear_updated_flag = true;
    esdf_integrator->updateFromTsdfLayer(clear_updated_flag);
  }

  esdf_integrator->updateFromTsdfLayerBatch();
  esdf_integrator->updateFromTsdfLayer(true);

  float max_truncation_distance = esdf_integrator_config.default_distance_m;
  int coarsening_factor = 1;

  const cartographer::transform::Rigid3d previous_pose;
  cartographer::mapping_3d::scan_matching::CeresVoxbloxESDFScanMatcher voxblox_scan_matcher(ceres_scan_matcher_options_);
  voxblox_scan_matcher.Match(previous_pose,
                             initial_pose_estimate,
  {{&cloud, voxblox_esdf}},
                             max_truncation_distance,
                             coarsening_factor,
                             &matched_pose_estimate,
                             &summary);



  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_unfiltered(new pcl::PointCloud<pcl::PointXYZI>);
  createDistancePointcloudFromEsdfLayer(voxblox_esdf->getEsdfLayer(), tsdf_unfiltered.get());
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (tsdf_unfiltered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.06);
  //pass.setFilterLimitsNegative (true);
  pass.filter (voxblox_esdf_cloud);
  for(auto& p : voxblox_esdf_cloud)
    p.intensity = std::abs(p.intensity);

  cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxESDF interpolated_voxblox_esdf(voxblox_esdf, max_truncation_distance);
  double min_x = -1.2;
  double min_y = -51.2;
  double min_z = 0.001;
  double max_x = 1.2;
  double max_y = 51.2;
  double max_z = 0.001;
  for(double x = min_x; x <= max_x; x += 0.01) {
    for(double y = min_y; y <= max_y; y += 0.01) {
      for(double z = min_z; z <= max_z; z += 0.01) {

        double q = max_truncation_distance;
        q = interpolated_voxblox_esdf.GetSDF((double)x,(double)y,(double)z,1);


        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = std::abs(q);
        interpolated_voxblox_esdf_cloud.push_back(p);
      }
    }
  }
}


void ScanMatchingBenchmark::evaluateProbabilityGridScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                                               const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                               cartographer::transform::Rigid3d& matched_pose_estimate,
                                                               pcl::PointCloud<pcl::PointXYZI>& probability_grid,
                                                               pcl::PointCloud<pcl::PointXYZI>& interpolated_probability_grid_cloud,
                                                               ceres::Solver::Summary& summary) {
  probability_grid.clear();
  interpolated_probability_grid_cloud.clear();
  cartographer::mapping_3d::HybridGrid hybrid_grid_high_res(0.05);
  cartographer::mapping_3d::HybridGrid hybrid_grid_low_res(0.4);
  cartographer::mapping_3d::proto::RangeDataInserterOptions range_data_inserter_options;
  range_data_inserter_options.set_hit_probability(0.56);
  range_data_inserter_options.set_miss_probability(0.44);
  range_data_inserter_options.set_num_free_space_voxels(5);
  cartographer::mapping_3d::RangeDataInserter range_data_inserter(range_data_inserter_options);

  cartographer::sensor::RangeData range_data;
  range_data.returns = cloud;
  range_data.origin = Eigen::Vector3f{0,0,0};

  range_data_inserter.Insert(range_data, &hybrid_grid_high_res);
//  range_data_inserter.Insert(range_data, &hybrid_grid_low_res);

  const cartographer::transform::Rigid3d previous_pose;
  cartographer::mapping_3d::scan_matching::CeresScanMatcher scan_matcher(ceres_scan_matcher_options_);
  scan_matcher.Match(previous_pose,
                     initial_pose_estimate,
  {{&cloud, &hybrid_grid_high_res}/*, {&cloud, &hybrid_grid_low_res}*/},
                     &matched_pose_estimate,
                     &summary);
  for(auto& p : hybrid_grid_high_res)
  {
    float probability =  hybrid_grid_high_res.GetProbability(p.first);
    Eigen::Vector3f cell_center = hybrid_grid_high_res.GetCenterOfCell(p.first);
    pcl::PointXYZI point;
    point.x = cell_center[0];
    point.y = cell_center[1];
    point.z = cell_center[2];
    point.intensity = probability;
    probability_grid.push_back(point);
  }

  cartographer::mapping_3d::scan_matching::InterpolatedGrid interpolated_grid(hybrid_grid_high_res);
  float min_x = -2.;
  float min_y = -51.2;
  float min_z = 0.;
  float max_x = 2.;
  float max_y = 51.2;
  float max_z = 0.01;
  for(float x = min_x; x <= max_x; x += 0.01) {
    for(float y = min_y; y <= max_y; y += 0.01) {
      for(float z = min_z; z <= max_z; z += 0.01) {
        float q = std::abs(interpolated_grid.GetProbability((double)x,(double)y,(double)z));
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = q;
        interpolated_probability_grid_cloud.push_back(p);
      }
    }
  }
}

