#include "chisel_tsdf_scan_matcher.h"

#include <cartographer/mapping_3d/scan_matching/ceres_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_tsdf.h>
#include <cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h>
#include <cartographer/mapping_3d/range_data_inserter.h>

#include <open_chisel/ChunkManager.h>
#include <open_chisel/truncation/ConstantTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include <open_chisel/Chisel.h>

#include <sensor_msgs/PointCloud2.h>



ChiselTSDFScanMatcher::ChiselTSDFScanMatcher(ros::NodeHandle &nh, ScanMatcherConfig config)
{
  config_ = config;

  if(config.publish_cloud) {
    map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("chisel_tsdf_pointcloud", 1, true);
    interpolated_map_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("chisel_tsdf_interpolated_pointcloud", 1, true);
    ros::spinOnce();
  }

}
void ChiselTSDFScanMatcher::evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                                                const cartographer::transform::Rigid3d& initial_pose_estimate,
                                                cartographer::transform::Rigid3d& matched_pose_estimate,
                                                double &time_map_update, //seconds
                                                double &time_scan_matching, //seconds
                                                ceres::Solver::Summary& summary) {
  std::cout << "CHISEL_TSDF\n";
  chisel::PointCloud cloudOut;
  cloudOut.GetMutablePoints().resize(scan_cloud.size());

  size_t i = 0;
  for (const Eigen::Vector3f& pt : scan_cloud)
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
  chisel_tsdf.reset(new chisel::Chisel<chisel::DistVoxel>({8,8,8}, config_.resolution, false, {0,0,0}));

  chisel::ProjectionIntegrator projection_integrator;
  projection_integrator.SetCentroids(chisel_tsdf->GetChunkManager().GetCentroids());
  projection_integrator.SetTruncator(chisel::TruncatorPtr(new chisel::ConstantTruncator(config_.truncation_distance, 1.0)));
  projection_integrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(1)));
  projection_integrator.SetCarvingDist(0.0);
  projection_integrator.SetCarvingEnabled(false);

  chisel_tsdf->GetMutableChunkManager().clearIncrementalChanges();
  //min and max dist are already filtered in the local trajectory builder

  std::clock_t start_map_update = std::clock();
  chisel_tsdf->IntegratePointCloud(projection_integrator, cloudOut,
                                   chisel_pose, 0.0f, HUGE_VALF);
  time_map_update = (std::clock() - start_map_update) / (double)CLOCKS_PER_SEC;
  //chisel_tsdf->UpdateMeshes();


  const cartographer::transform::Rigid3d previous_pose;

  cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions ceres_scan_matcher_options;
  ceres_scan_matcher_options.set_translation_weight(0.0);
  ceres_scan_matcher_options.set_rotation_weight(0.0);
  ceres_scan_matcher_options.add_occupied_space_weight(10.0);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_max_num_iterations(300);
  ceres_scan_matcher_options.mutable_ceres_solver_options()->set_num_threads(1);


  cartographer::mapping_3d::scan_matching::CeresTSDFScanMatcher chisel_scan_matcher(ceres_scan_matcher_options);
  std::clock_t start_scan_matching = std::clock();
  chisel_scan_matcher.Match(initial_pose_estimate,
                            initial_pose_estimate,
  {{&scan_cloud, chisel_tsdf}},
                            config_.truncation_distance*1.2,
                            1,
                            &matched_pose_estimate,
                            &summary);
  time_scan_matching = (std::clock() - start_scan_matching) / (double)CLOCKS_PER_SEC;



  cartographer::mapping_3d::scan_matching::InterpolatedTSDF interpolated_chisel_tsdf(chisel_tsdf, config_.truncation_distance*1.2);
  float min_x = config_.interpolation_map_min_x;
  float min_y = config_.interpolation_map_min_y;
  float min_z = config_.interpolation_map_min_z;
  float max_x = config_.interpolation_map_max_x;
  float max_y = config_.interpolation_map_max_y;
  float max_z = config_.interpolation_map_max_z;
  for(double x = min_x; x <= max_x; x += 0.01) {
    for(double y = min_y; y <= max_y; y += 0.01) {
      for(double z = min_z; z <= max_z; z += 0.01) {

        double q = interpolated_chisel_tsdf.GetSDF((double)x,(double)y,(double)z,1);
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = std::abs(q);
        interpolated_map_cloud_.push_back(p);
      }
    }
  }


  for(double x = min_x; x <= max_x; x += config_.resolution) {
    for(double y = min_y; y <= max_y; y += config_.resolution) {
      for(double z = min_z; z <= max_z; z += config_.resolution) {
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
        if(std::abs(sdf) > 1.2 * config_.truncation_distance)
        {
          sdf = NAN;
          valid = false;
        }
        if(valid) {
          pcl::PointXYZI p;
          p.x = x;
          p.y = y;
          p.z = z;
          p.intensity = sdf;
          map_cloud_.push_back(p);
        }

      }
    }
  }

  if(config_.publish_cloud) {
    publishClouds();
  }
}
