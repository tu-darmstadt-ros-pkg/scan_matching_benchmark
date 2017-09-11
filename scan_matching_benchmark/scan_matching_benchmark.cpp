#include "scan_matching_benchmark.h"

#include "test_set_generator.h"

#include <cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/range_data_inserter.h>
#include <cartographer/mapping_3d/proto/range_data_inserter_options.pb.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_voxblox_tsdf.h>
#include <cartographer/mapping_3d/scan_matching/interpolated_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <voxblox/core/common.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_ros/ptcloud_vis.h>
#include <voxblox/interpolator/interpolator.h>

#include <iostream>

enum ScanMatcherType { PROBABILITY_GRID, CHISEL_TSDF, VOXBLOX_TSDF };

ScanMatchingBenchmark::ScanMatchingBenchmark(ros::NodeHandle &nh)
{
  cartographer::sensor::PointCloud pointcloud;

  TestSetGenerator generator(0.02, 2.0);
  generator.generateCube(pointcloud);


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

  ros::spinOnce();

  ScanMatcherType scan_matcher_type = PROBABILITY_GRID;

  pcl::PointCloud<pcl::PointXYZI> hybrid_grid_cloud;
  pcl::PointCloud<pcl::PointXYZI> hybrid_grid_interpolated_cloud;
  pcl::PointCloud<pcl::PointXYZI> voxblox_tsdf_cloud;
  pcl::PointCloud<pcl::PointXYZI> voxblox_interpolated_tsdf_cloud;

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

  std::cout << "VOXBLOX_TSDF\n";
  evaluateVoxbloxTSDFScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, voxblox_tsdf_cloud, voxblox_interpolated_tsdf_cloud, summary);
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

  ros::spin();



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
  integrator_config.default_truncation_distance = 0.1;
  integrator_config.max_ray_length_m = 30.0;

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
  double min_x = -1.5;
  double min_y = -1.5;
  double min_z = 0.;
  double max_x = 1.5;
  double max_y = 1.5;
  double max_z = 0.00;
  for(double x = min_x; x <= max_x; x += 0.001) {
    for(double y = min_y; y <= max_y; y += 0.001) {
      for(double z = min_z; z <= max_z; z += 0.001) {

        double q = max_truncation_distance;
        q = interpolated_voxblox_tsdf.GetSDF((double)x,(double)y,(double)z,1);
        double x1, y1, z1, x2, y2, z2;

        interpolated_voxblox_tsdf.ComputeInterpolationDataPoints(x, y, z, &x1, &y1, &z1, &x2, &y2, &z2);
        //x1 += 0.025;
        //x2 += 0.025;
       // y1 += 0.025;
       // y2 += 0.025;
       // z1 += 0.025;
      //  z2 += 0.025;

        if(x1 > x){
          x1 -= 0.05;
          x2 -= 0.05;
        }
        if(z1 > z){
          z1 -= 0.05;
          z2 -= 0.05;
        }
        if(z1 > z){
          z1 -= 0.05;
          z2 -= 0.05;
        }

        //ROS_INFO("point p %f %f %f p0 %f %f %f p1 %f %f %f",x,y,z,x1, y1, z1, x2, y2, z2);

        const double q111 = interpolated_voxblox_tsdf.getVoxelSDF(x1, y1, z1);
        const double q112 = interpolated_voxblox_tsdf.getVoxelSDF(x1, y1, z2);
        const double q121 = interpolated_voxblox_tsdf.getVoxelSDF(x1, y2, z1);
        const double q122 = interpolated_voxblox_tsdf.getVoxelSDF(x1, y2, z2);
        const double q211 = interpolated_voxblox_tsdf.getVoxelSDF(x2, y1, z1);
        const double q212 = interpolated_voxblox_tsdf.getVoxelSDF(x2, y1, z2);
        const double q221 = interpolated_voxblox_tsdf.getVoxelSDF(x2, y2, y1);
        const double q222 = interpolated_voxblox_tsdf.getVoxelSDF(x2, y2, z2);


        //ROS_INFO("point q %f %f %f  %f %f %f  %f %f",q111,q112,q121,q122,q211,q212,q221,q222);

        const double normalized_x = (x - x1) / (x2 - x1);
        const double normalized_y = (y - y1) / (y2 - y1);
        const double normalized_z = (z - z1) / (z2 - z1);

        // Compute pow(..., 2) and pow(..., 3). Using pow() here is very expensive.
        const double normalized_xx = normalized_x * normalized_x;
        const double normalized_xxx = normalized_x * normalized_xx;
        const double normalized_yy = normalized_y * normalized_y;
        const double normalized_yyy = normalized_y * normalized_yy;
        const double normalized_zz = normalized_z * normalized_z;
        const double normalized_zzz = normalized_z * normalized_zz;

        // We first interpolate in z, then y, then x. All 7 times this uses the same
        // scheme: A * (2t^3 - 3t^2 + 1) + B * (-2t^3 + 3t^2).
        // The first polynomial is 1 at t=0, 0 at t=1, the second polynomial is 0
        // at t=0, 1 at t=1. Both polynomials have derivative zero at t=0 and t=1.
        const double q11 = (q111 - q112) * normalized_zzz * 2. +
            (q112 - q111) * normalized_zz * 3. + q111;
        const double q12 = (q121 - q122) * normalized_zzz * 2. +
            (q122 - q121) * normalized_zz * 3. + q121;
        const double q21 = (q211 - q212) * normalized_zzz * 2. +
            (q212 - q211) * normalized_zz * 3. + q211;
        const double q22 = (q221 - q222) * normalized_zzz * 2. +
            (q222 - q221) * normalized_zz * 3. + q221;
        const double q1 = (q11 - q12) * normalized_yyy * 2. +
            (q12 - q11) * normalized_yy * 3. + q11;
        const double q2 = (q21 - q22) * normalized_yyy * 2. +
            (q22 - q21) * normalized_yy * 3. + q21;
        double int_res = (q1 - q2) * normalized_xxx * 2. + (q2 - q1) * normalized_xx * 3. +
            q1;

        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = int_res;
        interpolated_voxblox_tsdf_cloud.push_back(p);
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
  range_data_inserter.Insert(range_data, &hybrid_grid_low_res);

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
  float min_y = -2.;
  float min_z = 0.;
  float max_x = 2.;
  float max_y = 2.;
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

