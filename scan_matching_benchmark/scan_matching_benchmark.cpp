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

#include <ctime>
#include <iostream>
#include <random>
#include <stdlib.h>
#include <ros/package.h>

float computeReprojectionError(const cartographer::sensor::PointCloud& scan_cloud,
                               const cartographer::transform::Rigid3d& pose_estimate) {
  cartographer::sensor::PointCloud projected_cloud = cartographer::sensor::TransformPointCloud(scan_cloud, pose_estimate.cast<float>());
  float summed_reprojection_error = 0.;
  for(int i = 0; i < projected_cloud.size(); ++i) {
    Eigen::Vector3f delta = projected_cloud[i]- scan_cloud[i];
    summed_reprojection_error += std::sqrt(delta.dot(delta));
  }
  return summed_reprojection_error / float(projected_cloud.size() - 1);
}


BatchScanMatchingBenchmark::BatchScanMatchingBenchmark(ros::NodeHandle &nh)
{

  int num_iterations_per_initial_error;
  double min_initial_error, max_initial_error, initial_error_stepsize, sample_resolution;
  std::string scenario_source, sample_type, sample_path;
  nh.param<int>("num_iterations_per_initial_error", num_iterations_per_initial_error, 1);
  nh.param<double>("min_initial_error", min_initial_error, 0.0);
  nh.param<double>("max_initial_error", max_initial_error, 0.8);
  nh.param<double>("sample_resolution", sample_resolution, 0.082413124);
  nh.param<double>("initial_error_stepsize", initial_error_stepsize, 0.2);
  nh.param<std::string>("scenario_source", scenario_source, "generator");
  nh.param<std::string>("sample_type", sample_type, "half_cylinder_half_cuboid");
  nh.param<std::string>("sample_path", sample_path, "");

  std::ofstream myfile;
  std::string log_file_path;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "_%m-%d-%Y_%H-%M-%S");
  std::string str = oss.str();
  log_file_path="scan_matching_benchmark"+str+".csv";
  myfile.open (log_file_path);
  myfile <<"scan_matcher"<<","<<"sample_resolution"<<","
        <<"sample_type"<<","<<"sample_size_x"<<","<<"sample_size_y"<<","<<"sample_size_z"<<","
       <<"grid_resolution"<<","<<"truncation_distance"<<","<<"esdf_distance"<<","
      <<"initial_error_x"<<","<<"initial_error_y"<<","<<"initial_error_z"<<","<<"initial_error_angle"<<","
     <<"matched_error_x"<<","<<"matched_error_y"<<","<<"matched_error_z"<<","<<"matched_error_angle"
    <<","<<"solver_iterations"<<","<<"solver_termination_type"<<","
   <<"time_map_update"<<","<<"time_scan_matching"<<","<<"boundary_extrapolation"<<","<<"cubic_interpolation"<<","<<"reprojection_error"<<"\n";

  std::random_device r;
  std::default_random_engine e1(M_PI);
  std::uniform_real_distribution<float> uniform_dist(-1, 1);

  ScanMatcherConfig scan_matcher_config;
  scan_matcher_config.publish_cloud = false;


  for(float initial_error = min_initial_error; initial_error <= max_initial_error; initial_error += initial_error_stepsize) {
    std::cout<<"Finished "<<(initial_error-min_initial_error)*100.0/(max_initial_error-min_initial_error)<<"%"<<std::endl;
    for(int i_initial_error= 0; i_initial_error < num_iterations_per_initial_error; ++i_initial_error) {
      cartographer::sensor::PointCloud pointcloud;
      double sample_size_x = 1.2 + scan_matcher_config.resolution * uniform_dist(e1) * 0.5;
      double sample_size_y = 1.2 + scan_matcher_config.resolution * uniform_dist(e1) * 0.5;
      double sample_size_z = 2.0 + scan_matcher_config.resolution * uniform_dist(e1) * 0.5;
      TestSetGenerator generator(sample_resolution);
      //generator.generateCuboid(pointcloud, sample_size_x, sample_size_y, sample_size_z);
      sample_size_y = sample_size_x;
      generator.generateHalfCylinderHalfCube(pointcloud, sample_size_x, sample_size_y, sample_size_z);

      Eigen::Vector3f initial_error_unscaled({uniform_dist(e1),uniform_dist(e1),uniform_dist(e1)});
      Eigen::Vector3f initial_error_scaled = initial_error*initial_error_unscaled.normalized();
      float initial_error_x = initial_error_scaled[0];
      float initial_error_y = initial_error_scaled[1];
      float initial_error_z = initial_error_scaled[2];

      Eigen::Vector3f initial_rotation_axis_unscaled({uniform_dist(e1),uniform_dist(e1),uniform_dist(e1)});
      Eigen::Vector3f initial_rotation_axis = initial_rotation_axis_unscaled.normalized();
      float initial_error_angle = 0;//initial_error*M_PI;
      //const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d::Translation({initial_error_x,initial_error_y,initial_error_z});

      Eigen::Matrix<double, 3, 1> translation({initial_error_x,initial_error_y,initial_error_z});
      Eigen::Quaternion<double> orientation;
      orientation.w() = std::cos(initial_error_angle * 0.5);
      orientation.x() = std::sin(initial_error_angle * 0.5) * initial_rotation_axis[0];
      orientation.y() = std::sin(initial_error_angle * 0.5) * initial_rotation_axis[1];
      orientation.z() = std::sin(initial_error_angle * 0.5) * initial_rotation_axis[2];
      LOG(INFO)<<"quat "<<orientation.w()<<" "<<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z();
      const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d(translation, orientation);
      cartographer::transform::Rigid3d matched_pose_estimate;
      ceres::Solver::Summary summary;
      double time_map_update = 0.0;
      double time_scan_matching = 0.0;
      /*
        bool boundary_extrapolation = false;
        bool cubic_interpolation = false;*/
      for (bool boundary_extrapolation : { /*false,*/ true }) {
        for (bool cubic_interpolation : { false, true }) {
          scan_matcher_config.boundary_extrapolation = boundary_extrapolation;
          scan_matcher_config.cubic_interpolation = cubic_interpolation;

          ProbabilityGridScanMatcher probability_grid_scan_matcher(nh, scan_matcher_config);
          probability_grid_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"ProbabilityGridScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";

          scan_matcher_config.multi_res_probability_grid = true;
          ProbabilityGridScanMatcher probability_grid_scan_matcher_multi_res(nh, scan_matcher_config);
          probability_grid_scan_matcher_multi_res.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"ProbabilityGridScanMatcherMultiRes"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";
          scan_matcher_config.multi_res_probability_grid = false;

          ChiselTSDFScanMatcher chisel_tsdf_scan_matcher(nh, scan_matcher_config);
          chisel_tsdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"ChiselTSDFScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";

          VoxbloxTSDFScanMatcher voxblox_tsdf_scan_matcher(nh, scan_matcher_config);
          voxblox_tsdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"VoxbloxTSDFScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";

          VoxbloxESDFScanMatcher voxblox_esdf_scan_matcher(nh, scan_matcher_config);
          voxblox_esdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"VoxbloxESDFScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";
        }
      }
    }
  }


  myfile.close();
  std::cout<<"Finished benchmark"<<std::endl;

  std::string pkg_path = ros::package::getPath("scan_matching_benchmark");
  std::string filename = pkg_path +"/scan_matching_benchmark/plot_solution_online.py ";
  std::string command = "python2 ";
  command += filename;
  command += log_file_path;
  std::cout<<command<<std::endl;
  system(command.c_str());
  ros::shutdown();
}


BatchScanMatchingBenchmarkFromFile::BatchScanMatchingBenchmarkFromFile(ros::NodeHandle &nh, std::string dir)
{

  int num_iterations_per_initial_error = 8;
  float min_initial_error = 0.0;
  float max_initial_error = 1.6001;
  float initial_error_stepsize = 0.1;

  std::ofstream myfile;
  std::string log_file_path;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "_%m-%d-%Y_%H-%M-%S");
  std::string str = oss.str();
  log_file_path="scan_matching_benchmark"+str+".csv";
  myfile.open (log_file_path);
  myfile <<"scan_matcher"<<","<<"sample_resolution"<<","
        <<"sample_type"<<","<<"sample_size_x"<<","<<"sample_size_y"<<","<<"sample_size_z"<<","
       <<"grid_resolution"<<","<<"truncation_distance"<<","<<"esdf_distance"<<","
      <<"initial_error_x"<<","<<"initial_error_y"<<","<<"initial_error_z"<<","<<"initial_error_angle"<<","
     <<"matched_error_x"<<","<<"matched_error_y"<<","<<"matched_error_z"<<","<<"matched_error_angle"
    <<","<<"solver_iterations"<<","<<"solver_termination_type"<<","
   <<"time_map_update"<<","<<"time_scan_matching"<<","<<"boundary_extrapolation"<<","<<"cubic_interpolation"<<","<<"reprojection_error"<<"\n";

  std::random_device r;
  std::default_random_engine e1(M_PI);
  std::uniform_real_distribution<float> uniform_dist(-1, 1);

  ScanMatcherConfig scan_matcher_config;
  scan_matcher_config.publish_cloud = false;


  TestSetGenerator generator(0.0);
  cartographer::sensor::PointCloud pointcloud;
  generator.loadPCDDir(pointcloud, "a");
  for(float initial_error = min_initial_error; initial_error <= max_initial_error; initial_error += initial_error_stepsize) {
    std::cout<<"Finished "<<(initial_error-min_initial_error)*100.0/(max_initial_error-min_initial_error)<<"%"<<std::endl;
    for(int i_initial_error= 0; i_initial_error < num_iterations_per_initial_error; ++i_initial_error) {
      float sample_resolution = 0.01213124;
      std::string sample_type = "half_cylinder_hlaf_cuboid";
      float sample_size_x = 1.2 + scan_matcher_config.resolution * uniform_dist(e1) * 0.5;
      float sample_size_y = 1.2 + scan_matcher_config.resolution * uniform_dist(e1) * 0.5;
      float sample_size_z = 2.0 + scan_matcher_config.resolution * uniform_dist(e1) * 0.5;
      //generator.generateCuboid(pointcloud, sample_size_x, sample_size_y, sample_size_z);
      sample_size_y = sample_size_x;
      //generator.generateHalfCylinderHalfCube(pointcloud, sample_size_x, sample_size_y, sample_size_z);

      Eigen::Vector3f initial_error_unscaled({uniform_dist(e1),uniform_dist(e1),uniform_dist(e1)});
      Eigen::Vector3f initial_error_scaled = initial_error*initial_error_unscaled.normalized();
      float initial_error_x = initial_error_scaled[0];
      float initial_error_y = initial_error_scaled[1];
      float initial_error_z = initial_error_scaled[2];

      Eigen::Vector3f initial_rotation_axis_unscaled({uniform_dist(e1),uniform_dist(e1),uniform_dist(e1)});
      Eigen::Vector3f initial_rotation_axis = initial_rotation_axis_unscaled.normalized();
      float initial_error_angle = 0;//initial_error*M_PI;
      //const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d::Translation({initial_error_x,initial_error_y,initial_error_z});

      Eigen::Matrix<double, 3, 1> translation({initial_error_x,initial_error_y,initial_error_z});
      Eigen::Quaternion<double> orientation;
      orientation.w() = std::cos(initial_error_angle * 0.5);
      orientation.x() = std::sin(initial_error_angle * 0.5) * initial_rotation_axis[0];
      orientation.y() = std::sin(initial_error_angle * 0.5) * initial_rotation_axis[1];
      orientation.z() = std::sin(initial_error_angle * 0.5) * initial_rotation_axis[2];
      LOG(INFO)<<"quat "<<orientation.w()<<" "<<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z();
      const cartographer::transform::Rigid3d initial_pose_estimate = cartographer::transform::Rigid3d(translation, orientation);
      cartographer::transform::Rigid3d matched_pose_estimate;
      ceres::Solver::Summary summary;
      double time_map_update = 0.0;
      double time_scan_matching = 0.0;
      /*
        bool boundary_extrapolation = false;
        bool cubic_interpolation = false;*/
      for (bool boundary_extrapolation : { false, true }) {
        for (bool cubic_interpolation : { false, true }) {
          scan_matcher_config.boundary_extrapolation = boundary_extrapolation;
          scan_matcher_config.cubic_interpolation = cubic_interpolation;

          ProbabilityGridScanMatcher probability_grid_scan_matcher(nh, scan_matcher_config);
          probability_grid_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"ProbabilityGridScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";

          scan_matcher_config.multi_res_probability_grid = true;
          ProbabilityGridScanMatcher probability_grid_scan_matcher_multi_res(nh, scan_matcher_config);
          probability_grid_scan_matcher_multi_res.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"ProbabilityGridScanMatcherMultiRes"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";
          scan_matcher_config.multi_res_probability_grid = false;

          ChiselTSDFScanMatcher chisel_tsdf_scan_matcher(nh, scan_matcher_config);
          chisel_tsdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"ChiselTSDFScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";

          VoxbloxTSDFScanMatcher voxblox_tsdf_scan_matcher(nh, scan_matcher_config);
          voxblox_tsdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"VoxbloxTSDFScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";

          VoxbloxESDFScanMatcher voxblox_esdf_scan_matcher(nh, scan_matcher_config);
          voxblox_esdf_scan_matcher.evaluateScanMatcher(pointcloud, initial_pose_estimate, matched_pose_estimate, time_map_update, time_scan_matching, summary);
          myfile << std::setprecision (15)<<"VoxbloxESDFScanMatcher"<<","<<sample_resolution<<","
                 <<sample_type<<","<<sample_size_x<<","<<sample_size_y<<","<<sample_size_z<<","
                <<scan_matcher_config.resolution<<","<<scan_matcher_config.truncation_distance<<","<<scan_matcher_config.esdf_distance<<","
               <<initial_error_x<<","<<initial_error_y<<","<<initial_error_z<<","<<initial_error_angle<<","
              <<matched_pose_estimate.translation()[0]<<","<<matched_pose_estimate.translation()[1]<<","<<matched_pose_estimate.translation()[2]<<","<<matched_pose_estimate.rotation().angularDistance(Eigen::Quaterniond::Identity())
             <<","<<summary.num_successful_steps+summary.num_unsuccessful_steps<<","<<ceres::TerminationTypeToString(summary.termination_type)
            <<","<<time_map_update<<","<<time_scan_matching<<","<<scan_matcher_config.boundary_extrapolation<<","<<scan_matcher_config.cubic_interpolation
           <<","<<computeReprojectionError(pointcloud, matched_pose_estimate)<<"\n";
        }
      }
    }
  }


  myfile.close();
  std::cout<<"Finished benchmark"<<std::endl;
}

