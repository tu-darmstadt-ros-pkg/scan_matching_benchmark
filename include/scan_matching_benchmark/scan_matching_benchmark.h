//=================================================================================================
// Copyright (c) 2017, Kevin Daun, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef SCAN_MATCHING_BENCHMARK_H_
#define SCAN_MATCHING_BENCHMARK_H_

#include <cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h>
#include <cartographer/sensor/point_cloud.h>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <voxblox/core/tsdf_map.h>



class ScanMatchingBenchmark
{
public:
  ScanMatchingBenchmark(ros::NodeHandle &nh);

private:

  void evaluateVoxbloxTSDFScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                      const cartographer::transform::Rigid3d& initial_pose_estimate,
                                      cartographer::transform::Rigid3d& matched_pose_estimate,
                                      pcl::PointCloud<pcl::PointXYZI>& voxblox_tsdf_cloud,
                                      pcl::PointCloud<pcl::PointXYZI>& interpolated_voxblox_tsdf_cloud,
                                      ceres::Solver::Summary& summary);

  void evaluateProbabilityGridScanMatcher(const cartographer::sensor::PointCloud& cloud,
                                          const cartographer::transform::Rigid3d& initial_pose_estimate,
                                          cartographer::transform::Rigid3d& matched_pose_estimate,
                                          pcl::PointCloud<pcl::PointXYZI>& probability_grid,
                                          pcl::PointCloud<pcl::PointXYZI>& interpolated_probability_grid_cloud,
                                          ceres::Solver::Summary& summary);

  cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions ceres_scan_matcher_options_;


  std::shared_ptr<voxblox::TsdfMap> voxblox_tsdf_;


};

#endif //SCAN_MATCHING_BENCHMARK_H_
