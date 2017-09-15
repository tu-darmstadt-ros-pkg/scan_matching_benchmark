#ifndef CHISEL_TSDF_SCAN_MATCHER_H_
#define CHISEL_TSDF_SCAN_MATCHER_H_

#include "scan_matcher.h"

class ChiselTSDFScanMatcher: public ScanMatcher {
public:
  ChiselTSDFScanMatcher(ros::NodeHandle &nh, ScanMatcherConfig config);
  void evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                           const cartographer::transform::Rigid3d& initial_pose_estimate,
                           cartographer::transform::Rigid3d& matched_pose_estimate,
                           double& time_map_update, //seconds
                           double& time_scan_matching, //seconds
                           ceres::Solver::Summary& summary) override;
protected:
};

#endif //CHISEL_TSDF_SCAN_MATCHER_H_
