#ifndef VOXBLOX_ESDF_SCAN_MATCHER_H_
#define VOXBLOX_ESDF_SCAN_MATCHER_H_

#include "scan_matcher.h"

class VoxbloxESDFScanMatcher: public ScanMatcher {
public:
  VoxbloxESDFScanMatcher(ros::NodeHandle &nh, ScanMatcherConfig config);
  void evaluateScanMatcher(const cartographer::sensor::PointCloud& scan_cloud,
                           const cartographer::transform::Rigid3d& initial_pose_estimate,
                           cartographer::transform::Rigid3d& matched_pose_estimate,
                           ceres::Solver::Summary& summary) override;
protected:
};

#endif //VOXBLOX_ESDF_SCAN_MATCHER_H_
