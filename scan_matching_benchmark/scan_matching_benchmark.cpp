#include "include/scan_matching_benchmark/scan_matching_benchmark.h"

#include <cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_tsdf_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/ceres_voxblox_tsdf_scan_matcher.h>


ScanMatchingBenchmark::ScanMatchingBenchmark(ros::NodeHandle &nh)
{
  cartographer::mapping_3d::scan_matching::proto::CeresScanMatcherOptions options;
  cartographer::mapping_3d::scan_matching::CeresVoxbloxTSDFScanMatcher voxblox_scan_matcher(options);



}
