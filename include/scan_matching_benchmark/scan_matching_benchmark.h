#ifndef SCAN_MATCHING_BENCHMARK_H_
#define SCAN_MATCHING_BENCHMARK_H_

#include <cartographer/sensor/point_cloud.h>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <voxblox/core/tsdf_map.h>



class ScanMatchingBenchmark
{
public:
  ScanMatchingBenchmark(ros::NodeHandle &nh);

private:

};

class BatchScanMatchingBenchmark
{
public:
  BatchScanMatchingBenchmark(ros::NodeHandle &nh);

private:

};

#endif //SCAN_MATCHING_BENCHMARK_H_
