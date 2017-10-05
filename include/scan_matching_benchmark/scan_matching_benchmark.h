#ifndef SCAN_MATCHING_BENCHMARK_H_
#define SCAN_MATCHING_BENCHMARK_H_

#include <cartographer/sensor/point_cloud.h>
#include <pcl/common/common.h>
#include <ros/ros.h>

#include <ctime>
#include <chrono>


float computeReprojectionError(const cartographer::sensor::PointCloud& scan_cloud,
                               const cartographer::transform::Rigid3d& pose_estimate);

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

class BatchScanMatchingBenchmarkFromFile
{
public:
  BatchScanMatchingBenchmarkFromFile(ros::NodeHandle &nh, std::string dir);

private:

};

#endif //SCAN_MATCHING_BENCHMARK_H_
