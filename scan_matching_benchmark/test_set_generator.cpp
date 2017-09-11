#include "scan_matching_benchmark/test_set_generator.h"

TestSetGenerator::TestSetGenerator():
  resolution_(0.1),
  size_(10.0) {}

TestSetGenerator::TestSetGenerator(float resolution, float size):
  resolution_(resolution),
  size_(size) {}

void TestSetGenerator::generateCube(cartographer::sensor::PointCloud& cloud) {
  cloud.clear();
  float min_xyz = -size_ / 2.0;
  float max_xyz = size_ / 2.0;

  for(float x = min_xyz; x <= max_xyz; x += resolution_) {
    for(float y = min_xyz; y <= max_xyz; y += resolution_) {
      float z = min_xyz;
      cloud.emplace_back(Eigen::Vector3f(x, y, z));
      z = max_xyz;
      cloud.emplace_back(Eigen::Vector3f(x, y, z));
    }
  }
  for(float x = min_xyz; x <= max_xyz; x += resolution_) {
    for(float z = min_xyz; z <= max_xyz; z += resolution_) {
      float y = min_xyz;
      cloud.emplace_back(Eigen::Vector3f(x, y, z));
      y = max_xyz;
      cloud.emplace_back(Eigen::Vector3f(x, y, z));
    }
  }

  for(float y = min_xyz; y <= max_xyz; y += resolution_) {
    for(float z = min_xyz; z <= max_xyz; z += resolution_) {
      float x = min_xyz;
      cloud.emplace_back(Eigen::Vector3f(x, y, z));
      x = max_xyz;
      cloud.emplace_back(Eigen::Vector3f(x, y, z));
    }
  }
}
