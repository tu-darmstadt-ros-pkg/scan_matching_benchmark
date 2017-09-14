#include "scan_matching_benchmark/test_set_generator.h"

TestSetGenerator::TestSetGenerator():
  resolution_(0.1) {}

TestSetGenerator::TestSetGenerator(float resolution):
  resolution_(resolution){}

void TestSetGenerator::generateCube(cartographer::sensor::PointCloud& cloud, float size) {
  cloud.clear();
  float min_xyz = -size / 2.0;
  float max_xyz = size / 2.0;

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

void TestSetGenerator::generateCuboid(cartographer::sensor::PointCloud& cloud, float size_x, float size_y, float size_z) {
  std::random_device r;
  std::default_random_engine e1(r());
  std::normal_distribution<float> normal_distribution(0, 0.01);

  cloud.clear();
  float min_x = -size_x / 2.0;
  float max_x = size_x / 2.0;
  float min_y = -size_y / 2.0;
  float max_y = size_y / 2.0;
  float min_z = -size_z / 2.0;
  float max_z = size_z / 2.0;

  for(float x = min_x; x <= max_x; x += resolution_) {
    for(float y = min_y; y <= max_y; y += resolution_) {
      float z = min_z;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
      z = max_z;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }
  for(float x = min_x; x <= max_x; x += resolution_) {
    for(float z = min_z; z <= max_z; z += resolution_) {
      float y = min_y;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
      y = max_y;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }

  for(float y = min_y; y <= max_y; y += resolution_) {
    for(float z = min_z; z <= max_z; z += resolution_) {
      float x = min_x;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
      x = max_x;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }
}
