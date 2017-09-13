#ifndef TEST_SET_GENERATOR_H_
#define TEST_SET_GENERATOR_H_

#include <cartographer/sensor/point_cloud.h>

class TestSetGenerator
{
public:
  TestSetGenerator();
  TestSetGenerator(float resolution);

  void generateCube(cartographer::sensor::PointCloud& cloud, float size);
  void generateCuboid(cartographer::sensor::PointCloud& cloud, float size_x, float size_y, float size_z);

  float resolution_;

private:


};

#endif //TEST_SET_GENERATOR_H_
