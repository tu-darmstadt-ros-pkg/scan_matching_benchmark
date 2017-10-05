#include "scan_matching_benchmark/test_set_generator.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <cartographer/sensor/voxel_filter.h>

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


void TestSetGenerator::generateCylinder(cartographer::sensor::PointCloud& cloud, float radius, float height) {
  std::random_device r;
  std::default_random_engine e1(r());
  std::normal_distribution<float> normal_distribution(0, 0.01);

  cloud.clear();
  float min_z = -height / 2.0;
  float max_z = height / 2.0;
  float angular_resolution = 2.0 * std::asin(resolution_ / (2 * radius));

  for(float x = - radius; x <= radius; x += resolution_) {
    for(float y = - radius; y <= radius; y += resolution_) {
      if(x*x+y*y < radius*radius)
      {
        float z = min_z;
        cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
        z = max_z;
        cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
      }
    }
  }

  for(float alpha = 0; alpha < 2.0 * M_PI; alpha += angular_resolution) {
    for(float z = min_z; z <= max_z; z += resolution_) {
      float x = std::cos(alpha) * radius;
      float y = std::sin(alpha) * radius;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }
}


void TestSetGenerator::generateHalfCylinderHalfCube(cartographer::sensor::PointCloud& cloud, float size_x, float size_y, float size_z) {

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

  //Half Cube x=>0
  for(float x = 0; x <= max_x; x += resolution_) {
    for(float y = min_y; y <= max_y; y += resolution_) {
      float z = min_z;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
      z = max_z;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }
  for(float x = 0; x <= max_x; x += resolution_) {
    for(float z = min_z; z <= max_z; z += resolution_) {
      float y = min_y;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
      y = max_y;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }

  for(float y = min_y; y <= max_y; y += resolution_) {
    for(float z = min_z; z <= max_z; z += resolution_) {
      float x = max_x;
      cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }


  //Half Cylinder x<=0
  float angular_resolution = 2.0 * std::asin(resolution_ / (size_x));
  float radius = 0.5 * size_x;

  for(float x = - radius; x < 0; x += resolution_) {
    for(float y = - radius; y <= radius; y += resolution_) {
      if(x*x+y*y < radius*radius)
      {
        float z = min_z;
        cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
        z = max_z;
        cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
      }
    }
  }

  for(float alpha = 0; alpha < 2.0 * M_PI; alpha += angular_resolution) {
    for(float z = min_z; z <= max_z; z += resolution_) {
      float x = std::cos(alpha) * radius;
      float y = std::sin(alpha) * radius;
      if(x < 0)
        cloud.emplace_back(Eigen::Vector3f(x + normal_distribution(e1), y + normal_distribution(e1), z + normal_distribution(e1)));
    }
  }

}

void loadPCD(cartographer::sensor::PointCloud& cloud, std::string dir)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (/*"/home/kevin/Downloads/gas_station_benchmark/423.043000000.pcd"*/dir, *pcl_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return ;
  }
  std::cout << "Loaded "
            << pcl_cloud->width * pcl_cloud->height
            << " data points from " << dir<< std::endl;
  for (size_t i = 0; i < pcl_cloud->points.size (); ++i) {
    cloud.push_back(Eigen::Vector3f({pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z}));
  }


}

void TestSetGenerator::loadPCDDir(cartographer::sensor::PointCloud& cloud, std::string dir) {
  cloud.clear();

  cartographer::sensor::PointCloud cloud_unfiltered;
  boost::filesystem::path p ("/home/kevin/Downloads/gas_station_benchmark/");

  try
  {
    if (boost::filesystem::exists(p))    // does p actually exist?
    {
      if (boost::filesystem::is_regular_file(p))        // is p a regular file?
        std::cout << p << " size is " << boost::filesystem::file_size(p) << '\n';

      else if (boost::filesystem::is_directory(p))      // is p a directory?
      {
        std::cout << "Reading files from " << p;
        for(boost::filesystem::directory_iterator it(p);
            it != boost::filesystem::directory_iterator();
            it++)
        {
          if (it->path().extension() == ".pcd") {
            loadPCD(cloud_unfiltered, it->path().string());
          }
        }
      }

      else
        std::cout << p << " exists, but is neither a regular file nor a directory\n";
    }
    else
      std::cout << p << " does not exist\n";
  }

  catch (const boost::filesystem::filesystem_error& ex)
  {
    std::cout << ex.what() << '\n';
  }

  cloud = cartographer::sensor::VoxelFiltered(cloud_unfiltered, 0.05);
  std::cout << "filter: " << cloud_unfiltered.size() <<" "<< cloud.size()<<std::endl;;



  // return (0);

}
