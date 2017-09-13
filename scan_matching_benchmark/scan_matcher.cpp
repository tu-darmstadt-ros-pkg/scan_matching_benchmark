#include "scan_matcher.h"

#include <pcl_conversions/pcl_conversions.h>

void ScanMatcher::publishClouds() {
  sensor_msgs::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(map_cloud_, map_cloud_msg);
  map_cloud_msg.header.stamp = ros::Time::now();
  map_cloud_msg.header.frame_id = "world";
  map_pointcloud_publisher_.publish(map_cloud_msg);

  sensor_msgs::PointCloud2 interpolated_map_cloud_msg;
  pcl::toROSMsg(interpolated_map_cloud_, interpolated_map_cloud_msg);
  interpolated_map_cloud_msg.header.stamp = ros::Time::now();
  interpolated_map_cloud_msg.header.frame_id = "world";
  interpolated_map_pointcloud_publisher_.publish(interpolated_map_cloud_msg);
}
