#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void simple_pcd() {
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size(); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
}

void plane_pcd() {
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = 120;
  cloud.height = 80;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    int x = i%cloud.width;
    int y = i / cloud.height;
    cloud.points[i].x = x*0.1f;
    cloud.points[i].y = y*0.15f;
    cloud.points[i].z = sqrt(12 * 12 - x*0.1f*x*0.1f-y*y*0.15f*0.15f);// 7.0f * rand() / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII("D:/git_clone/pcl_slns/PCL_solutions/data/plane_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
}
int main (int argc, char** argv) {
  plane_pcd();
  return (0);
}
