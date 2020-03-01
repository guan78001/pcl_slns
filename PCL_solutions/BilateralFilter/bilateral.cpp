#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/io/ply_io.h>
#include <omp.h>
using namespace std;

void fill_pointCloud(pcl::PointCloud<pcl::PointXYZ>&cloud) {
  // Fill in the cloud data
  cloud.width = 120;
  cloud.height = 80;
  cloud.is_dense = true;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    int x = i % cloud.width;
    int y = i / cloud.height;
    cloud.points[i].x = x * 0.1f;
    cloud.points[i].y = y * 0.15f;
    cloud.points[i].z = 7.0 + (rand() % 10 - 5) * 0.01;

    //double val = 12 * 12 - x * 0.1f * x * 0.1f - y * y * 0.15f * 0.15f;
    //if (val <= 0) {
    //  cloud.points[i].z = 12;
    //} else cloud.points[i].z = sqrt(val); // 7.0f * rand() / (RAND_MAX + 1.0f);
  }

}

int
main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  fill_pointCloud(*cloud);
  pcl::io::savePLYFile("plane_mesh.ply", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;



  // Create the filtering object
  //pcl::BilateralFilter<pcl::PointXYZ> sor;
  //sor.setInputCloud(cloud);
  //sor.setHalfSize(0.1);
  //sor.setStdDev(1);

  pcl::FastBilateralFilter<pcl::PointXYZ> sor;
  double t0 = omp_get_wtime();
  sor.setInputCloud (cloud);
  sor.setSigmaS(0.1);
  sor.setSigmaR(1);
  sor.applyFilter(*cloud_filtered);
  //sor.filter (*cloud_filtered);
  double t1 = omp_get_wtime();
  cout << "used time:" << t1 - t0 << " seconds" << endl;
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::io::savePLYFile("plane_mesh_FastBilateralFilter.ply", *cloud_filtered);

  return (0);
}