#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/ply_io.h>
#include <omp.h>
using namespace std;

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);
  pcl::io::savePLYFile("table_scene_lms400.ply", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  double t0 = omp_get_wtime();

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.1);
  outrem.setMinNeighborsInRadius (4);
  // apply filter
  outrem.filter (*cloud_filtered);

  double t1 = omp_get_wtime();
  cout << "used time:" << t1 - t0 << " seconds" << endl;
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
  pcl::io::savePLYFile("table_scene_lms400_inliers.ply", *cloud_filtered);
  return (0);
}

//int
//main (int argc, char** argv) {
//  if (argc != 2) {
//    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
//    exit(0);
//  }
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//
//  // Fill in the cloud data
//  cloud->width  = 50;
//  cloud->height = 10;
//  cloud->points.resize (cloud->width * cloud->height);
//
//  for (std::size_t i = 0; i < cloud->points.size (); ++i) {
//    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//  }
//  pcl::io::savePLYFile("input.ply", *cloud);
//  if (strcmp(argv[1], "-r") == 0) {
//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//    // build the filter
//    outrem.setInputCloud(cloud);
//    outrem.setRadiusSearch(0.8);
//    outrem.setMinNeighborsInRadius (2);
//    // apply filter
//    outrem.filter (*cloud_filtered);
//    pcl::io::savePLYFile("cloud_filtered_r.ply", *cloud_filtered);
//  } else if (strcmp(argv[1], "-c") == 0) {
//    // build the condition
//    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
//        pcl::ConditionAnd<pcl::PointXYZ> ());
//    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//                               pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
//    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//                               pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
//    // build the filter
//    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//    condrem.setCondition (range_cond);
//    condrem.setInputCloud (cloud);
//    condrem.setKeepOrganized(true);
//    // apply filter
//    condrem.filter (*cloud_filtered);
//    pcl::io::savePLYFile("cloud_filtered_c.ply", *cloud_filtered);
//  } else {
//    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
//    exit(0);
//  }
//  //std::cerr << "Cloud before filtering: " << std::endl;
//  //for (std::size_t i = 0; i < cloud->points.size (); ++i)
//  //  std::cerr << "    " << cloud->points[i].x << " "
//  //            << cloud->points[i].y << " "
//  //            << cloud->points[i].z << std::endl;
//  //// display pointcloud after filtering
//  //std::cerr << "Cloud after filtering: " << std::endl;
//  //for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
//  //  std::cerr << "    " << cloud_filtered->points[i].x << " "
//  //            << cloud_filtered->points[i].y << " "
//  //            << cloud_filtered->points[i].z << std::endl;
//  return (0);
//}