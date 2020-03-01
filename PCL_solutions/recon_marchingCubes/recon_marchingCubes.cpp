#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

int main (int argc, char** argv) {
  // ȷ���ļ���ʽ
  char tmpStr[100];
  strcpy(tmpStr,argv[1]);
  char* pext = strrchr(tmpStr, '.');
  std::string extply("ply");
  std::string extpcd("pcd");
  if(pext) {
    *pext='\0';
    pext++;
  }
  std::string ext(pext);
  //�����֧���ļ���ʽ���˳�����
  if (!((ext == extply)||(ext == extpcd))) {
    std::cout << "�ļ���ʽ��֧��!" << std::endl;
    std::cout << "֧���ļ���ʽ��*.pcd��*.ply��" << std::endl;
    return(-1);
  }

  //�����ļ���ʽѡ�����뷽ʽ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>) ; //�������ƶ���ָ�룬���ڴ洢����
  if (ext == extply) {
    if (pcl::io::loadPLYFile(argv[1] , *cloud) == -1) {
      PCL_ERROR("Could not read ply file!\n") ;
      return -1;
    }
  } else {
    if (pcl::io::loadPCDFile(argv[1] , *cloud) == -1) {
      PCL_ERROR("Could not read pcd file!\n") ;
      return -1;
    }
  }

  // ���Ʒ�����
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute (*normals); //���㷨�ߣ�����洢��normals��
  //* normals ����ͬʱ������ķ������ͱ��������

  //�����ƺͷ��߷ŵ�һ��
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals


  //����������
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  //��ʼ��MarchingCubes���󣬲����ò���
  pcl::MarchingCubes<pcl::PointNormal> *mc;
  //mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
  int hoppe_or_rbf = 1;
  if (hoppe_or_rbf == 0)
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
  else {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    float off_surface_displacement = 0.0f;
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
  }


  //����������������ڴ洢���
  pcl::PolygonMesh mesh;

  //����MarchingCubes����Ĳ���
  mc->setIsoLevel (0.0f);
  mc->setGridResolution (60,60,60);
  mc->setPercentageExtendGrid (0.0f);
  //������������
  mc->setInputCloud (cloud_with_normals);

  //ִ���ع������������mesh��
  mc->reconstruct (mesh);

  //��������ͼ
  pcl::io::savePLYFile("result.ply", mesh);

  // ��ʾ���ͼ
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0); //���ñ���
  viewer->addPolygonMesh(mesh,"my"); //������ʾ������
  viewer->addCoordinateSystem (1.0); //��������ϵ
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
