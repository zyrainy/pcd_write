//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/marching_cubes_hoppe.h>
//#include <pcl/surface/marching_cubes_rbf.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <fstream>
//#include <iostream>
//#include <stdio.h>
//#include <string.h>
//#include <string>
//
//int main(int argc, char** argv)
//{
//	
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PCLPointCloud2 cloud_blob;
//	//*打开点云文件
//	if (pcl::io::loadPCDFile("rabbit.pcd", cloud_blob) == -1) {
//		PCL_ERROR("Couldn't read file \n");
//		return(-1);
//	}
//	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
//
//		std::cout << "Loaded "
//		<< cloud->size()
//		<< " data points from test_file.pcd with the following fields: "
//		<< std::endl;
//	// 估计法向量
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals); //计算法线，结果存储在normals中
//	//* normals 不能同时包含点的法向量和表面的曲率
//	cout << "法线计算完成...." << endl;
//	//将点云和法线放到一起
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//* cloud_with_normals = cloud + normals
//
//
//	//创建搜索树
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//	cout << "开始初始化RBF..." << endl;
//
//	pcl::MarchingCubes<pcl::PointNormal> *mc;
//	mc = new pcl::MarchingCubesRBF<pcl::PointNormal>();
//
//	//创建多变形网格，用于存储结果
//	pcl::PolygonMesh mesh;
//
//	//设置MarchingCubes对象的参数
//	mc->setIsoLevel(0.0f);
//	mc->setGridResolution(20, 20, 20);
//	mc->setPercentageExtendGrid(0.0f);
//
//	//设置搜索方法
//	mc->setInputCloud(cloud_with_normals);
//
//	//执行重构，结果保存在mesh中
//	cout << "开始网格重构..." << endl;
//	mc->reconstruct(mesh);
//	cout << "网格重构成功..." << endl;
//	//保存网格图
//	pcl::io::savePLYFile("rbf_rabbit.ply", mesh);
//
//	// 显示结果图
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0); //设置背景
//	viewer->addPolygonMesh(mesh, "my"); //设置显示的网格
//	//viewer->addCoordinateSystem(1.0); //设置坐标系
//	viewer->initCameraParameters();
//	while (!viewer->wasStopped()) {
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//	return (0);
//}

#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFilePLY("rbf_file.ply", mesh);

	// 显示结果图
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //设置背景
	viewer->addPolygonMesh(mesh, "my"); //设置显示的网格
	viewer->addCoordinateSystem(1.0); //设置坐标系
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	return (0);
}
