////#include<iostream>
//#include<pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include<pcl/point_types.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/obj_io.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//int main(int argc, char** argv) {
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PCLPointCloud2 cloud_blob;
//	//*打开点云文件
//	if (pcl::io::loadPCDFile("cat.pcd", cloud_blob) == -1) {
//		PCL_ERROR("Couldn't read file rabbit.pcd\n");
//		return(-1);
//	}
//	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
//
//	//法线估计对象
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	//存储估计的法线
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	//定义kd树指针
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	//估计法线存储到其中
//	n.compute(*normals);//Concatenate the XYZ and normal fields*
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_width_normals(new pcl::PointCloud<pcl::PointNormal>);
//	//链接字段
//	pcl::concatenateFields(*cloud, *normals, *cloud_width_normals);
//
//	//定义搜索树对象
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	//点云构建搜索树
//	tree2->setInputCloud(cloud_width_normals);
//
//	//定义三角化对象
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//	//存储最终三角化的网络模型
//	pcl::PolygonMesh triangles;//设置连接点之间的最大距离，（即是三角形最大边长）
//	gp3.setSearchRadius(200.0f);
//	//设置各种参数值
//	gp3.setMu(2.5f);
//	gp3.setMaximumNearestNeighbors(100);
//	gp3.setMaximumSurfaceAngle(M_PI_4);
//	gp3.setMinimumAngle(M_PI / 18);
//	gp3.setMaximumAngle(2 * M_PI / 3);
//	gp3.setNormalConsistency(false);
//
//	//设置搜索方法和输入点云
//	gp3.setInputCloud(cloud_width_normals);
//	gp3.setSearchMethod(tree2);
//
//	//执行重构，结果保存在triangles中
//	gp3.reconstruct(triangles);
//
//	//保存网格图  
//	//pcl::io::saveOBJFile("result.obj", triangles);
//	std::string output_dir = "greedy_cat.ply";
//	std::string sav = "saved mesh in:";
//	sav += output_dir;
//	pcl::console::print_info(sav.c_str());
//	std::cout << std::endl;
//
//	pcl::io::savePLYFileBinary(output_dir.c_str(), triangles);
//
//	// 显示结果图  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("MAP3D MESH"));
//	////设置背景;
//	viewer->setBackgroundColor(0, 0, 0);
//	//设置显示的网格
//	viewer->addPolygonMesh(triangles, "my");
//	//viewer->initCameraParameters();
//	while (!viewer->wasStopped()) {
//		viewer->spin();
//	}
//	std::cout << "success" << std::endl;
//	return 0;
//}

//#include <iostream>
//#include <pcl/common/common.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/point_types.h>
//#include <pcl/surface/mls.h>
//#include <pcl/surface/poisson.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//
//
//using namespace pcl;
//using namespace std;
//
//int
//main(int argc, char** argv)
//{
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PCLPointCloud2 cloud_blob;
//	//*打开点云文件
//	if (pcl::io::loadPCDFile("cat.pcd", cloud_blob) == -1) {
//		PCL_ERROR("Couldn't read file rabbit.pcd\n");
//		return(-1);
//	}
//	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
//	
//
//	/*滤波阶段*/
//	PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
//	PassThrough<PointXYZ> filter;
//	filter.setInputCloud(cloud);
//	filter.filter(*filtered);
//	cout << "低通滤波　　　完成" << endl;
//
//	/*法向计算阶段*/
//	NormalEstimationOMP<PointXYZ, Normal> ne;
//	ne.setNumberOfThreads(8);
//	ne.setInputCloud(filtered);
//	ne.setRadiusSearch(5);
//	Eigen::Vector4f centroid;
//	compute3DCentroid(*filtered, centroid);
//	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
//
//	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
//	ne.compute(*cloud_normals);
//
//	for (size_t i = 0; i < cloud_normals->size(); ++i) {
//		cloud_normals->points[i].normal_x *= -1;
//		cloud_normals->points[i].normal_y *= -1;
//		cloud_normals->points[i].normal_z *= -1;
//	}
//
//
//	PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
//	//将点云数据的坐标和法向信息拼接
//	concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
//
//	cout << "法向计算　　　完成" << endl;
//
//
//
//	/*poission 重建阶段*/
//	//创建poisson重建对象
//	Poisson<PointNormal> poisson;
//	// poisson.setDepth(9);
//	//输入poisson重建点云数据
//	poisson.setInputCloud(cloud_smoothed_normals);
//	//创建网格对象指针，用于存储重建结果
//	PolygonMesh mesh;
//	//poisson重建开始
//	poisson.reconstruct(mesh);
//
//	//将重建结果存储到硬盘，并保存为PLY格式
//	io::savePLYFile("possion_cat.ply", mesh);
//	cout << "曲面重建　　　完成" << endl;
//
//
//	/*图形显示阶段*/
//	cout << "开始图形显示......" << endl;
//	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("my viewer"));
//
//	viewer->setBackgroundColor(0, 0, 7);
//	viewer->addPolygonMesh(mesh, "my");
//	viewer->addCoordinateSystem(50.0);
//	viewer->initCameraParameters();
//
//	while (!viewer->wasStopped()) {
//
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//
//	return (0);
//}