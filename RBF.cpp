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
//	//*�򿪵����ļ�
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
//	// ���Ʒ�����
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals); //���㷨�ߣ�����洢��normals��
//	//* normals ����ͬʱ������ķ������ͱ��������
//	cout << "���߼������...." << endl;
//	//�����ƺͷ��߷ŵ�һ��
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//* cloud_with_normals = cloud + normals
//
//
//	//����������
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//	cout << "��ʼ��ʼ��RBF..." << endl;
//
//	pcl::MarchingCubes<pcl::PointNormal> *mc;
//	mc = new pcl::MarchingCubesRBF<pcl::PointNormal>();
//
//	//����������������ڴ洢���
//	pcl::PolygonMesh mesh;
//
//	//����MarchingCubes����Ĳ���
//	mc->setIsoLevel(0.0f);
//	mc->setGridResolution(20, 20, 20);
//	mc->setPercentageExtendGrid(0.0f);
//
//	//������������
//	mc->setInputCloud(cloud_with_normals);
//
//	//ִ���ع������������mesh��
//	cout << "��ʼ�����ع�..." << endl;
//	mc->reconstruct(mesh);
//	cout << "�����ع��ɹ�..." << endl;
//	//��������ͼ
//	pcl::io::savePLYFile("rbf_rabbit.ply", mesh);
//
//	// ��ʾ���ͼ
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0); //���ñ���
//	viewer->addPolygonMesh(mesh, "my"); //������ʾ������
//	//viewer->addCoordinateSystem(1.0); //��������ϵ
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
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFilePLY("rbf_file.ply", mesh);

	// ��ʾ���ͼ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //���ñ���
	viewer->addPolygonMesh(mesh, "my"); //������ʾ������
	viewer->addCoordinateSystem(1.0); //��������ϵ
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	return (0);
}
