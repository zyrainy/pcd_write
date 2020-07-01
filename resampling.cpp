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
//	//*�򿪵����ļ�
//	if (pcl::io::loadPCDFile("cat.pcd", cloud_blob) == -1) {
//		PCL_ERROR("Couldn't read file rabbit.pcd\n");
//		return(-1);
//	}
//	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
//
//	//���߹��ƶ���
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	//�洢���Ƶķ���
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	//����kd��ָ��
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	//���Ʒ��ߴ洢������
//	n.compute(*normals);//Concatenate the XYZ and normal fields*
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_width_normals(new pcl::PointCloud<pcl::PointNormal>);
//	//�����ֶ�
//	pcl::concatenateFields(*cloud, *normals, *cloud_width_normals);
//
//	//��������������
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	//���ƹ���������
//	tree2->setInputCloud(cloud_width_normals);
//
//	//�������ǻ�����
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//	//�洢�������ǻ�������ģ��
//	pcl::PolygonMesh triangles;//�������ӵ�֮��������룬���������������߳���
//	gp3.setSearchRadius(200.0f);
//	//���ø��ֲ���ֵ
//	gp3.setMu(2.5f);
//	gp3.setMaximumNearestNeighbors(100);
//	gp3.setMaximumSurfaceAngle(M_PI_4);
//	gp3.setMinimumAngle(M_PI / 18);
//	gp3.setMaximumAngle(2 * M_PI / 3);
//	gp3.setNormalConsistency(false);
//
//	//���������������������
//	gp3.setInputCloud(cloud_width_normals);
//	gp3.setSearchMethod(tree2);
//
//	//ִ���ع������������triangles��
//	gp3.reconstruct(triangles);
//
//	//��������ͼ  
//	//pcl::io::saveOBJFile("result.obj", triangles);
//	std::string output_dir = "greedy_cat.ply";
//	std::string sav = "saved mesh in:";
//	sav += output_dir;
//	pcl::console::print_info(sav.c_str());
//	std::cout << std::endl;
//
//	pcl::io::savePLYFileBinary(output_dir.c_str(), triangles);
//
//	// ��ʾ���ͼ  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("MAP3D MESH"));
//	////���ñ���;
//	viewer->setBackgroundColor(0, 0, 0);
//	//������ʾ������
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
//	//*�򿪵����ļ�
//	if (pcl::io::loadPCDFile("cat.pcd", cloud_blob) == -1) {
//		PCL_ERROR("Couldn't read file rabbit.pcd\n");
//		return(-1);
//	}
//	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
//	
//
//	/*�˲��׶�*/
//	PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
//	PassThrough<PointXYZ> filter;
//	filter.setInputCloud(cloud);
//	filter.filter(*filtered);
//	cout << "��ͨ�˲����������" << endl;
//
//	/*�������׶�*/
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
//	//���������ݵ�����ͷ�����Ϣƴ��
//	concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
//
//	cout << "������㡡�������" << endl;
//
//
//
//	/*poission �ؽ��׶�*/
//	//����poisson�ؽ�����
//	Poisson<PointNormal> poisson;
//	// poisson.setDepth(9);
//	//����poisson�ؽ���������
//	poisson.setInputCloud(cloud_smoothed_normals);
//	//�����������ָ�룬���ڴ洢�ؽ����
//	PolygonMesh mesh;
//	//poisson�ؽ���ʼ
//	poisson.reconstruct(mesh);
//
//	//���ؽ�����洢��Ӳ�̣�������ΪPLY��ʽ
//	io::savePLYFile("possion_cat.ply", mesh);
//	cout << "�����ؽ����������" << endl;
//
//
//	/*ͼ����ʾ�׶�*/
//	cout << "��ʼͼ����ʾ......" << endl;
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