//
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>
//int main()
//{
//	std::string filename = "D:\\AllProject\\c++\\PCL_demo\\x64\\Release\\result.pcd";
//	//std::string filename = argv[1];
//	//std::cout << "Reading " << filename << std::endl;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	if (pcl::io::loadPCDFile <pcl::PointXYZ>(filename.c_str(), *cloud) == -1)
//		// load the file
//	{
//		PCL_ERROR("Couldn't read file");
//		return (-1);
//	}
//	std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;
//	//����һ������cloud
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	//pcl::io::loadPCDFile("D:\\AllProject\\c++\\rabbit.pcb", *cloud);//�����������
//
//	// Normal estimation*
//	//�������
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	//����kdtree�����н��ڵ㼯����
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	//Ϊkdtree��ӵ�������
//	tree->setInputCloud(cloud);
//
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	//���Ʒ������ʱ����Ҫ�����Ľ��ڵ��С
//	n.setKSearch(20);
//	//��ʼ���з������
//	n.compute(*normals);
//	//* normals should not contain the point normals + surface curvatures
//
//	//��ʾ��
//	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
//
//	//���ñ���ɫ
//	viewer.setBackgroundColor(0, 0, 0);
//
//	//����zֵ������Ⱦ�����ɫ
//	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
//
//	//�����Ҫ��ʾ�ĵ�������
//	viewer.addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
//	//���õ���ʾ��С
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//
//	//�����Ҫ��ʾ�ĵ��Ʒ���cloudΪԭʼ����ģ�ͣ�normalΪ������Ϣ��1��ʾ��Ҫ��ʾ����ĵ��Ƽ������ÿ1������ʾһ�η���0.01��ʾ���򳤶ȡ�
//	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 1, "normals");
//
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//}