#include"kinect2_grabber.h"
#include <kinect.h>
#include<pcl\io\pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZRGBA PointType;
int main(void)
{
	int c = 0;
	int total = 0;//文件名
	//pcl::PointCloud<PointType> cloud_filtered;// 按范围过滤后的点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
	//显示窗口初始化
	m_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	m_viewer->setBackgroundColor(0, 0, 0);//设置背景颜色
	m_viewer->initCameraParameters();
	//m_viewer->addCoordinateSystem();//添加红绿蓝坐标轴
	m_viewer->createInteractor();
	// 获取Kinect设备	
	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr pointCloud_XYZRGBA;  //指针变量
	// Retrieved Point Cloud Callback Function 检测到的点云回调函数
	//boost::mutex mutex; //并发编程互斥锁
	//boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function = [&pointCloud_XYZRGBA, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr)
	//{ //只能用ConstPtr，不能用Ptr
	//	boost::mutex::scoped_lock lock(mutex);
	//	/* Point Cloud Processing */
	//	pointCloud_XYZRGBA = ptr->makeShared();
		//ptr->makeShared() = NULL;
	//};
	// Retrieved Point Cloud Callback Function 检测到的点云回调函数
	//It should be written in callback function that retrieve new point cloud.
	boost::mutex mutex; //并发编程互斥锁
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> pointcloud_function =  [&pointCloud_XYZRGBA, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr)
	{
	    //Save Point Cloud 
	    boost::mutex::scoped_lock lock(mutex);
		pointCloud_XYZRGBA = ptr->makeShared();
	};
	boost::shared_ptr<pcl::Grabber> grabberForKinect = boost::shared_ptr<pcl::Grabber>(new pcl::Kinect2Grabber);// Kinect2Grabber
	boost::signals2::connection connection = grabberForKinect->registerCallback(pointcloud_function); // Register Callback Function
	grabberForKinect->start();  //Start Grabber
	while (!m_viewer->wasStopped())
	{
		// Update Viewer
		c++;
		m_viewer->spinOnce(1); //调用交互程序并更新屏幕一次。
		boost::mutex::scoped_try_lock lock(mutex);
		//cloud_filtered = filterByScope(pointCloud_XYZRGBA, Z_nearby, Z_faraway, X_left, X_right, Y_bottom, Y_top);//截取对应范围图像
		if (lock.owns_lock() && pointCloud_XYZRGBA)
		{
			//cloud_filtered = filterByScope(pointCloud_XYZRGBA, Z_nearby, Z_faraway, X_left, X_right, Y_bottom, Y_top);
			//5 显示 Point Cloud
			if (!m_viewer->updatePointCloud(pointCloud_XYZRGBA->makeShared(), "cloud"))
			{//pointCloud_XYZRGBA是指针类型，不能用( . )运算符
				m_viewer->addPointCloud(pointCloud_XYZRGBA->makeShared(), "cloud");
			}
			if (c % 5 == 0)
			{//每隔一段时间保存一次图片
				//存储图片。
				char name[100];
				sprintf(name, "E:\\Kinect V2.0\\Kinect2.0-pcd\\final\\contradiction\\kinect_2\\%d.pcd", total++);
				pcl::io::savePCDFileASCII(name, *pointCloud_XYZRGBA);//以.pcd的格式保存点云数据到磁盘
			}
		}
	}//while
}