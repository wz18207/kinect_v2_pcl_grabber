#include"kinect2_grabber.h"
#include <kinect.h>
#include<pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZRGBA PointType;
int main(void) {
	int c = 0;
	int total = 0;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
	m_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	m_viewer->setBackgroundColor(0, 0, 0);
	m_viewer->initCameraParameters();
	m_viewer->createInteractor();
	pcl::PointCloud<PointType>::ConstPtr pointCloud_XYZRGBA;
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> pointcloud_function = [&pointCloud_XYZRGBA, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {

		boost::mutex::scoped_lock lock(mutex);
		pointCloud_XYZRGBA = ptr->makeShared();
	};
	boost::shared_ptr<pcl::Grabber> grabberForKinect = boost::shared_ptr<pcl::Grabber>(new pcl::Kinect2Grabber);
	boost::signals2::connection connection = grabberForKinect->registerCallback(pointcloud_function);
	grabberForKinect->start();
	while (!m_viewer->wasStopped()) {
		c++;
		m_viewer->spinOnce(1);
		boost::mutex::scoped_try_lock lock(mutex);
		if (lock.owns_lock() && pointCloud_XYZRGBA) {
			if (!m_viewer->updatePointCloud(pointCloud_XYZRGBA->makeShared(), "cloud")) {
				m_viewer->addPointCloud(pointCloud_XYZRGBA->makeShared(), "cloud");
			}
			if (c % 5 == 0) {
				char name[100];
				sprintf(name, "%d.pcd", total++);
				pcl::io::savePCDFileASCII(name, *pointCloud_XYZRGBA);
			}
		}
	}
}
