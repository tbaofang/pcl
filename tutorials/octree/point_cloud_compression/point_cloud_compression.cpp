/*************************************************************************
    > File Name: point_cloud_compression.cpp
    > Author: tangbf
    > Mail: 
    > Created Time: 2018年10月27日 星期六 16时53分16秒
 ************************************************************************/


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <iostream>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>

using namespace std;

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer() : viewer ("Point Cloud Compression Example")
	{
	}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		if(!viewer.wasStopped())
		{
			stringstream compressedData;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGBA> ());
			PointCloudEncoder->encodePointCloud(cloud, compressedData);
			PointCloudDecoder->decodePointCloud(compressedData, cloudOut);
			viewer.showCloud(cloudOut);
		}
	}

	void run()
	{
		bool showStatistics = true;
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> ();
		pcl::Grabber* interface = new pcl::OpenNIGrabber ();
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
		boost::signals2::connection c = interface->registerCallback (f);
		interface->start();
		while(!viewer.wasStopped())
		{
			sleep(1);
		}
		interface->stop();
		delete(PointCloudEncoder);
		delete(PointCloudDecoder);
		
	}

	pcl::visualization::CloudViewer viewer;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
};

int main(int argc, char** argv)
{
	SimpleOpenNIViewer v;
	v.run();

	return 0;
}
