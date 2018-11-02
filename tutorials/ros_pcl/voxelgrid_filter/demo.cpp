/*************************************************************************
    > File Name: demo.cpp
    > Author: tangbf
    > Mail: 
    > Created Time: 2018年10月29日 星期一 09时56分00秒
 ************************************************************************/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	pcl::PCDReader reader;
	reader.read("../table_scene_lms400.pcd", *cloud);

	cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data point (" << pcl::getFieldsList (*cloud) << ")." << endl;
	//visualization::CloudViewer viewer("show");
	//viewer.showCloud(cloud);
	
	//create the filtering object
	VoxelGrid<PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered);
	
	cerr << "PointCloud after filtering " << cloud_filtered->width * cloud_filtered->height << " data points (" << getFieldsList (*cloud_filtered) << ")." << endl;

	PCDWriter writer;
	writer.write("../table_scene_lms400_downsampled.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
	return (0);
	
}
