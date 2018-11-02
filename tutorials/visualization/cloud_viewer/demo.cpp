/*************************************************************************
    > File Name: demo.cpp
    > Author: tangbf
    > Mail: 
    > Created Time: 2018年10月28日 星期日 10时38分13秒
 ************************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

using namespace std;
using namespace pcl;

int user_data;

void viewerOneOff(visualization::PCLVisualizer & viewer)
{
	viewer.setBackgroundColor(1,0, 0.5, 1.0);
	PointXYZ o;
	o.x = 1.0; o.y = 0; o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	cout << "i only run once" << endl;
}

void viewerPsycho(visualization::PCLVisualizer & viewer)
{
	static unsigned count = 0;
	stringstream ss;
	ss << "Once per viewer loop: " << count++;
	// viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 700, 300, "text", 0);
	user_data++;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;
	for(float x = -0.5f; x <= 0.5f; x += 0.01f)
	{
		for(float y = -0.5f; y <= 0.5f; y += 0.01f)
		{
			pcl::PointXYZ point;
			point.x = x; point.y = y; point.z = 0;
			point_cloud_ptr->points.push_back(point);
		}
	}
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;

	//cloud viewer
	pcl::visualization::CloudViewer viewer ("cv");
	viewer.showCloud (point_cloud_ptr);
	
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	viewer.runOnVisualizationThreadOnce(viewerPsycho);

	cin.get();

	while(!viewer.wasStopped())
	{
		
	}




}
