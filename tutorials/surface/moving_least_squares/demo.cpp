/*************************************************************************
    > File Name: demo.cpp
    > Author: tangbf
    > Mail: 
    > Created Time: 2018年10月31日 星期三 11时25分46秒
 ************************************************************************/

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	io::loadPCDFile("../bun0.pcd", *cloud);

	visualization::CloudViewer viewer ("cloud viewer");
	viewer.showCloud(cloud);
	cin.get();

	search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
	PointCloud<PointNormal> mls_points;

	MovingLeastSquares<PointXYZ, PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	mls.process(mls_points);

	io::savePCDFile("../bun0-mls.pcd", mls_points);



}
