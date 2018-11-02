/*************************************************************************
    > File Name: demo.cpp
    > Author: tangbf
    > Mail: 
    > Created Time: 2018年10月31日 星期三 10时07分49秒
 ************************************************************************/
//Source:http://pointclouds.org/documentation/tutorials/greedy_projection.php

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_io.h>

#include <pcl/console/time.h>


using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	PCLPointCloud2 cloud_blob;
	io::loadPCDFile("../bun0.pcd", cloud_blob);
	fromPCLPointCloud2(cloud_blob, *cloud);

	visualization::CloudViewer viewer ("cloud viewer");
	viewer.showCloud(cloud);
	cin.get();

	NormalEstimation<PointXYZ, Normal> n;
	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
	search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	concatenateFields(*cloud, *normals, *cloud_with_normals);

	viewer.showCloud(cloud);
	cin.get();

	search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	GreedyProjectionTriangulation<PointNormal> gp3;
	PolygonMesh triangles;

	gp3.setSearchRadius(0.025);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI/4);
	gp3.setMinimumAngle(M_PI/18);
	gp3.setMaximumAngle(2*M_PI/3);
	gp3.setNormalConsistency(false);
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	vector<int> parts = gp3.getPartIDs();
	vector<int> states = gp3.getPointStates();


	pcl::console::TicToc tt;
	tt.tic();

	cout << "aaaa" << endl;

	tt.toc();
	tt.toc_print();
	

	io::saveVTKFile("../mesh.vtk", triangles);
	//viewer.showCloud(triangles);
	//cin.get();

	return 0;



}
