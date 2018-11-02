/*************************************************************************
    > File Name: demo.cpp
    > Author: tangbf
    > Mail: 
    > Created Time: 2018年10月31日 星期三 14时12分45秒
 ************************************************************************/

#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	if(argc < 5)
	{
		cout << "Usage: " << "<Resolution>" << "<SearchPoint.x>" << "<SearchPoint.y>" << "<SearchPoint.z>" << endl;
		return -2;
	}
	else
		cout << "Resolution: " << argv[1] << endl;
	
	cout << "first parameter: " << argv[0] << endl;

	PointCloud<PointXYZ>::Ptr pCubeCloud(new PointCloud<PointXYZ>);
	for(int x = 0; x < 100; ++x)
		for(int y = 0; y < 100; ++y)
			for(int z = 0; z < 100; ++z)
				pCubeCloud->push_back(PointXYZ((float)x/100.0f, (float)y/100.0f, (float)z/100.0f));
	io::savePCDFile("../cube.pcd", *pCubeCloud, true);

	// visualization::CloudViewer viewer1 ("Cloud Viewer");
	// viewer1.showCloud(pCubeCloud);
	// cin.get();

	PointCloud<PointXYZ>::Ptr pCube (new PointCloud<PointXYZ>);
	if(io::loadPCDFile("../cube.pcd", *pCube) == -1)
		return -1;

	float resolution = atof(argv[1]);
	octree::OctreePointCloudSearch<PointXYZ> octree (resolution);
//	octree.defineBoundingBox(0, 0, 0, 1, 1, 1);
	octree.setInputCloud(pCube);
	octree.addPointsFromInputCloud();

	PointXYZ searchPoint (atof(argv[2]), atof(argv[3]), atof(argv[3]));

	std::vector<int> pointIdxVec;

	if(octree.voxelSearch(searchPoint, pointIdxVec))
	{
		cout << "Leaf Count: " << octree.getLeafCount() << endl;
		cout << "Tree Depth: " << octree.getTreeDepth() << endl;
		cout << "Branch Count: " << octree.getBranchCount() << endl;
		cout << "Voxel Diameter: " << octree.getVoxelSquaredDiameter() << endl;
		cout << "Voxel Side Length: " << octree.getVoxelSquaredSideLen() << endl;
		double minx, miny, minz, maxx, maxy, maxz;
		octree.getBoundingBox(minx, miny, minz, maxx, maxy, maxz);
		cout << "BoundingBox: " << "(" << minx << "-" << maxx << ")" << ", " << "(" << miny << "-" << maxy <<")" << ", " << "(" << minz << "-" << maxz << ")" << endl;
	}
	
	PointCloud<RGB>::Ptr pPointsRGB (new PointCloud<RGB>);
	pPointsRGB->width = pCube->size();
	pPointsRGB->height = 1;
	pPointsRGB->resize(pPointsRGB->width * pPointsRGB->height);

	for(size_t i = 0; i < pPointsRGB->size(); ++i)
		pPointsRGB->points[i].b = 255;
	for(size_t i = 0; i < pointIdxVec.size(); ++i)
	{
		pPointsRGB->points[pointIdxVec[i]].b = 0;
		pPointsRGB->points[pointIdxVec[i]].r = 255;
	}	

	PointCloud<PointXYZRGBA>::Ptr pCloudShow (new PointCloud<PointXYZRGBA>);
	pCloudShow->width = pCube->size();
	pCloudShow->height = 1;
	pCloudShow->resize(pPointsRGB->width * pPointsRGB->height);

	concatenateFields(*pCube, *pPointsRGB, *pCloudShow);

	visualization::CloudViewer viewer1 ("Cloud Viewer");
	viewer1.showCloud(pCube);
	cin.get();



	visualization::PCLVisualizer viewer;
	viewer.setCameraFieldOfView(0.785398);
	viewer.setBackgroundColor(0.5, 0.5, 0.5);
	viewer.setCameraPosition(
			0, 0, 5,
			0, 0, -1,
			0, 1, 0
			);
	viewer.addPointCloud(pCloudShow, "Out");
	viewer.addCoordinateSystem(1.0, "cloud", 0);
	cin.get();
	while(!viewer.wasStopped())
		viewer.spinOnce();

	// system("pause");
	return 0; 




}
