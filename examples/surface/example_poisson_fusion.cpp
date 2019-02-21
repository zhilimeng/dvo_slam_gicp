#include <dvo/surface/poisson_reconstruction.h>
#include <dvo/core/pointcloud_io.h>
#include <dvo/core/rgbd_image.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <iostream>
#include <string>

using namespace dvo;

void onProgressBarCallBack(const PoissonReconstruction &poisson, const int &value)
{
	std::cout << "progress value: " << value << std::endl;
}

int main(int argc,char* argv[])
{
	std::string infile = "D:/testData/slam_data/tsdf_fusion.asc";
	std::string outfile = "D:/testData/slam_data/poisson_tsdf.obj";
	PoissonReconstruction poisson;
	/* * \brief for class member function: boost::bind(&Class::onProgressBarCallBack,this,_1,_2) */
	poisson.addProgressBarCallback(boost::bind(onProgressBarCallBack, _1, _2));
	poisson.setWithInitCompute(true);
	poisson.setStddevMulThresh(1.5);
	poisson.setKNeighbor(120);
	poisson.setSharpnessAngle(30);
	poisson.setDepth(10);
	poisson.setDegree(3);
	poisson.setPointWeight(1.0);
	poisson.setSamplesPerNode(12.0);
	poisson.setScaleFactor(1.1);
	poisson.setHoleFillScale(1);
	poisson.setSmoothIterations(2);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::cout << "load points...\n";
	loadPointXYZNormal(infile, *cloud);
	std::cout << "load point cloud size: " << cloud->size() << std::endl;
	poisson.setInputCloud(cloud);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	pcl::io::saveOBJFile(outfile, mesh, 8);

}
