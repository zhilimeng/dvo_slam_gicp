#include <dvo/surface/poisson_reconstruction.h>
#include <dvo/core/pointcloud_io.h>
#include <dvo/core/rgbd_image.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <string>
using namespace boost::program_options;

using namespace dvo;

void onProgressBarCallBack(const PoissonReconstruction &poisson, const int &value)
{
	std::cout << "progress value: " << value << std::endl;
}

int main(int argc,char* argv[])
{
	options_description opts("poisson reconstruction options");

	opts.add_options()
		("help,h", "help info")
		("in,i", value<std::string>()->required(), "input file")
		("out,o", value<std::string>()->required(), "output file")
		("withInitCompute,c", value<bool>()->default_value(true), "with initial compute or not")
		("stdMul,u", value<float>()->default_value(1.5), "multiplier for the distance threshold calculation")
		("kNeighbors,k", value<int>()->default_value(120), "k nearest neighbors for filtering and bilateral smooth")
		("sharpnessAngle,g", value<float>()->default_value(30.0), "sharpness angle for bilateral smooth")
		("depth,d", value<int>()->default_value(10), "octree depth")
		("degree,b", value<int>()->default_value(3), "b spline degree")
		("pointWeight,w", value<float>()->default_value(0.0), "point weight")
		("samplesPerNode,n", value<float>()->default_value(12.0), "samples per node")
		("scaleFactor,f", value<float>()->default_value(1.05), "scale factor")
		("holeScale,t", value<float>()->default_value(1.0), "hole scale value")
		("smooth,s", value<int>()->default_value(1), "smooth iterations")
		("aRatio,a", value<float>()->default_value(0.0), "area island ratio");


	variables_map vm;
	store(parse_command_line(argc, argv, opts), vm);
	std::string infile, outfile;
	PoissonReconstruction poisson;
	/* * \brief for class member function: boost::bind(&Class::onProgressBarCallBack,this,_1,_2) */
	poisson.addProgressBarCallback(boost::bind(onProgressBarCallBack, _1, _2));
	if (vm.count("help"))
	{
		std::cout << opts << std::endl;
		return 0;
	}
	if (vm.count("in"))
	    infile = vm["in"].as<std::string>();
	else
	{
		std::cout << "error: in file is required\n";
		return -1;
	}
	if (vm.count("out"))
		outfile = vm["out"].as<std::string>();
	else
	{
		std::cout << "error: out file is required\n";
		return -1;
	}
	if (vm.count("withInitCompute"))
	{
		bool with_init = vm["withInitCompute"].as<bool>();
		poisson.setWithInitCompute(with_init);
	}
	if (vm.count("stdMul"))
	{
		float std_mul = vm["stdMul"].as<float>();
		poisson.setStddevMulThresh(std_mul);
	}
	if (vm.count("kNeighbors"))
	{
		int k = vm["kNeighbors"].as<int>();
		poisson.setKNeighbor(k);
	}
	if (vm.count("sharpnessAngle"))
	{
		float angle = vm["sharpnessAngle"].as<float>();
		poisson.setSharpnessAngle(angle);
	}
	if (vm.count("depth"))
	{
		int depth = vm["depth"].as<int>();
		poisson.setDepth(depth);
	}
	if (vm.count("degree"))
	{
		int degree = vm["degree"].as<int>();
		poisson.setDegree(degree);
	}
	if (vm.count("pointWeight"))
	{
		float pointWeight = vm["pointWeight"].as<float>();
		poisson.setPointWeight(pointWeight);
	}
	if (vm.count("samplesPerNode"))
	{
		float samplePerNode = vm["samplesPerNode"].as<float>();
		poisson.setSamplesPerNode(samplePerNode);
	}
	if (vm.count("scaleFactor"))
	{
		float scaleFactor = vm["scaleFactor"].as<float>();
		poisson.setScaleFactor(scaleFactor);
	}
	if (vm.count("holeScale"))
	{
		float trim = vm["holeScale"].as<float>();
		//		poisson.setTrimmingValue(trim);
		poisson.setHoleFillScale(trim);
	}
	if (vm.count("aRatio"))
	{
		float aRatio = vm["aRatio"].as<float>();
		poisson.setIslandAreaRatio(aRatio);
	}

	if (vm.count("smooth"))
	{
		int smooth = vm["smooth"].as<int>();
		poisson.setSmoothIterations(smooth);
	}
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::cout << "load points...\n";
	loadPointXYZNormal(infile, *cloud);
	std::cout << "load point cloud size: " << cloud->size() << std::endl;
	poisson.setInputCloud(cloud);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	pcl::io::saveOBJFile(outfile, mesh, 8);

}
