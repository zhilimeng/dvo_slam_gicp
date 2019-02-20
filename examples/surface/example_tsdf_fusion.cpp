#include <pcl/io/obj_io.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/frame.h>
#include <dvo/core/utility.h>
#include <dvo/core/pointcloud_io.h>
#include <dvo/surface/scalable_tsdf_volume.h>

using namespace dvo;
void readRGBDPairFiles(const std::string &folder,std::vector<std::string> &rgb_files,std::vector<std::string> &depth_files)
{
	std::ifstream in(folder + "/assoc.txt");
	double rgb_timestamp = 0.0;
	double depth_timestamp = 0.0;
	std::string rgb_file, depth_file;
	while (in.good() && !in.eof())
	{
		in >> rgb_timestamp >> rgb_file >> depth_timestamp >> depth_file;
		rgb_files.push_back(folder + "/" + rgb_file);
		depth_files.push_back(folder + "/" + depth_file);
	}
}
void test_tum_rgbd_tsdf(const std::string &folder)
{
	float fx = 1220.f;
	float fy = 1220.f;
	float cx = 240.f;
	float cy = 320.f;
	int width = 480;
	int height = 640;
	float depth_scale = 32000;

	IntrinsicMatrix K = IntrinsicMatrix::create(fx,fy,cx,cy);
	RgbdCameraPyramid camera(480,640,K);
	std::vector<std::string> rgb_files,depth_files;
	readRGBDPairFiles(folder,rgb_files,depth_files);

	std::vector<Eigen::Matrix4d> poses;
	loadTransformations(folder + "/final_poses", poses);

	float length = 2;
	int resolution = 1024;
	float sdf_trunc_percentage = 0.004;

	ScalableTSDFVolume volume(length / resolution, length * sdf_trunc_percentage,false, 16, 4);
	volume.reset();
	volume.setTsdfThreshold(0.98);
	for(size_t i = 0; i < rgb_files.size(); ++i)
	{
		std::cout << "process " << i << " frame\n";
		RgbdImagePyramidPtr cur = loadRgbdImagePyramid(camera, rgb_files[i], depth_files[i], i, depth_scale);
		volume.integrate(cur->level(0), K, poses[i].cast<float>().inverse());
	}
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
	pcl::PolygonMesh mesh;

	volume.extractPointCloud(cloud);
	dvo::savePointXYZRGBNormal(folder + "/tsdf_xyzrgbn.asc", cloud);
	volume.extractPolygonMesh(mesh);
	pcl::io::saveOBJFile(folder + "/tsdf_mesh.obj", mesh);

}
int main()
{
	std::string folder = "D:/Data/slam_data/rgbd_dataset_freiburg1_desk2";
	test_tum_rgbd_tsdf(folder);
}
