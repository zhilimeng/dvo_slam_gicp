#include <iostream>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/utility.h>
#include <dvo/core/pointcloud_io.h>
#include <dvo/registration/feature_tracking.h>
#include <dvo/registration/point_tracking.h>
#include <dvo/registration/dense_tracking.h>
#include <pcl/common/time.h>
using namespace dvo;
struct RgbdPair
{
	double rgb_timestamp;
	std::string rgb_file;
	double depth_timestamp;
	std::string depth_file;
};

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
void test_tum_fr1desk2(const std::string &folder, int ref_id, int cur_id)
{
	std::vector<std::string> rgb_files, depth_files;

	//dvo::getFilesFromDirectory(folder + "/rgb", ".png", rgb_files);
	//dvo::getFilesFromDirectory(folder + "/depth", ".png", depth_files);
	readRGBDPairFiles(folder, rgb_files, depth_files);

	dvo::IntrinsicMatrix K = dvo::IntrinsicMatrix::create(517.3f, 516.5f, 318.6f, 255.3f);
	dvo::RgbdCameraPyramid camera(640, 480, K);
	float depth_scale = 5000.f;
	dvo::RgbdImagePyramidPtr ref = dvo::loadRgbdImagePyramid(camera, rgb_files[ref_id], depth_files[ref_id], ref_id, depth_scale);
	dvo::RgbdImagePyramidPtr cur = dvo::loadRgbdImagePyramid(camera, rgb_files[cur_id], depth_files[cur_id], cur_id, depth_scale);

	saveRgbdImagePoints(folder + "/" + std::to_string(ref_id) + ".asc", ref->level(0), Eigen::Matrix4d::Identity(), 1);
	saveRgbdImagePoints(folder + "/" + std::to_string(cur_id) + ".asc", cur->level(0), Eigen::Matrix4d::Identity(), 1);
	dvo::FeatureTracker::Config cfg;
	cfg.descriptor_type = "SURF";
	cfg.detector_type = "SURF";
	cfg.max_keypoints_num = 1000;
	cfg.knn = 3;
	cfg.min_valid_matches = 6;
	cfg.max_descriptor_ratio = 0.35;
	cfg.max_keypoint_2d_dist = 300;
	cfg.min_keypoint_pair3d_dist = 0.015;
	cfg.max_keypoint_pair3d_dev_dist = 0.002;
	cfg.max_corr_dist = 0.01;
	cfg.verbose = true;

	dvo::FeatureTracker ft;
	ft.configure(cfg);
	pcl::StopWatch sw;
	Eigen::Matrix4d feature_trans = Eigen::Matrix4d::Identity();
	ft.align(ref->level(0), cur->level(0), feature_trans);
	double time = sw.getTime();

	std::cout << "feature match time: " << time << "\n";
	std::cout << "feature trans:\n" << feature_trans << "\n";

	saveRgbdImagePoints(folder + "/" + std::to_string(cur_id) + "_feature_trans.asc", cur->level(0), feature_trans, 1);

	dvo::DenseConfig dense_cfg;
	dense_cfg.firstLevel = 0;
	dense_cfg.lastLevel = 0;
	dense_cfg.maxIterationsPerLevel = 50;
	dense_cfg.precision = 1e-6;
	dense_cfg.mu = 0.0;
	dense_cfg.num_sample = 5000;

	dvo::DenseTracker dt;
	dt.configure(dense_cfg);

	sw.reset();
	Eigen::Matrix4d dense_trans;
	dt.match(*ref, *cur, dense_trans, feature_trans);
	time = sw.getTime();
	std::cout << "dense match time: " << time << "\n";
	std::cout << "transformation: \n" << dense_trans << std::endl;

	saveRgbdImagePoints(folder + "/" + std::to_string(cur_id) + "_dense_trans.asc", cur->level(0), dense_trans, 1);

	Eigen::Matrix4d init_trans;
	loadTransformationBinary(folder + "/velocity.transform", init_trans);
	dvo::PointTracker::Config point_cfg;
	point_cfg.corr_dist_threshold = 0.025;
	point_cfg.max_angle = 45.0;
	point_cfg.precision = 1e-6;
	point_cfg.max_iterations = 50;

	ref->level(0).buildPointCloud(4);
	cur->level(0).buildPointCloud(4);
	IPointCloudPtr ref_cloud = IPointCloud::create(ref->level(0).pointcloud);
	IPointCloudPtr cur_cloud = IPointCloud::create(cur->level(0).pointcloud);
	Eigen::Matrix4d icp_trans;
	dvo::PointTracker pt;
	pt.configure(point_cfg);
	pt.align(ref_cloud, cur_cloud, icp_trans, init_trans);
	std::cout << "icp transformation:\n" << icp_trans << std::endl;
	saveRgbdImagePoints(folder + "/" + std::to_string(cur_id) + "_icp_trans.asc", cur->level(0), icp_trans, 1);

	savePointXYZRGBNormal(folder + "/" + std::to_string(ref_id) + ".asc", *(ref_cloud->cloud()));
	savePointXYZRGBNormal(folder + "/" + std::to_string(cur_id) + "icp_xyzrgbn.asc",*(cur_cloud->cloud()), icp_trans.cast<float>());
}
int main()
{
	std::string folder = "D:/Data/slam_data/rgbd_dataset_freiburg1_desk2";
	int ref_id = 50;
	int cur_id = 51;

	test_tum_fr1desk2(folder, ref_id, cur_id);
}
