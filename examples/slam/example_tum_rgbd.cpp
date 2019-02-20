#include <dvo/core/utility.h>
#include <dvo/core/pointcloud_io.h>
#include <dvo/slam/keyframe_tracker.h>

#include <boost/filesystem.hpp>
using namespace dvo;

void readAssocFile(const std::string &file, std::vector<std::string> &rgb_files,std::vector<std::string> &depth_files)
{
	std::ifstream in(file);
	double  rgb_time, depth_time;
	std::string rgb_file, depth_file;
	while (in.good() && !in.eof())
	{
		in >> rgb_time >> rgb_file >> depth_time >> depth_file;
		rgb_files.push_back(rgb_file);
		depth_files.push_back(depth_file);
	}
}
int main(int argc, char *argv[])
{
	std::string config_file;
	if (argc > 2)
		config_file = std::string(argv[2]);
	else
		config_file = "../../../../data/dvo_slam_tum.yaml";
	BenchmarkConfig benchmark_cfg;
	CameraConfig camera_cfg;
	KeyframeTrackerConfig keyframe_tracker_cfg;
	loadKeyframeTrackerConfig(config_file, benchmark_cfg, camera_cfg, keyframe_tracker_cfg);

	IntrinsicMatrix K = IntrinsicMatrix::create(camera_cfg.fx, camera_cfg.fy, camera_cfg.cx, camera_cfg.cy);
	RgbdCameraPyramid camera(camera_cfg.width, camera_cfg.height,K);
	std::string folder = benchmark_cfg.data_folder;
	boost::filesystem::path cloud_dir(folder + "/cloud");
	if (!boost::filesystem::exists(cloud_dir))
		boost::filesystem::create_directories(cloud_dir / "group");
	std::vector<std::string> rgb_files, depth_files;
	readAssocFile(folder + "/" + benchmark_cfg.assoc_file, rgb_files, depth_files);

	float fscale = camera_cfg.scale;
	size_t num_frame = rgb_files.size();
	size_t start = 0;
	std::vector<FramePtr> frames;
	for (size_t i = 0; i < num_frame; ++i)
	{
		std::cout << "load " << i << " frame\n";
		RgbdImagePyramidPtr current = loadRgbdImagePyramid(camera, folder + "/" + rgb_files[i],
			folder + "/" + depth_files[i],i, fscale);
		
		current->level(0).buildPointCloud(4, 1.5f);
		IPointCloud::Ptr icloud = IPointCloud::create(current->level(0).pointcloud);
		if (i == start)
		{
			float average_spacing = icloud->computeAverageSpacing(8);
			keyframe_tracker_cfg.average_spacing = average_spacing;
		}
		icloud->samplePointCloud(keyframe_tracker_cfg.frame_sample_num);
		Frame::Ptr frame = Frame::create(current, icloud);
		frames.push_back(frame);
	}
	std::cout << "average spacing: " << keyframe_tracker_cfg.average_spacing << std::endl;
	std::cout << "start tracking...\n";
	std::vector<Eigen::Matrix4d> online_poses, final_poses;
	double timestamp = 0.0;
	Eigen::Matrix4d pose;

	KeyframeTracker keyframe_tracker;
	keyframe_tracker.configure(keyframe_tracker_cfg);
	keyframe_tracker.init(Eigen::Matrix4d::Identity());
	for (size_t i = start; i < num_frame; ++i)
	{
		std::cout << "process image: " << i << std::endl;
		if (num_frame - i == 1)
			keyframe_tracker.forceKeyframe();

		keyframe_tracker.update(frames[i - start], timestamp, pose);
		online_poses.push_back(pose);
		timestamp += 0.001;
	}
	std::cout << "online tracking done!\n";

	std::cout << "start final optimization...\n";
	keyframe_tracker.optimize();
	keyframe_tracker.getFinalPoses(final_poses);
	std::cout << "optimize done!\n";
	saveTransformations(folder + "/init_poses.txt", online_poses);
	saveTransformations(folder + "/final_poses.txt", final_poses);
	keyframe_tracker.saveFragmentPoints(folder + "/cloud/group");

}