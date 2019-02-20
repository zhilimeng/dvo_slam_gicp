#include <dvo/slam/config.h>

#include <opencv2/opencv.hpp>
namespace dvo
{
//////////////////////////////////////////////////////////////////////////
bool readBoolValue(cv::FileNode &fn, const std::string &value)
{
	cv::FileNode bn = fn[value];
	if (bn.isString())
	{
		std::string val_str = (std::string)bn;
		boost::to_upper(val_str);
		if (val_str == "TRUE" || val_str == "T")
			return true;
	}else if (bn.isInt())
	{
		return static_cast<bool>((int)bn);
	}else if (bn.isReal())
	{
		return static_cast<bool>((float)bn);
	}
	return false;
}

bool loadKeyframeTrackerConfig(const std::string &cfg_file,BenchmarkConfig &benchmark_cfg, CameraConfig &camera_cfg, KeyframeTrackerConfig &keyframe_cfg)
{
	cv::FileStorage fs;
	if (!fs.open(cfg_file,cv::FileStorage::READ))
	{
		std::cerr << "can not open file: " << cfg_file << std::endl;
		return false;
	}
	cv::FileNode benchmark_node = fs["BenchmarkConfig"];
	benchmark_cfg.data_folder = (std::string)benchmark_node["data_folder"];
	benchmark_cfg.assoc_file = (std::string)benchmark_node["assoc_file"];
	benchmark_cfg.traj_file = (std::string)benchmark_node["trajectory_file"];

	cv::FileNode camera_node = fs["CameraConfig"];
	camera_cfg.fx = (float)camera_node["fx"];
	camera_cfg.fy = (float)camera_node["fy"];
	camera_cfg.cx = (float)camera_node["cx"];
	camera_cfg.cy = (float)camera_node["cy"];
	camera_cfg.scale = (float)camera_node["scale"];
	camera_cfg.width = (int)camera_node["width"];
	camera_cfg.height = (int)camera_node["height"];

	cv::FileNode keyframe_node = fs["KeyframeTrackerConfig"];
	keyframe_cfg.average_spacing = (float)keyframe_node["average_spacing"];
	keyframe_cfg.frame_sample_num = (int)keyframe_node["frame_sample_num"];
	keyframe_cfg.min_keyframe_overlap = (float)keyframe_node["min_keyframe_overlap"];
	keyframe_cfg.max_localmap_frames = (int)keyframe_node["max_localmap_frames"];
	keyframe_cfg.tracking_mode = static_cast<TrackingMode>((int)keyframe_node["tracking_mode"]);

	keyframe_cfg.group_keyframe_size = (int)keyframe_node["group_keyframe_size"];
	keyframe_cfg.group_sample_num = (int)keyframe_node["group_sample_num"];
	keyframe_cfg.min_group_overlap = (float)keyframe_node["min_group_overlap"];
	keyframe_cfg.search_radius = (float)keyframe_node["search_radius"];

	keyframe_cfg.max_keypoints_num = (int)keyframe_node["max_keypoints_num"];
	keyframe_cfg.knn = (int)keyframe_node["knn"];
	keyframe_cfg.max_iterations = (int)keyframe_node["max_iterations"];
	keyframe_cfg.min_valid_matches = (int)keyframe_node["min_valid_matches"];
	keyframe_cfg.max_descriptor_ratio = (float)keyframe_node["max_descriptor_ratio"];
	keyframe_cfg.max_keypoint_2d_dist = (float)keyframe_node["max_keypoint_2d_dist"];
	keyframe_cfg.min_keypoint_pair3d_dist = (float)keyframe_node["min_keypoint_pair3d_dist"];
	keyframe_cfg.max_keypoint_pair3d_dev_dist = (float)keyframe_node["max_keypoint_pair3d_dev_dist"];

	keyframe_cfg.first_level = (int)keyframe_node["first_level"];
	keyframe_cfg.last_level = (int)keyframe_node["last_level"];
	keyframe_cfg.dense_precison = (float)keyframe_node["dense_precision"];
	keyframe_cfg.mu = (float)keyframe_node["mu"];
	keyframe_cfg.icp_precision = (float)keyframe_node["icp_precision"];
	keyframe_cfg.max_angle = (float)keyframe_node["max_angle"];

	return true;
}

//////////////////////////////////////////////////////////////////////////

}