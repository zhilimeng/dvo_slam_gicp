/*!
 * \file config.h
 *
 * \author mengzhili
 * \date 2019/01/22 
 * 
 */
#ifndef DVO_SLAM_CONFIG_H
#define DVO_SLAM_CONFIG_H
#include <boost/algorithm/string.hpp>
#include <dvo/core/dvo_exports.h>
namespace dvo
{
struct BenchmarkConfig 
{
	std::string data_folder;
	std::string assoc_file;
	std::string traj_file;
};

struct  CameraConfig
{
	float fx, fy;
	float cx, cy;
	float scale;
	int width;
	int height;
};

enum TrackingMode
{
	kTexture = 0,
	kGeometry
};

struct KeyframeTrackerConfig
{
	float average_spacing;
	int frame_sample_num;
	// frontend
	float min_keyframe_overlap;
	float max_localmap_frames;
	TrackingMode tracking_mode;

	// feature tracking
	int max_keypoints_num;
	int knn;
	int max_iterations;
	int min_valid_matches;
	float max_descriptor_ratio;
	float max_keypoint_2d_dist;
	float min_keypoint_pair3d_dist;
	float max_keypoint_pair3d_dev_dist;

	// dense
	int first_level;
	int last_level;
	float dense_precison;
	float mu;

	// icp
	int icp_precision;
	float max_angle;

	// backend
	int group_keyframe_size;
	int group_sample_num;
	float min_group_overlap;
	float search_radius;
};

bool DVO_EXPORTS loadKeyframeTrackerConfig(const std::string &cfg_file, BenchmarkConfig &benchmark_cfg,
										CameraConfig &camera_cfg, KeyframeTrackerConfig &keyframe_cfg);
}
#endif
