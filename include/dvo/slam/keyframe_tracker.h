/*!
 * \file keyframe_tracker.h
 *
 * \author mengzhili
 * \date 2019/01/23 
 * 
 */
#ifndef DVO_SLAM_KEYFRAME_TRACKER_H
#define DVO_SLAM_KEYFRAME_TRACKER_H
#include <dvo/core/frame.h>
#include <dvo/slam/config.h>
namespace dvo
{
class DVO_EXPORTS KeyframeTracker
{
public:
	KeyframeTracker();
	~KeyframeTracker();

	void configure(const KeyframeTrackerConfig &cfg);

	void init(const Eigen::Matrix4d &initial_transformation);

	void update(const FramePtr &current, const double &current_time, Eigen::Matrix4d &absolute_transformation);

	void optimize();

	void forceKeyframe();

	void getFinalPoses(std::vector<Eigen::Matrix4d> &final_poses);

	void saveFragmentPoints(const std::string &folder);
private:
	class Impl;
	boost::scoped_ptr<Impl> impl_;
};
}
#endif
