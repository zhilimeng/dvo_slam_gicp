/*!
 * \file local_tracker.h
 *
 * \author mengzhili
 * \date 2019/01/22 
 * 
 */
#ifndef RULER_SLAM_LOCAL_TRACKER_H
#define RULER_SLAM_LOCAL_TRACKER_H
#include <boost/signals2.hpp>

#include <dvo/core/frame.h>
#include <dvo/core/eigen.h>
#include <dvo/registration/feature_tracking.h>
#include <dvo/registration/dense_tracking.h>
#include <dvo/registration/point_tracking.h>
#include <dvo/slam/config.h>
#include <dvo/slam/local_map.h>
namespace dvo
{
class LocalTracker
{
public:
	typedef Eigen::Matrix4d Matrix4;
	typedef Eigen::Matrix6d Matrix6;
	typedef boost::signals2::signal<void(const LocalTracker&, const LocalMap::Ptr&, bool force_keyframe)> MapCompleteSignal;
	typedef MapCompleteSignal::slot_type MapCompleteCallback;

	struct Config 
	{
		TrackingMode tracking_mode;
		float max_corr_dist;
		float min_keyframe_overlap;
		int max_localmap_frames;
	};
public:
	LocalTracker() :force_keyframe_(false) {}
	~LocalTracker() {}
	void configue(LocalTracker::Config &cfg) { cfg_ = cfg; }
	void configureFeatureTracking(const FeatureTracker::Config &feature_cfg) { feature_tracker_.configure(feature_cfg); }
	void configureDenseTracking(const DenseConfig &dense_cfg) { dense_tracker_.configure(dense_cfg); }
	void configurePointTracking(const PointTracker::Config &point_cfg) { point_tracker_.configure(point_cfg); }

	boost::signals2::connection addMapCompleteCallback(const MapCompleteCallback &callback);
	LocalMap::Ptr getLocalMap() const;
	void getCurrentPose(Matrix4 &pose);

	void initNewLocalMap(const FramePtr &keyframe, const FramePtr &frame, const Matrix4 &keyframe_pose = Matrix4::Identity());
	void update(const FramePtr &frame, Matrix4 &pose);
	void forceCompleteCurrentLocalMap();
private:
	void initNewLocalMap(const FramePtr &keyframe, const FramePtr &frame, const Matrix4 &t_odometry,const Matrix6 &i_odometry, const Matrix4 &keyframe_pose);
	bool isNewKeyframe(const FramePtr &frame, const Matrix4 &t_odometry, const Matrix4 &t_keyframe);
private:
	LocalMap::Ptr local_map_;
	LocalTracker::Config cfg_;
	FeatureTracker feature_tracker_;
	PointTracker point_tracker_;
	DenseTracker dense_tracker_;
	MapCompleteSignal map_complete_;

	FramePtr keyframe_;
	FramePtr active_frame_;
	Matrix4 velocity_;
	bool force_keyframe_;


};
}

#endif
