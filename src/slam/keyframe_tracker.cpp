#include <dvo/slam/keyframe_tracker.h>

#include <dvo/slam/local_tracker.h>
#include <dvo/slam/fragment_optimizer.h>

namespace dvo
{
class KeyframeTracker::Impl
{
public:
	Impl() :graph_(), lt_()
	{
		lt_.addMapCompleteCallback(boost::bind(&KeyframeTracker::Impl::onMapComplete, this, _1, _2, _3));
	}
	void onMapComplete(const LocalTracker &lt, const LocalMap::Ptr &m, bool force_keyframe)
	{
		graph_.add(m, force_keyframe);
	}
	void init(const Eigen::Matrix4d &initial_transformation)
	{
		initial_transformation_ = initial_transformation;
	}
	void update(const FramePtr &current, const double &current_time, Eigen::Matrix4d &absolute_transformation)
	{
		current->image()->level(0).timestamp = current_time;
		if (!previous_)
		{
			previous_ = current;
			absolute_transformation = initial_transformation_;
			return;
		}
		if (!lt_.getLocalMap())
		{
			lt_.initNewLocalMap(previous_, current, initial_transformation_);
			lt_.getCurrentPose(absolute_transformation);
			return;
		}
		lt_.update(current, absolute_transformation);
	}
	void forceKeyframe()
	{
		lt_.forceCompleteCurrentLocalMap();
	}
	void optimize()
	{
		graph_.optimize();
	}
	void getFinalPoses(std::vector<Eigen::Matrix4d> &final_poses)
	{
		final_poses.clear();
		graph_.getFinalPoses(final_poses);
	}
	void saveFragmentPoints(const std::string &folder)
	{
		graph_.saveFragmentPoints(folder);
	}
public:
	LocalTracker lt_;
	FragmentOptimizer graph_;
	FramePtr previous_;
	Eigen::Matrix4d initial_transformation_;

};

KeyframeTracker::KeyframeTracker() :impl_(new KeyframeTracker::Impl())
{

}
KeyframeTracker::~KeyframeTracker()
{

}
void KeyframeTracker::configure(const KeyframeTrackerConfig &cfg)
{
	LocalTracker::Config local_cfg;
	local_cfg.min_keyframe_overlap = cfg.min_keyframe_overlap;
	local_cfg.max_localmap_frames = cfg.max_localmap_frames;
	local_cfg.tracking_mode = cfg.tracking_mode;
	local_cfg.max_corr_dist = 3 * cfg.average_spacing;

	FeatureTracker::Config feature_cfg;
	feature_cfg.max_keypoints_num = cfg.max_keypoints_num;
	feature_cfg.knn = cfg.knn;
	feature_cfg.max_iterations = cfg.max_iterations;
	feature_cfg.min_valid_matches = cfg.min_valid_matches;
	feature_cfg.max_descriptor_ratio = cfg.max_descriptor_ratio;
	feature_cfg.max_keypoint_2d_dist = cfg.max_keypoint_2d_dist;
	feature_cfg.min_keypoint_pair3d_dist = cfg.min_keypoint_pair3d_dist;
	feature_cfg.max_keypoint_pair3d_dev_dist = cfg.max_keypoint_pair3d_dev_dist;
	feature_cfg.max_corr_dist = 3 * cfg.average_spacing;

	DenseConfig dense_cfg;
	dense_cfg.firstLevel = cfg.first_level;
	dense_cfg.lastLevel = cfg.last_level;
	dense_cfg.precision = cfg.dense_precison;
	dense_cfg.num_sample = cfg.frame_sample_num;
	dense_cfg.mu = cfg.mu;

	PointTracker::Config point_cfg;
	point_cfg.corr_dist_threshold = 3 * cfg.average_spacing;
	point_cfg.max_angle = cfg.max_angle;
	point_cfg.precision = cfg.icp_precision;

	FragmentOptimizer::Config backend_cfg;
	backend_cfg.group_keyframe_size = cfg.group_keyframe_size;
	backend_cfg.group_sample_num = cfg.group_sample_num;
	backend_cfg.min_group_overlap = cfg.min_group_overlap;
	backend_cfg.voxel_length = 2 * cfg.average_spacing;
	backend_cfg.search_radius = cfg.search_radius;
	backend_cfg.max_corr_dist = 4 * cfg.average_spacing;

	impl_->lt_.configue(local_cfg);
	impl_->lt_.configureFeatureTracking(feature_cfg);
	impl_->lt_.configureDenseTracking(dense_cfg);
	impl_->lt_.configurePointTracking(point_cfg);

	impl_->graph_.configure(backend_cfg);
	impl_->graph_.configurePointTracking(point_cfg);
	impl_->graph_.initialize();
}

void KeyframeTracker::init(const Eigen::Matrix4d &initial_transformation)
{
	impl_->init(initial_transformation);
}

void KeyframeTracker::update(const FramePtr &current, const double &current_time, Eigen::Matrix4d &absolute_transformation)
{
	impl_->update(current, current_time, absolute_transformation);
}

void KeyframeTracker::optimize()
{
	impl_->optimize();
}

void KeyframeTracker::forceKeyframe()
{
	impl_->forceKeyframe();
}

void KeyframeTracker::getFinalPoses(std::vector<Eigen::Matrix4d> &final_poses)
{
	impl_->getFinalPoses(final_poses);
}

void KeyframeTracker::saveFragmentPoints(const std::string &folder)
{
	impl_->saveFragmentPoints(folder);
}

}