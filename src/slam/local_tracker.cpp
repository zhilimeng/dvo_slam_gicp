#include <dvo/slam/local_tracker.h>
namespace dvo
{
boost::signals2::connection LocalTracker::addMapCompleteCallback(const MapCompleteCallback &callback)
{
	return map_complete_.connect(callback);
}

LocalMap::Ptr LocalTracker::getLocalMap() const
{
	return local_map_;
}

void LocalTracker::getCurrentPose(Matrix4 &pose)
{
	local_map_->getCurrentFramePose(pose);
}

void LocalTracker::initNewLocalMap(const FramePtr &keyframe, const FramePtr &frame, const Matrix4 &keyframe_pose /*= Matrix4::Identity()*/)
{
	keyframe_ = keyframe;
	keyframe_->pose(keyframe_pose);
	active_frame_ = frame;

	Matrix4 feature_trans = Matrix4::Identity();
	if (!feature_tracker_.align(keyframe_->image()->level(0), active_frame_->image()->level(0), feature_trans))
	{
		std::cout << "\t warning: feature tracking fail!\n";
		feature_trans = velocity_;
	}
	else {
		velocity_ = feature_trans;
	}

	Matrix4 final_trans;
	Matrix6 information;
	switch (cfg_.tracking_mode)
	{
	case kTexture:
	{
		DenseTracker::Result dense_result;
		dense_tracker_.match(*keyframe_->image(), *active_frame_->image(), dense_result, feature_trans);
		final_trans = dense_result.transformation;
		information = dense_result.information;
		break;
	}
	case kGeometry:
	{
		PointTracker::Result point_result;
		point_tracker_.align(keyframe_->icloud(), active_frame_->icloud(), point_result, feature_trans);
		final_trans = point_result.transformation;
		information = point_result.information;
		break;
	}
	}
	initNewLocalMap(keyframe, frame, final_trans, information, keyframe_pose);
}

void LocalTracker::initNewLocalMap(const FramePtr &keyframe, const FramePtr &frame, const Matrix4 &t_odometry, const Matrix6 &i_odometry, const Matrix4 &keyframe_pose)
{
//	frame->pose(keyframe_pose * t_odometry);
	std::cout << "\t current keyframe: " << keyframe_->id() << std::endl;
	local_map_ = LocalMap::create(keyframe, keyframe_pose);
	local_map_->addFrame(frame);
	local_map_->addKeyframeMeasurement(t_odometry, i_odometry);
}

bool LocalTracker::isNewKeyframe(const FramePtr &frame, const Matrix4 &t_odometry, const Matrix4 &t_keyframe)
{
	int image_diff_threshold = cfg_.max_localmap_frames;
	float keyframe_overlap_threshold = cfg_.min_keyframe_overlap;

	float keyframe_overlap = keyframe_->icloud()->computeOverlap(frame->icloud(), t_keyframe, cfg_.max_corr_dist);
//	std::cout << "\t keyframe overlap: " << keyframe_overlap << std::endl;

	return keyframe_overlap < keyframe_overlap_threshold || (frame->id() - keyframe_->id()) > image_diff_threshold;
}

void LocalTracker::update(const FramePtr &frame, Matrix4 &pose)
{
	Matrix4 active_feature_trans = Matrix4::Identity();
	Matrix4 keyframe_feature_trans = Matrix4::Identity();

	active_frame_ = local_map_->getCurrentFrame();
	Matrix4 active_frame_pose;
	local_map_->getCurrentFramePose(active_frame_pose);
	if (!feature_tracker_.align(active_frame_->image()->level(0), frame->image()->level(0), active_feature_trans))
	{
		std::cout << "\t warning: feature tracking fail!\n";
		active_feature_trans = velocity_;
	}
	else 
	{
		velocity_ = active_feature_trans;
	}
	keyframe_feature_trans = keyframe_->pose().inverse() * active_frame_pose * active_feature_trans;

	Matrix4 t_odometry, t_keyframe;
	Matrix6 i_odometry, i_keyframe;
	switch (cfg_.tracking_mode)
	{
	case kTexture:
	{
		DenseTracker::Result r_odo, r_key;
		dense_tracker_.match(*keyframe_->image(), *frame->image(), r_key, keyframe_feature_trans);
		dense_tracker_.match(*active_frame_->image(), *frame->image(), r_odo, active_feature_trans);
		t_odometry = r_odo.transformation;
		i_odometry = r_odo.information;
		t_keyframe = r_key.transformation;
		i_keyframe = r_key.information;
		break;
	}
	case kGeometry:
	{
		PointTracker::Result r_odo, r_key;
		point_tracker_.align(keyframe_->icloud(), frame->icloud(), r_key, keyframe_feature_trans);
		point_tracker_.align(active_frame_->icloud(), frame->icloud(), r_odo, active_feature_trans);
		t_odometry = r_odo.transformation;
		i_odometry = r_odo.information;
		t_keyframe = r_key.transformation;
		i_keyframe = r_key.information;
		break;
	}
	default:
		break;
	}

	// is keyframe
	if (!isNewKeyframe(frame,t_odometry,t_keyframe) && !force_keyframe_)
	{
		local_map_->addFrame(frame);
		local_map_->addOdometryMeasurement(t_odometry, i_odometry);
		local_map_->addKeyframeMeasurement(t_keyframe, i_keyframe);
	}
	else
	{
		force_keyframe_ = false;
		LocalMap::Ptr old_map = local_map_;
		map_complete_(*this, old_map, force_keyframe_);
		keyframe_ = old_map->getCurrentFrame();
		Matrix4 keyframe_pose;
		old_map->getCurrentFramePose(keyframe_pose);
		initNewLocalMap(keyframe_, frame, t_odometry,i_odometry, keyframe_pose);
	}
	local_map_->getCurrentFramePose(pose);
}

void LocalTracker::forceCompleteCurrentLocalMap()
{
	force_keyframe_ = true;
}

}