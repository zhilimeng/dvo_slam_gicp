#include <dvo/slam/fragment.h>

#include <dvo/core/volume_fusion.h>
#include <dvo/core/volume_feature_fusion.h>
#include <dvo/core/pointcloud_io.h>
namespace dvo
{

void Fragment::addLocalMap(const LocalMap::Ptr &m)
{
	std::vector<FramePtr> local_frames = m->getFrames();
	for (size_t i = 0; i< local_frames.size() -1; ++i)
	{
		frames_.push_back(local_frames[i]);
	}
	keyframes_.push_back(m->getKeyframe());
}

	//////////////////////////////////////////////////////////////////////////
void Fragment::updatePose(const Matrix4 &first_pose)
{
	Matrix4 first_pose_inverse = frames_.front()->pose().inverse();
	for (const auto &frame : frames_)
	{
		Matrix4 relative_pose = first_pose_inverse * frame->pose();
		frame->pose(first_pose * relative_pose);
	}
}
//////////////////////////////////////////////////////////////////////////
void Fragment::buildPointCloud(float voxel_length)
{
	if (!point_cloud_requires_build_) return;
	Matrix4 first_pose_inverse = frames_.front()->pose().inverse();
	point_cloud_.reset(new PointCloud);
	// voxel downsample keyframe pointcloud;
	VolumeFusion volume_fusion;
	volume_fusion.setVoxelLength(voxel_length);
	for (const auto &frame : frames_)
	{
		if (frame->isKeyframe())
		{
			Matrix4 trans2first = first_pose_inverse * frame->pose();
			volume_fusion.integrate(*(frame->cloud()), trans2first);
		}
	}
	volume_fusion.extractPointCloud(*point_cloud_);
	point_cloud_requires_build_ = false;
}
void Fragment::buildKeyPointCloud(float voxel_length)
{
	if (!descriptors_requires_build_) return;
	const std::string keypoint_type = "SURF";
	const std::string desc_type = "SURF";
	const int desc_size = 64;
	const int num_keypoints = 1000;
	Matrix4 first_pose_inverse = frames_.front()->pose().inverse();
	keypoint_cloud_.reset(new PointCloud);
	VolumeFeatureFusion feature_fusion;
	feature_fusion.setVoxelLength(voxel_length);
	feature_fusion.setDescriptorSize(desc_size);
	for (const auto &frame:frames_)
	{
		if (frame->isKeyframe())
		{
			RgbdImage &image = frame->image()->level(0);
			image.detectKeyPoint(keypoint_type, desc_type, num_keypoints);
			Matrix4 trans2first = first_pose_inverse * frame->pose();
			feature_fusion.integrate(*image.kpoints, image.keys_desc, trans2first);
		}
	}
	feature_fusion.extractPointAndDescriptors(*keypoint_cloud_, features_);
	descriptors_requires_build_ = false;
}
void Fragment::saveFragmentPoints(const std::string &folder)
{
	std::string filename = folder + "/" + std::to_string(id_) + "_" + std::to_string(frames_.front()->id()) + ".asc";
	Eigen::Matrix4f pose_f = frames_.front()->pose().cast<float>();
	savePointXYZRGBNormal(filename, *point_cloud_, pose_f);
}
}