#ifndef DVO_SLAM_LOCAL_MAP_H
#define DVO_SLAM_LOCAL_MAP_H
#include <dvo/core/frame.h>
#include <dvo/core/eigen.h>
namespace dvo
{
class LocalMap
{
public:
	typedef boost::shared_ptr<LocalMap> Ptr;
	typedef pcl::PointXYZRGBNormal PointT;
	typedef Eigen::Matrix4d Matrix4;
	typedef Eigen::Matrix6d Matrix6;
	virtual ~LocalMap();

	/* * \brief create a new local map */
	static LocalMap::Ptr create(const FramePtr &keyframe,const Matrix4 &keyframe_pose);

	FramePtr getKeyframe();

	const std::vector<FramePtr>& getFrames() const { return frames_; }

	void setKeyframePose(const Matrix4 &keyframe_pose);
	FramePtr getCurrentFrame();
	void getCurrentFramePose(Matrix4 &current_pose);

	void addFrame(const FramePtr &frame);

	void addOdometryMeasurement(const Matrix4 &pose, const Matrix6 &information);

	void addKeyframeMeasurement(const Matrix4 &pose, const Matrix6 &information);

	void optimize();
private:
	LocalMap(const FramePtr &keyframe,const Matrix4 &keyframe_pose);
	class Impl;
	boost::scoped_ptr<Impl> impl_;
	std::vector<FramePtr> frames_;
};
}
#endif