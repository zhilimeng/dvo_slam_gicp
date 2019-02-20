#ifndef DVO_SLAM_FRAGMENT_H
#define DVO_SLAM_FRAGMENT_H
#include <dvo/core/frame.h>
#include <dvo/slam/local_map.h>

namespace dvo
{
class Fragment
{
public:
	typedef Eigen::Matrix4d Matrix4;
	typedef pcl::PointXYZRGBNormal PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
public:
	Fragment():
		point_cloud_(),keypoint_cloud_(),features_(),id_(-1),
		point_cloud_requires_build_(true),descriptors_requires_build_(true)
	{}
	~Fragment() {}
public:
	void id(int id) { id_ = id; }
	int id() { return id_; }
	size_t size() { return frames_.size(); }

	void addFrame(const FramePtr &frame) { frames_.push_back(frame); }
	FramePtr getFrame(int idx) {  return frames_[idx]; }

	void addLocalMap(const LocalMap::Ptr &m);

	/* * \brief update fragment pose
	   * \param[in] first frame pose in fragment
	*/
	void updatePose(const Matrix4 &first_pose);
	Matrix4 getPose() { return frames_.front()->pose(); }

	void buildPointCloud(float voxel_length);
	void buildKeyPointCloud(float voxel_length);
	PointCloudPtr getPointCloud() { return point_cloud_; }
	PointCloudPtr getKeyPointCloud() { return keypoint_cloud_; }
	cv::Mat getFeatures() { return features_; }

	void saveFragmentPoints(const std::string &folder);

private:
	std::vector<FramePtr> frames_;
	std::vector<FramePtr> keyframes_;
	PointCloudPtr point_cloud_;
	PointCloudPtr keypoint_cloud_;
	cv::Mat features_;
	int id_;
	bool point_cloud_requires_build_;
	bool descriptors_requires_build_;
};
typedef boost::shared_ptr<Fragment> FragmentPtr;
}
#endif