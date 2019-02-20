/*!
 * \file frame.h
 * \brief 
 *
 * \author mengzhili
 * \date 2018/11/21 13:55
 *
 * 
 */
#ifndef DVO_FRAME_H
#define DVO_FRAME_H

#include <dvo/core/rgbd_image.h>
#include <dvo/core/ipoint_cloud.h>

namespace dvo
{
class DVO_EXPORTS Frame
{
public:
	typedef boost::shared_ptr<Frame> Ptr;
	typedef boost::shared_ptr<const Frame> ConstPtr;
	typedef IPointCloud::PointCloudPtr PointCloudPtr;
	typedef Eigen::Matrix4d Matrix4;
		
	virtual ~Frame();
	/* * \brief  create a frame with given image and point cloud*/
	static Frame::Ptr create(const dvo::RgbdImagePyramidPtr& image, const dvo::IPointCloudPtr &ipointcloud, const Matrix4& pose = Matrix4::Identity());

	/* * \brief get RgbdImagePyramidPtr */
	RgbdImagePyramidPtr image(){ return (image_); }
	/* * \brief get IPointCloudPtr */
	IPointCloudPtr icloud(){ return (icloud_); }
	/* * \brief get PointCloudPtr */
	PointCloudPtr cloud(){ return (icloud_->cloud()); }
	/* * \brief get frame pose */
	Matrix4 pose(){ return (pose_); }
	/* * \brief set frame pose */
	void pose(const Matrix4& pose){ pose_ = pose; }
	/* * \brief get frame centroid*/
	Eigen::Vector4d centroid() { return pose_ * icloud_->centroid(); }
	/* * \brief get frame id */
	int id(){ return image_->id(); }

	bool isKeyframe() { return keyframe_flag_; }
	void setKeyframe() { keyframe_flag_ = true; }
private:
	RgbdImagePyramidPtr image_;
	IPointCloudPtr icloud_;
	Matrix4 pose_;
	bool keyframe_flag_;
private:
	Frame(const dvo::RgbdImagePyramidPtr& image, const dvo::IPointCloudPtr &ipointcloud, const Matrix4& pose = Matrix4::Identity());
};

typedef Frame::Ptr FramePtr;
typedef Frame::ConstPtr FrameConstPtr;
}
#endif