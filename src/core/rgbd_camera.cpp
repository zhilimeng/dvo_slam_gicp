#include <dvo/core/rgbd_camera.h>
#include <dvo/core/rgbd_image.h>
#include <boost/make_shared.hpp>

namespace dvo
{
RgbdCamera::RgbdCamera(int width, int height, const dvo::IntrinsicMatrix& intrinsics) :
	width_(width), height_(height), intrinsics_(intrinsics)
{
	pointcloud_template_.resize(Eigen::NoChange, width_*height_);
	int idx = 0;
	for (int y = 0; y < height_; ++y)
	{
		for (int x = 0; x < width_; ++x,++idx)
		{
			pointcloud_template_(0, idx) = (x - intrinsics_.cx()) / intrinsics_.fx();
			pointcloud_template_(1, idx) = (y - intrinsics_.cy()) / intrinsics_.fy();
			pointcloud_template_(2, idx) = 1.0;
			pointcloud_template_(3, idx) = 0.0;
		}
	}
}

RgbdCamera::~RgbdCamera()
{

}
int RgbdCamera::width() const
{
	return width_;
}
int RgbdCamera::height() const
{
	return height_;
}
const dvo::IntrinsicMatrix& RgbdCamera::intrinsics() const
{
	return intrinsics_;
}

const PointCloudTemplate& RgbdCamera::pointcloudTemplate() const
{
	return pointcloud_template_;
}

RgbdImagePtr RgbdCamera::create(const cv::Mat& intensity, const cv::Mat& depth) const
{
	RgbdImagePtr result(new RgbdImage(*this));
	result->intensity = intensity;
	result->depth = depth;
	result->initialize();
	return result;
}
RgbdImagePtr RgbdCamera::create() const
{
	return boost::make_shared<RgbdImage>(*this);
}

bool RgbdCamera::hasSameSize(const cv::Mat& img) const
{
	return img.cols == width_ && img.rows == height_;
}
/*
	RgbdCameraPyramid
*/
RgbdCameraPyramid::RgbdCameraPyramid(const RgbdCamera& base)
{
	levels_.push_back(boost::make_shared<RgbdCamera>(base));
}

RgbdCameraPyramid::RgbdCameraPyramid(int base_with, int base_height, const dvo::IntrinsicMatrix& base_intrinsics)
{
	levels_.push_back(boost::make_shared<RgbdCamera>(base_with, base_height, base_intrinsics));
}
RgbdCameraPyramid::~RgbdCameraPyramid()
{

}
RgbdImagePyramidPtr RgbdCameraPyramid::create(const cv::Mat& base_intensity, const cv::Mat& base_depth)
{
	return RgbdImagePyramidPtr(new RgbdImagePyramid(*this, base_intensity, base_depth));
}

void RgbdCameraPyramid::build(size_t levels)
{
	size_t start = levels_.size();
	for (size_t idx = start; idx < levels; ++idx)
	{
		RgbdCameraPtr& previous = levels_[idx - 1];

		dvo::IntrinsicMatrix intrinsics(previous->intrinsics());
		intrinsics.scale(0.5);
		intrinsics.data(0, 2) -= 0.25f;
		intrinsics.data(1, 2) -= 0.25f;
		levels_.push_back(boost::make_shared<RgbdCamera>(previous->width() / 2, previous->height() / 2, intrinsics));
	}
}

const RgbdCamera& RgbdCameraPyramid::level(size_t level)
{
	build(level + 1);

	return *levels_[level];
}

const RgbdCamera& RgbdCameraPyramid::level(size_t level) const
{
	return *levels_[level];
}
}