#ifndef  DVO_RGBD_CAMERA_H
#define  DVO_RGBD_CAMERA_H
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/smart_ptr.hpp>
#include <dvo/core/intrinsic_matrix.h>

namespace dvo
{
class RgbdImage;
typedef boost::shared_ptr<RgbdImage> RgbdImagePtr;

class RgbdImagePyramid;
typedef boost::shared_ptr<RgbdImagePyramid> RgbdImagePyramidPtr;

typedef Eigen::Matrix<float, 4, Eigen::Dynamic, Eigen::ColMajor> PointCloudTemplate;
class DVO_EXPORTS RgbdCamera
{
public:
	RgbdCamera(int width, int height, const dvo::IntrinsicMatrix& intrinsices);
	~RgbdCamera();
	int width() const;
	int height() const;
	const dvo::IntrinsicMatrix& intrinsics() const;
	const PointCloudTemplate& pointcloudTemplate() const;

	RgbdImagePtr create(const cv::Mat& intensity, const cv::Mat& depth) const;
	RgbdImagePtr create() const;
private:
	bool hasSameSize(const cv::Mat& img) const;
private:
	int width_;
	int height_;
	dvo::IntrinsicMatrix intrinsics_;
	PointCloudTemplate pointcloud_template_;
};

typedef boost::shared_ptr<RgbdCamera> RgbdCameraPtr;
typedef boost::shared_ptr<const RgbdCamera> RgbdCameraConstPtr;

class DVO_EXPORTS RgbdCameraPyramid
{
public:
	RgbdCameraPyramid(const RgbdCamera& base);
	RgbdCameraPyramid(int base_with, int base_height, const dvo::IntrinsicMatrix& base_intrinsics);
	~RgbdCameraPyramid();

	RgbdImagePyramidPtr create(const cv::Mat& base_intensity, const cv::Mat& base_depth);

	void build(size_t levels);

	const RgbdCamera& level(size_t level);
	const RgbdCamera& level(size_t level) const;
private:
	std::vector<RgbdCameraPtr> levels_;
};

typedef boost::shared_ptr<RgbdCameraPyramid> RgbdCameraPyramidPtr;
typedef boost::shared_ptr<const RgbdCameraPyramid> RgbdCameraPyramidConstPtr;
}
#endif