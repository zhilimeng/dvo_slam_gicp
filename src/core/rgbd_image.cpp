#include <dvo/core/rgbd_image.h>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

#include <dvo/core/interpolation.h>
namespace dvo
{
class IsInvalidKeyPoint
{
private:
	cv::Mat depth_;
public:
	IsInvalidKeyPoint(const cv::Mat &depth) :depth_(depth) {}

	bool operator()(const cv::KeyPoint& elem)
	{
		if (elem.pt.x < 0 || elem.pt.y < 0 || elem.pt.x > depth_.cols - 1 || elem.pt.y > depth_.rows - 1)
			return true;

		int x0 = (int)std::floor(elem.pt.x);
		int y0 = (int)std::floor(elem.pt.y);

		int x1 = x0 + 1;
		int y1 = y0 + 1;
		if (!std::isfinite(depth_.at<float>(y0, x0)))
			return true;
		if (!std::isfinite(depth_.at<float>(y1, x0)))
			return true;
		if (!std::isfinite(depth_.at<float>(y0, x1)))
			return true;
		if (!std::isfinite(depth_.at<float>(y1, x1)))
			return true;
		return false;
	}
};

RgbdImage::RgbdImage(const RgbdCamera& camera):
	camera_(camera),
	intensity_requires_calculation_(true),
	depth_requires_calculation_(true),
	pointcloud_requires_build_(true),
	pointcloud_requires_sample_(true),
	jwjz_requires_compute_(true),
	width(0),
	height(0),
	pointcloud(),
	kpoints(new pcl::PointCloud<PointT>),
	kp_tree(new pcl::KdTreeFLANN<PointT>)
	{

	}
RgbdImage::~RgbdImage()
{

}

const RgbdCamera& RgbdImage::camera() const
{
	return camera_;
}

void RgbdImage::initialize()
{
	assert(hasIntensity() || hasDepth());
	if (hasIntensity() && hasDepth())
	{ 
		assert(intensity.size() == depth.size());
	}

	if (hasIntensity())
	{
		assert(intensity.type() == cv::DataType<IntensityType>::type && intensity.channels() == 1);
		width = intensity.cols;
		height = intensity.rows;
	}
	if (hasDepth())
	{
		assert(depth.type() == cv::DataType<DepthType>::type && depth.channels() == 1);
		width = depth.cols;
		height = depth.rows;
	}

	intensity_requires_calculation_ = true;
	depth_requires_calculation_ = true;
	pointcloud_requires_build_ = true;
}

bool RgbdImage::hasIntensity() const
{
	return !intensity.empty();
}

bool RgbdImage::hasDepth() const
{
	return !depth.empty();
}

bool RgbdImage::hasPointCloud() const
{
	return !pointcloud->empty();
}
void RgbdImage::buildPointCloud(int step /*= 1*/, float depth_th /*= std::numeric_limits<float>::max()*/)
{
	if (!pointcloud_requires_build_) return;
	assert(hasDepth());

	pointcloud.reset(new PointCloud);
	pointcloud->reserve(width*height);
	indices.reserve(width*height);
	acceleration.reserve(width*height);

	const PointCloudTemplate& cloud_template = camera_.pointcloudTemplate();
	calculateDerivatives();

	int num_point = 0;
	PointT point;
	IntensityAndDepth idg;
	for (int y = 1; y < height -1; y += step)
	{
		for (int x = 1; x < width -1; x += step)
		{
			int idx = y * width + x;
			const float* depth_ptr = depth.ptr<const float>(y, x);
			const float* zdx_ptr = depth_dx.ptr<const float>(y, x);
			const float* zdy_ptr = depth_dy.ptr<const float>(y, x);

			const float* intensity_ptr = intensity.ptr<const float>(y, x);
			const float* idx_ptr = intensity_dx.ptr<const float>(y, x);
			const float* idy_ptr = intensity_dy.ptr<const float>(y, x);

			const float* depth_ptr1 = depth.ptr<const float>(y, x-1);
			const float* depth_ptr2 = depth.ptr<const float>(y, x+1);
			const float* depth_ptr3 = depth.ptr<const float>(y-1, x);
			const float* depth_ptr4 = depth.ptr<const float>(y+1, x);

			int idx1 = y * width + x - 1;
			int idx2 = y * width + x + 1;
			int idx3 = (y - 1)*width + x;
			int idx4 = (y + 1)*width + x;
			//		3
			//	1   p	2
			//		4

			if (std::isfinite(*depth_ptr) && *depth_ptr < depth_th && std::isfinite(*depth_ptr1) && std::isfinite(*depth_ptr2) && std::isfinite(*depth_ptr3) && std::isfinite(*depth_ptr4))
			{
				point.getVector3fMap() = cloud_template.col(idx).head(3)*(*depth_ptr);
				point.r = point.g = point.b = static_cast<unsigned char>(*intensity_ptr);
				Eigen::Vector4f n = (cloud_template.col(idx4)*(*depth_ptr4) - cloud_template.col(idx3)*(*depth_ptr3)).cross3
									(cloud_template.col(idx2)*(*depth_ptr2) - cloud_template.col(idx1)*(*depth_ptr1));
				n.normalize();
				point.getNormalVector4fMap() = n;
				pointcloud->points.push_back(point);

				idg.i = *intensity_ptr;
				idg.d = *depth_ptr;
				idg.idx = *idx_ptr;
				idg.idy = *idy_ptr;
				idg.zdx = *zdx_ptr;
				idg.zdy = *zdy_ptr;
				idg.curvature = 0.f;

				acceleration.push_back(idg);
				indices.push_back(num_point);
				num_point++;
			}
		}
	}
	pointcloud_requires_build_ = false;
}

void RgbdImage::samplePointCloud(int num_sample)
{
	if (!pointcloud_requires_sample_ ||pointcloud->size() < 1.2 *num_sample) return;
	
	pcl::NormalSpaceSampling<PointT, PointT> normal_sample;
	normal_sample.setInputCloud(pointcloud);
	normal_sample.setNormals(pointcloud);
	normal_sample.setBins(4, 4, 4);
	normal_sample.setSeed(0);
	normal_sample.setSample(num_sample);
	normal_sample.filter(indices);
	pointcloud_requires_sample_ = false;		
}

void RgbdImage::computePointsJwJz()
{
	if (!jwjz_requires_compute_) return;
	jws.resize(indices.size());
	jzs.resize(indices.size());
	int num = 0;
	for (const auto &idx : indices)
	{
		const Eigen::Vector4f &p = pointcloud->points[idx].getVector4fMap();
		Matrix2x6 Jw;
		Vector6 Jz;

		//compute jw
		NumType z = 1.0f / p(2);
		NumType z_sqr = 1.0f / (p(2)*p(2));

		Jw(0, 0) = z;
		Jw(0, 1) = 0.0f;
		Jw(0, 2) = -p(0) * z_sqr;
		Jw(0, 3) = Jw(0, 2) * p(1);//j(0, 3) = -p(0) * p(1) * z_sqr;
		Jw(0, 4) = 1.0f - Jw(0, 2) * p(0);//j(0, 4) =  (1.0 + p(0) * p(0) * z_sqr);
		Jw(0, 5) = -p(1) * z;

		Jw(1, 0) = 0.0f;
		Jw(1, 1) = z;
		Jw(1, 2) = -p(1) * z_sqr;
		Jw(1, 3) = -1.0f + Jw(1, 2) * p(1); //j(1, 3) = -(1.0 + p(1) * p(1) * z_sqr);
		Jw(1, 4) = -Jw(0, 3); //j(1, 4) =  p(0) * p(1) * z_sqr;
		Jw(1, 5) = p(0) * z;

		// compute jz
		Jz(0) = 0.0;
		Jz(1) = 0.0;
		Jz(2) = 1.0;
		Jz(3) = p(1);
		Jz(4) = -p(0);
		Jz(5) = 0.0;
		jws[num] = Jw;
		jzs[num] = Jz;
		num++;
		//jws.push_back(Jw);
		//jzs.push_back(Jz);
	}

	jwjz_requires_compute_ = false;
}

float RgbdImage::computeErrorMetric(const RgbdImage& other, const Eigen::Matrix4d& transform,float corr_dist_threshold)
{
	float error = 0.f;
	float sqr_corr_dist = corr_dist_threshold * corr_dist_threshold;
	Eigen::Matrix4f transformf = transform.cast<float>();
	pcl::PointCloud<PointT> kpoints_transformed;
	pcl::transformPointCloud(*(other.kpoints), kpoints_transformed, transformf);
	std::vector<int> nn_index(1);
	std::vector<float> nn_distance(1);
	for (size_t i = 0; i < other.kpoints->size(); ++i)
	{
		kp_tree->nearestKSearch(kpoints_transformed.points[i], 1, nn_index, nn_distance);
		// compute the error
		error += nn_distance[0] <= sqr_corr_dist ? nn_distance[0] / sqr_corr_dist : 1.0;
	}
	return error;
}

cv::Point2f RgbdImage::point2UV(const PointT& pt)
{
	const IntrinsicMatrix& k = camera_.intrinsics();
	float x, y;
	x = pt.x / pt.z * k.fx() + k.cx();
	y = pt.y / pt.z * k.fy() + k.cy();
	return cv::Point2f(x, y);
}
RgbdImage::PointT RgbdImage::uv2Point(const cv::Point2f &uv)
{
	int x0 = static_cast<int>(std::floor(uv.x));
	int y0 = static_cast<int>(std::floor(uv.y));
	int x1 = x0 + 1;
	int y1 = y0 + 1;

	float x_weight = uv.x - x0;
	float y_weight = uv.y - y0;

	int idx1 = y0 * width + x0;
	int idx2 = y0 * width + x1;
	int idx3 = y1 * width + x1;
	int idx4 = y1 * width + x0;
	//	1 -- 2
	//  |	 |
	//  3 -- 4
	float d1 = depth.at<float>(y0, x0);
	float d2 = depth.at<float>(y0, x1);
	float d3 = depth.at<float>(y1, x1);
	float d4 = depth.at<float>(y1, x0);
	float c0 = intensity.at<float>(y0, x0);
	
	const PointCloudTemplate& cloud_template = camera_.pointcloudTemplate();
	const IntrinsicMatrix& K = camera_.intrinsics();
	Eigen::Vector4f n = (cloud_template.col(idx4)*d4 - cloud_template.col(idx2)*d2).cross3(cloud_template.col(idx3)*d3 - cloud_template.col(idx1)*d1);
	n.normalize();
	float d = (d1*x_weight + d2*(1 - x_weight) + d4*x_weight + d3*(1 - x_weight) +
		d1*y_weight + d4*(1 - y_weight) + d2*y_weight + d3*(1 - y_weight)) * 0.25f;
	PointT pt;
	pt.z = d;
	pt.x = (uv.x - K.cx()) * d / K.fx();
	pt.y = (uv.y - K.cy()) * d / K.fy();
	pt.r = pt.g = pt.b = static_cast<unsigned char>(c0);
	pt.getNormalVector4fMap() = n;
	return pt;
}
void RgbdImage::calculateDerivatives()
{
	calculateIntensityDerivatives();
	calculateDepthDerivatives();
}
bool RgbdImage::calculateIntensityDerivatives()
{
	if (!intensity_requires_calculation_) return false;
	assert(hasIntensity());

	calculateDerivativeX<IntensityType>(intensity, intensity_dx);
	calculateDerivativeYSseFloat(intensity, intensity_dy);

	intensity_requires_calculation_ = false;
	return true;
}

void RgbdImage::calculateDepthDerivatives()
{
	if (!depth_requires_calculation_) return;

	assert(hasDepth());

	calculateDerivativeX<DepthType>(depth, depth_dx);
	calculateDerivativeY<DepthType>(depth, depth_dy);

	depth_requires_calculation_ = false;
}

template<typename T>
void RgbdImage::calculateDerivativeX(const cv::Mat& img, cv::Mat& result)
{
	result.create(img.size(), img.type());
	for (int y = 0; y < img.rows; ++y)
	{
		for (int x = 0; x < img.cols; ++x)
		{
			int prev = std::max(x - 1, 0);
			int next = std::min(x + 1, img.cols - 1);

			result.at<T>(y, x) = (T)(img.at<T>(y, next) - img.at<T>(y, prev)) * 0.5f;
		}
	}
}

template<typename T>
void RgbdImage::calculateDerivativeY(const cv::Mat& img, cv::Mat& result)
{
	result.create(img.size(), img.type());
	for (int y = 0; y < img.rows; ++y)
	{
		for (int x = 0; x < img.cols; ++x)
		{
			int prev = std::max(y - 1, 0);
			int next = std::min(y + 1, img.rows - 1);

			result.at<T>(y, x) = (T)(img.at<T>(next, x) - img.at<T>(prev, x)) * 0.5f;
		}
	}
}

bool RgbdImage::inImage(const float& x, const float& y) const
{
	return x >= 0 && x < width && y >= 0 && y < height;
}
Vector8f RgbdImage::accelerationAt(const int &x, const int &y) const
{
	const float* depth_ptr = depth.ptr<const float>(y, x);
	const float* zdx_ptr = depth_dx.ptr<const float>(y, x);
	const float* zdy_ptr = depth_dy.ptr<const float>(y, x);

	const float* intensity_ptr = intensity.ptr<const float>(y, x);
	const float* idx_ptr = intensity_dx.ptr<const float>(y, x);
	const float* idy_ptr = intensity_dy.ptr<const float>(y, x);

	Vector8f data;
	data[0] = *intensity_ptr;
	data[1] = *depth_ptr;
	data[2] = *idx_ptr;
	data[3] = *idy_ptr;
	data[4] = *zdx_ptr;
	data[5] = *zdy_ptr;
	data[6] = 0.f;
	data[7] = 0.f;
	return data;
}

int RgbdImage::detectKeyPoint(const std::string &detectorType, const std::string& descriptorType, int maxKeypointNum)
{
	if (!keys.empty())
		return (int)keys.size();

	cv::Mat grey;

	intensity.convertTo(grey, CV_8U);

	if (!descriptorType.compare("SURF"))
	{
		double threshold = 2.0;
		cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(threshold,4,2,true);
		detector->detect(grey, keys);
	}
	else if (!descriptorType.compare("SIFT"))
	{
		int nfeatures = 1;
		cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(nfeatures);
		detector->detect(grey, keys);
	}
	else if (!descriptorType.compare("FAST"))
	{
		cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(3);
		detector->detect(grey, keys);
	}
	else{
		double threshold = 2.0;
		cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(threshold, 4, 2, true);
		detector->detect(grey, keys);
	}

	// remove key point which depth val is invalid
	keys.erase(std::remove_if(keys.begin(), keys.end(), IsInvalidKeyPoint(depth)), keys.end());
	int kpt_size = static_cast<int>(keys.size());
	if (kpt_size > maxKeypointNum)
	{
		std::sort(keys.begin(), keys.end(), [&](const cv::KeyPoint& p1, const cv::KeyPoint& p2)->bool
		{
			return p1.response > p2.response;
		});
		keys.resize(maxKeypointNum);
	}

	// generate corresponding 3d points
	kpoints->resize(keys.size());
	for (size_t i = 0; i < keys.size(); ++i)
	{
		kpoints->points[i] = uv2Point(keys[i].pt);
	}
	kp_tree->setInputCloud(kpoints);

	// compute keypoint's descriptor
	if (!descriptorType.compare("SURF"))
	{
		cv::Ptr<cv::xfeatures2d::SurfDescriptorExtractor> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
		extractor->compute(grey, keys, keys_desc);
	}else if (!descriptorType.compare("SIFT"))
	{
		cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
		extractor->compute(grey, keys, keys_desc);
	}else if (!descriptorType.compare("ORB"))
	{
		cv::Ptr<cv::ORB> extractor = cv::ORB::create();
		extractor->compute(grey, keys, keys_desc);
	}
	else
	{
		cv::Ptr<cv::xfeatures2d::SurfDescriptorExtractor> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
		extractor->compute(grey, keys, keys_desc);
	}
	return static_cast<int>(keys.size());			
}

void RgbdImage::createFlannIndex()
{
	if (flann_index.empty())
	{
		flann_index = new cv::flann::Index(keys_desc, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
	}
}

/*
	RgbdImagePyramid
*/

RgbdImagePyramid::RgbdImagePyramid(RgbdCameraPyramid& camera, const cv::Mat& intensity, const cv::Mat& depth)
	:camera_(camera)
{
	levels_.push_back(camera_.level(0).create(intensity, depth));
}
RgbdImagePyramid::~RgbdImagePyramid()
{

}
void RgbdImagePyramid::build(const int num_levels)
{
	if (levels_.size() >= num_levels)return;
		
	int first = static_cast<int>(levels_.size());

	for (int idx = first; idx < num_levels; ++idx)
	{
		levels_.push_back(camera_.level(idx).create());
		Interpolation::pyrDownMeanSmooth<IntensityType>(levels_[idx - 1]->intensity, levels_[idx]->intensity);

		Interpolation::pyrDownSubsample<float>(levels_[idx - 1]->depth, levels_[idx]->depth);

		levels_[idx]->initialize();
	}
}

RgbdImage& RgbdImagePyramid::level(size_t idx)
{
	assert(idx < levels_.size());
	return *levels_[idx];
}

double RgbdImagePyramid::timestamp() const
{
	return !levels_.empty() ? levels_[0]->timestamp : 0.0;
}
} // namespace dvo