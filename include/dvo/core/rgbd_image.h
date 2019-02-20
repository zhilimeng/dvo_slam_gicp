#ifndef DVO_RGBD_IMAGE_H
#define DVO_RGBD_IMAGE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <dvo/core/data_types.h>
#include <dvo/core/rgbd_camera.h>
namespace dvo
{
class DVO_EXPORTS RgbdImage
{
public:
	RgbdImage(const RgbdCamera& camera);
	virtual ~RgbdImage();

	typedef cv::Vec<float, 8> Vec8f;
	typedef pcl::PointXYZRGBNormal PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef std::vector<Matrix2x6, Eigen::aligned_allocator<Matrix2x6> > JwVector;
	typedef std::vector<Vector6, Eigen::aligned_allocator<Vector6> > JzVector;
public:
	cv::Mat intensity;
	cv::Mat intensity_dx;
	cv::Mat intensity_dy;

	cv::Mat depth;
	cv::Mat depth_dx;
	cv::Mat depth_dy;


	PointCloudPtr pointcloud;
	std::vector<int> indices;        
	IntensityAndDepthVector acceleration;
	JwVector jws;
	JzVector jzs;

	int width;
	int height;
	double timestamp;
public:
	std::vector<cv::KeyPoint> keys;
	pcl::PointCloud<PointT>::Ptr kpoints;
	pcl::KdTree<PointT>::Ptr kp_tree;
	cv::Mat keys_desc;
	cv::Ptr<cv::flann::Index> flann_index;
public:
	const RgbdCamera& camera() const;
	bool hasIntensity() const;
	bool hasDepth() const;
	bool hasPointCloud() const;

	void initialize();
	void calculateDerivatives();
	bool calculateIntensityDerivatives();
	void calculateDepthDerivatives();


	void buildPointCloud(int step = 1,float depth_th = std::numeric_limits<float>::max());
	void samplePointCloud(int num_sample);
	void computePointsJwJz();
	/* * \brief param transform is the transformation matrix from other to this  */
	float computeErrorMetric(const RgbdImage& other, const Eigen::Matrix4d& transform, float corr_dist_threshold);

	cv::Point2f point2UV(const PointT& pt);
	PointT uv2Point(const cv::Point2f& uv);
	bool inImage(const float& x, const float& y) const;

	Vector8f accelerationAt(const int &x,const int &y) const;
public:
	int detectKeyPoint(const std::string &detectorType, const std::string& descriptorType, int maxKeypointNum);
	void createFlannIndex();
private:
	bool intensity_requires_calculation_, depth_requires_calculation_, pointcloud_requires_build_;
	bool pointcloud_requires_sample_;
	bool jwjz_requires_compute_;
	const RgbdCamera& camera_;

	template<typename T>
	void calculateDerivativeX(const cv::Mat& img, cv::Mat& result);

	template<typename T>
	void calculateDerivativeY(const cv::Mat& img, cv::Mat& result);

	void calculateDerivativeYSseFloat(const cv::Mat& img, cv::Mat& result);

};

class DVO_EXPORTS RgbdImagePyramid
{
public:
	typedef boost::shared_ptr<dvo::RgbdImagePyramid> Ptr;

	RgbdImagePyramid(RgbdCameraPyramid& camera, const cv::Mat& intensity, const cv::Mat& depth);
	virtual ~RgbdImagePyramid();

	void build(const int num_levels);
	RgbdImage& level(size_t idx);

	double timestamp() const;
	int id(){ return image_id_; }
	void id(int image_id){ image_id_ = image_id; }
	std::string name(){ return name_; }
	void name(const std::string &name){ name_ = name; }
private:
	int image_id_;
	std::string name_;
	RgbdCameraPyramid& camera_;
	std::vector<RgbdImagePtr> levels_;
};

}
#endif


