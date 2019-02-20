/*!
 * \file feature_tracking.h
 * \brief rgbd feature tracking using surf feature
 *
 * \author mengzhili
 * \date 2018/11/20 15:05
 *
 * 
 */
#ifndef  DVO_FEATURE_TRACKING_H
#define  DVO_FEATURE_TRACKING_H

#include <cstdlib>
#include <dvo/core/rgbd_image.h>
#include <pcl/correspondence.h>
#include <pcl/registration/transformation_estimation_svd.h>
namespace dvo
{

class DVO_EXPORTS FeatureTracker
{
public:
	typedef boost::shared_ptr<FeatureTracker> Ptr;
	typedef pcl::PointXYZRGBNormal PointT;
	typedef pcl::registration::TransformationEstimation<PointT, PointT, double> TransformationEstimation;
	typedef Eigen::Matrix4d Matrix4;
	typedef Eigen::Matrix<double, 6, 6> Matrix6;

	struct Config
	{
		std::string detector_type;
		std::string descriptor_type;

		int max_keypoints_num;
		int knn;
		int max_iterations;
		int min_valid_matches;
		float max_descriptor_ratio;
		float max_keypoint_2d_dist;
		float min_keypoint_pair3d_dist;
		float max_keypoint_pair3d_dev_dist;
		float max_corr_dist;
		bool verbose;

		Config():detector_type("SURF"),descriptor_type("SURF"),max_keypoints_num(1000),knn(3),
			max_iterations(0), min_valid_matches(6),max_descriptor_ratio(0.35), max_keypoint_2d_dist(200),
			min_keypoint_pair3d_dist(0.01),max_keypoint_pair3d_dev_dist(0.0015),verbose(false)
		{

		}
	};

	struct Result
	{
		Matrix4 transformation;
		Matrix6 information;
		pcl::CorrespondencesPtr corrs;
	};


	FeatureTracker():transformation_estimation_(new pcl::registration::TransformationEstimationSVD<PointT, PointT, double>())
	{
	}
	void configure(const FeatureTracker::Config& cfg) { cfg_ = cfg; }

	const dvo::FeatureTracker::Config& configuration() const { return cfg_; }

	bool align(dvo::RgbdImage &ref, dvo::RgbdImage &mov, Matrix4& final_trans, const Matrix4& init_trans = Matrix4::Identity());

	bool align(dvo::RgbdImage &ref, dvo::RgbdImage &mov, Result& result, const Matrix4& init_trans = Matrix4::Identity());
private:
	struct CoMatch
	{
		cv::DMatch match;
		int valid_pair_num;
	};
private:
	int findMatches(dvo::RgbdImage& ref, dvo::RgbdImage& mov, std::vector<cv::DMatch> &matches, const Matrix4& init_trans = Matrix4::Identity());
	void knnMatch(dvo::RgbdImage& ref,dvo::RgbdImage& mov, std::vector<std::vector<cv::DMatch> >& matches, int knn);
	void estimateTransformation(dvo::RgbdImage& ref, dvo::RgbdImage& mov, std::vector<cv::DMatch> &matches, Result& result);
	bool validateSegment(const PointT& p0, const PointT& p1, const PointT& q0, const PointT& q1, float threshold);
	inline int getRandomIndex(int n){ return (static_cast<int>(n * (rand() / (RAND_MAX + 1.0)))); }
private:
	Config cfg_;
	TransformationEstimation::Ptr transformation_estimation_;
};
}
#endif