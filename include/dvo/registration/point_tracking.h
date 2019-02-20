#ifndef DVO_POINT_TRACKING_H
#define DVO_POINT_TRACKING_H
#include <pcl/common/common_headers.h>
#include <dvo/core/ipoint_cloud.h>
#include <dvo/registration/icp.h>

namespace dvo
{
class DVO_EXPORTS PointTracker
{
public:
	typedef boost::shared_ptr<PointTracker> Ptr;
	typedef Eigen::Matrix<double, 4, 4> Matrix4;
	typedef Eigen::Matrix<double, 6, 6> Matrix6;
	PointTracker(){}
	~PointTracker(){}

	struct Config
	{
		int max_iterations;
		float precision;
		float corr_dist_threshold;
		float max_angle;
	};

	struct Result 
	{
		Matrix4 transformation;
		Matrix6 information;
		pcl::CorrespondencesPtr corrs;
	};

	void configure(const PointTracker::Config &config){ cfg_ = config; }
	void setPrecision(float precison) { cfg_.precision = precison; }
	void setMaxIterations(float max_iterations) { cfg_.max_iterations = max_iterations; }
	void setMaxCorrespondenceDistance(float max_corr_dist) { cfg_.corr_dist_threshold = max_corr_dist; }
	void setMaxAngle(float max_anlge) { cfg_.max_angle = max_anlge; }

	bool align(IPointCloud::Ptr &target_cloud, IPointCloud::Ptr &source_cloud, Matrix4& final_trans, const Matrix4& init_trans = Matrix4::Identity());
	bool align(IPointCloud::Ptr &target_cloud, IPointCloud::Ptr &source_cloud, Result& result, const Matrix4& init_trans = Matrix4::Identity());
private:
	PointTracker::Config cfg_;
};
}
#endif