/*!
 * \file icp.h
 * \brief  ICP class and ICPWithNormals class to align  point clouds
 *	see <pcl/registration/icp.h>
 * \author mengzhili
 * \date 2018/11/20 14:03
 *
 * 
 */
#ifndef DVO_ICP
#define DVO_ICP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
namespace dvo
{
template<typename PointT>
void compute3DCentroid(const std::vector<PointT>& cloud, int n, int step, Eigen::Vector4f& centroid)
{
	centroid.setZero();
	if (cloud.empty())
		return;
	int n_valid = 0;
	int n_step = step < 1 ? 1 : step;
	for (int i = 0; i < n; i += n_step)
	{
		centroid[0] += cloud[i].x;
		centroid[1] += cloud[i].y;
		centroid[2] += cloud[i].z;
		++n_valid;
	}
}
class  ICPWithNormals
{
public:
	typedef Eigen::Matrix<double, 4, 4> Matrix4d;
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vec4Xf;

	typedef boost::shared_ptr<ICPWithNormals> Ptr;
	typedef boost::shared_ptr<const ICPWithNormals> ConstPtr;

	typedef pcl::PointXYZRGBNormal PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef PointCloud::ConstPtr PointCloudConstPtr;
	typedef pcl::KdTree<PointT>::Ptr KdTreePtr;

	/* * \brief  Empty constructor*/
	ICPWithNormals():
		input_()
		, target_()
		, tree_()
		, max_iterations_(50)
		, max_angle_(45.0)
		, corr_dist_threshold_(0.01)
		, precision_(1e-9)
		, final_transformation_(Matrix4d::Identity())
		, source_indices_(new std::vector<int>)
		, corrs_(new pcl::Correspondences)
	{
	}

	/* * \brief  Empty destructor*/
	~ICPWithNormals(){}
	inline void setInputSource(const PointCloudConstPtr &cloud){ input_ = cloud; }
	inline void setInputTarget(const PointCloudConstPtr &cloud){ target_ = cloud; tree_.reset(); }
	inline void setTargetTree(const KdTreePtr &target_tree){ tree_ = target_tree; }
	inline void setSourceIndices(const pcl::IndicesPtr &source_indices){ source_indices_ = source_indices; }

	inline void setMaxIterations(int max_iterations){ max_iterations_ = max_iterations; }
	inline void setPrecision(float precision) { precision_ = precision; }
	inline void setMaxCorrespondenceDistance(float corr_dist_threshold){ corr_dist_threshold_ = corr_dist_threshold; }	
	inline void setMaxAngle(float angle){ max_angle_ = angle; }
public:
    bool align(const Matrix4d &init_transformation);
	inline const Matrix4d& getFinalTransformation(){ return (final_transformation_); }
	inline const Matrix6d& getInformation() { return information_; }
	pcl::CorrespondencesPtr getCorrespondences(){ return (corrs_); }
private:
	inline bool initCompute();
	void computeWeights(const std::vector<float>& residuals, int n, float mean, float precison, std::vector<float>& weights);
	float computeScale(const std::vector<float>& residuals, int n, const std::vector<float>& weights, float mean);
	float computeCompleteDataLogLikelihood(const std::vector<float>& residuals, int n, const std::vector<float>& weights, float mean, float precision);
	void computeATAandATb(const Vec4Xf& xyz_s, const Vec4Xf& xyz_t, const Vec4Xf& nor_t, int n, float scale, Matrix6d& ATA, Vector6d& ATb);
private:
	PointCloudConstPtr input_;
	PointCloudConstPtr target_;
	KdTreePtr tree_;
	pcl::IndicesPtr source_indices_;

	int max_iterations_;
	float corr_dist_threshold_;
	float max_angle_;
	float precision_;

	Matrix4d final_transformation_;
	Matrix6d information_;
	pcl::CorrespondencesPtr corrs_;

};
} // namespace dvo
#endif