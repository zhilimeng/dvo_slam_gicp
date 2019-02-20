/*!
 * \file ipoint_cloud.h
 * \brief IPointCloud class
 *
 * \author mengzhili
 * \date 2018/11/20 16:41
 *
 * 
 */
#ifndef DVO_POINT_CLOUD_H
#define DVO_POINT_CLOUD_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <dvo/core/dvo_exports.h>
namespace dvo
{
class DVO_EXPORTS IPointCloud
{
public:
	typedef boost::shared_ptr<IPointCloud> Ptr;
	typedef boost::shared_ptr<const IPointCloud> ConstPtr;

	typedef pcl::PointXYZRGBNormal PointT;
	typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
	typedef pcl::KdTree<PointT>::Ptr KdTreePtr;
	typedef Eigen::Matrix4d Matrix4;

	static IPointCloud::Ptr create(const PointCloudPtr &cloud, const Matrix4 &pose = Matrix4::Identity());
	~IPointCloud(){}

	/* * \brief  get point cloud*/
	PointCloudPtr cloud(){ return (cloud_); }

	/* * \brief  get point cloud kd tree*/
	KdTreePtr tree(){ return (tree_); }

	/* * \brief  get point cloud pose*/
	const Matrix4& pose(){ return (pose_); }

	/* * \brief  set point cloud pose*/
	void pose(const Matrix4& pose){ pose_ = pose; }

	/* * \brief get point cloud id */
	int id(){ return id_; }

	/* * \brief set point cloud id*/
	void id(int idx);

	/* * \brief get point cloud centroid */
	Eigen::Vector4d centroid();

	/* * \brief get point cloud view direction from camera pose */
	Eigen::Vector3d direction();

	/* * \brief get point indices ptr*/
	pcl::IndicesPtr indices(){ return indices_; }

	/* * \brief set point indices*/
	void indices(std::vector<int> vindices){ indices_.reset(new std::vector<int>(vindices)); indices_requires_sample_ = false; }
public:
	void computeNormals(float search_radius);
	void samplePointCloud(int num_samples);
	float computeOverlap(const IPointCloud::Ptr& another, const Matrix4 &trans, float distance_threshold);
	float computeAverageSpacing(int k);
private:
	PointCloudPtr cloud_;
	KdTreePtr tree_;
	Matrix4 pose_;
	Eigen::Vector4d centroid_; 
	pcl::IndicesPtr indices_;
	int id_;

	bool normal_requires_build_;
	bool indices_requires_sample_;

private:
	IPointCloud(const PointCloudPtr &cloud, const Matrix4 &pose = Matrix4::Identity());
};
typedef boost::shared_ptr<IPointCloud> IPointCloudPtr;
typedef boost::shared_ptr<const IPointCloud> IPointCloudConstPtr;
}
#endif