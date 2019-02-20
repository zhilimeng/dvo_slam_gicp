#include <dvo/core/ipoint_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/normal_space.h>

namespace dvo
{
//////////////////////////////////////////////////////////////////////////
IPointCloud::Ptr IPointCloud::create(const PointCloudPtr &cloud, const Matrix4 &pose /*= Matrix4::Identity()*/)
{
	IPointCloud::Ptr result(new IPointCloud(cloud,pose));
	return result;
}


//////////////////////////////////////////////////////////////////////////
void IPointCloud::computeNormals(float search_radius)
{
	if (!normal_requires_build_) return;
	std::vector<int> nn_indices;
	std::vector<float> nn_dists;

	for (auto& pt : cloud_->points)
	{
		Eigen::Vector4f plane_parameters;
		float curvature;
		if (tree_->radiusSearch(pt,search_radius,nn_indices,nn_dists) == 0 ||
			!pcl::computePointNormal(*cloud_,nn_indices,plane_parameters,curvature))
		{
			pt.normal[0] = pt.normal[1] = pt.normal[2] = pt.curvature = std::numeric_limits<float>::quiet_NaN();
			cloud_->is_dense = false;
			continue;
		}
		else
		{
			pt.normal[0] = plane_parameters[0];
			pt.normal[1] = plane_parameters[1];
			pt.normal[2] = plane_parameters[2];
			pt.curvature = curvature;
		}
		pcl::flipNormalTowardsViewpoint(pt, 0, 0, 0, plane_parameters[0], plane_parameters[1], plane_parameters[2]);
	}


}

void IPointCloud::samplePointCloud(int num_samples)
{
	if (!indices_requires_sample_ || cloud_->size() < num_samples)
		return; 
	indices_->clear();
	pcl::NormalSpaceSampling<PointT, PointT> normal_space;
	normal_space.setInputCloud(cloud_);
	normal_space.setNormals(cloud_);
	normal_space.setBins(4, 4, 4);
	normal_space.setSample(num_samples);
	normal_space.filter(*indices_);
}

//////////////////////////////////////////////////////////////////////////
float IPointCloud::computeOverlap(const IPointCloud::Ptr& another, const Matrix4 &trans, float distance_threshold)
{
	float max_corr_sqr_dist = distance_threshold * distance_threshold;
	pcl::PointCloud<PointT> transformed_cloud;
	transformed_cloud.reserve(another->cloud()->size());
	for (size_t i = 0; i < another->cloud()->size(); i += 10)
	{
		const PointT &src = another->cloud()->points[i];
		PointT tgt = src;
		tgt.x = trans(0, 0) * src.x + trans(0, 1) * src.y + trans(0, 2)*src.z + trans(0, 3);
		tgt.y = trans(1, 0) * src.x + trans(1, 1) * src.y + trans(1, 2)*src.z + trans(1, 3);
		tgt.z = trans(2, 0) * src.x + trans(2, 1) * src.y + trans(2, 2)*src.z + trans(2, 3);
		transformed_cloud.push_back(tgt);
	}
	std::vector<int> nn_index(1);
	std::vector<float> nn_distance(1);
	int nr = 0;
	int nvalid = 0;
	for (size_t i = 0; i < transformed_cloud.size(); ++i)
	{
		tree_->nearestKSearch(transformed_cloud.points[i], 1, nn_index, nn_distance);
		if (nn_distance[0] < max_corr_sqr_dist)
			++nr;
		nvalid++;
	}
	return static_cast<float>(nr) / static_cast<float>(nvalid);
}

float IPointCloud::computeAverageSpacing(int k)
{
	float sum_spacings = 0.f;
	std::vector<int> nn_indices;
	std::vector<float> nn_dists;
	for (const auto &point : cloud_->points)
	{
		tree_->nearestKSearch(point, k + 1, nn_indices, nn_dists);
		float sum_spacing = 0.f;
		for (int i = 0; i < k + 1; ++i)
			sum_spacing += std::sqrt(nn_dists[i]);
		sum_spacings += sum_spacing / static_cast<float>(k);
	}
	return sum_spacings / static_cast<float>(cloud_->size());
}

//////////////////////////////////////////////////////////////////////////
Eigen::Vector4d IPointCloud::centroid()
{
	return pose_ * centroid_;
}

//////////////////////////////////////////////////////////////////////////
Eigen::Vector3d IPointCloud::direction()
{
	return pose_.block<3,1>(0,3) - (pose_ * centroid_).head(3);
}

//////////////////////////////////////////////////////////////////////////
void IPointCloud::id(int idx)
{
	id_ = idx;
}

//////////////////////////////////////////////////////////////////////////
IPointCloud::IPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, const Matrix4 &pose /*= Matrix4::Identity()*/)
	:cloud_(cloud),pose_(pose), tree_(),indices_(new std::vector<int>),  normal_requires_build_(true)
	, indices_requires_sample_(true)
{
	// build kdtree
	tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
	tree_->setInputCloud(cloud_);

	// compute centroid;
	size_t num_points = cloud_->size();
	centroid_.setZero();
	int n_valid = 0;
	for (size_t i = 0; i < num_points; i += 10)
	{
		centroid_[0] += cloud_->points[i].x;
		centroid_[1] += cloud_->points[i].y;
		centroid_[2] += cloud_->points[i].z;
		n_valid++;
	}
	centroid_ /= static_cast<double>(n_valid);
	centroid_[3] = 1;

	// compute indices;
	indices_->resize(num_points);
	for (size_t i = 0; i < num_points; ++i)
	{
		(*indices_)[i] = i;
	}
//		indices_requires_sample_ = false;
}

} // namespace dvo