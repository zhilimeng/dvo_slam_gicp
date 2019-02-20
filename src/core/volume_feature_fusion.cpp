#include <dvo/core/volume_feature_fusion.h>
namespace dvo
{
bool VolumeFeatureFusion::integrate(const PointCloud &cloud, const cv::Mat &features, const Matrix4 &pose /* = Matrix4::Identity() */)
{
	if (cloud.size() != features.rows)
	{
		std::cerr << "point size and descriptors size are not same!\n";
		return false;
	}
	int num_points = static_cast<int>(cloud.size());
	Eigen::Matrix4f posef = pose.cast<float>();
	for (int i = 0; i < num_points; ++i)
	{
		PointT pt_w = cloud.points[i];
		cv::Mat desc_w = features.row(i);

		pt_w.getVector4fMap() = posef * pt_w.getVector4fMap();
		pt_w.getNormalVector4fMap() = posef * pt_w.getNormalVector4fMap();

		Eigen::Vector3i idx = locateVolumeUnit(pt_w.getVector3fMap());
		auto iter = volume_units_.find(idx);
		if (iter == volume_units_.end())
		{
			volume_units_[idx] = std::make_pair(pt_w, desc_w);
		}
		else
		{
			const PointT &pt_old = iter->second.first;
			const cv::Mat &desc_old = iter->second.second;
			PointT pt_new = pt_old;
			cv::Mat desc_new = desc_old.clone();

			pt_new.getVector4fMap() = (pt_w.getVector4fMap() + pt_old.getVector4fMap()) * 0.5;
			pt_new.getNormalVector4fMap() = (pt_w.getNormalVector4fMap() + pt_old.getNormalVector4fMap()) * 0.5;
			for (int k = 0; k < descriptor_size_; ++k)
			{
				desc_new.at<float>(k) = (desc_w.at<float>(k) + desc_old.at<float>(k)) * 0.5;
			}
			volume_units_[idx] = std::make_pair(pt_new, desc_new);
		}

	}
	return true;
}

void VolumeFeatureFusion::extractPointAndDescriptors(PointCloud &cloud, cv::Mat &descriptors)
{
	size_t num_points = volume_units_.size();
	descriptors.create(num_points, descriptor_size_, CV_32FC1);
	int idx = 0;
	for (const auto &unit : volume_units_)
	{
		cloud.points.push_back(unit.second.first);
		for (int k = 0; k < descriptor_size_; ++k)
		{
			descriptors.at<float>(idx, k) = unit.second.second.at<float>(k);
		}
	}
}

} // namespace dvo