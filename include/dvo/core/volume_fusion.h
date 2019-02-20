#ifndef  DVO_VOLUME_FUSION_H
#define DVO_VOLUME_FUSION_H

#include <unordered_map>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/hash_eigen.h>
#include <dvo/core/utility.h>
namespace dvo
{
class DVO_EXPORTS VolumeFusion
{
public:
	typedef pcl::PointXYZRGBNormal PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	VolumeFusion();
	~VolumeFusion();
public:
	void setVoxelLength(float voxleLength) { voxel_length_ = voxleLength; }
	void integrate(const PointCloud &cloud, const Eigen::Matrix4d& pose = Eigen::Matrix4d::Identity());
	void extractPointCloud(pcl::PointCloud<PointT>& cloud);

public:
	Eigen::Vector3i locateVolumeUnit(const Eigen::Vector3f& point)
	{
		return Eigen::Vector3i(std::floor(point(0) / voxel_length_), std::floor(point(1) / voxel_length_), std::floor(point(2) / voxel_length_));
	}

private:
	float voxel_length_;
	std::unordered_map<Eigen::Vector3i, PointT, hash_eigen::hash<Eigen::Vector3i>> volume_units_;

};
} // namespace dvo
#endif