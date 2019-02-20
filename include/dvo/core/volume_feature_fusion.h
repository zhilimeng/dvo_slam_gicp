#ifndef DVO_COMMON_VOLUME_FEATURE_FUSION_H
#define DVO_COMMON_VOLUME_FEATURE_FUSION_H
#include <unordered_map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

#include <dvo/core/dvo_exports.h>
#include <dvo/core/hash_eigen.h>
namespace dvo
{
	class DVO_EXPORTS VolumeFeatureFusion
	{
	public:
		typedef pcl::PointXYZRGBNormal PointT;
		typedef pcl::PointCloud<PointT> PointCloud;
		typedef Eigen::Matrix4d Matrix4;
		VolumeFeatureFusion() :voxel_length_(0.001), descriptor_size_(64) {}
		~VolumeFeatureFusion() {}
	public:
		void setVoxelLength(float voxel_length) { voxel_length_ = voxel_length; }
		void setDescriptorSize(int descriptor_size) { descriptor_size_ = descriptor_size; }
		bool integrate(const PointCloud &cloud, const cv::Mat &features, const Matrix4 &pose = Matrix4::Identity());
		void extractPointAndDescriptors(PointCloud &cloud, cv::Mat &descriptors);
	private:
		Eigen::Vector3i locateVolumeUnit(const Eigen::Vector3f &point)
		{
			return Eigen::Vector3i(std::floor(point(0) / voxel_length_), std::floor(point(1) / voxel_length_), std::floor(point(2) / voxel_length_));
		}
	private:
		float voxel_length_;
		int descriptor_size_;
		std::unordered_map<Eigen::Vector3i, std::pair<PointT, cv::Mat>, hash_eigen::hash<Eigen::Vector3i> > volume_units_;
	};
} // namespace dvo
#endif
