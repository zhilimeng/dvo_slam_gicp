#ifndef DVO_SCALABLE_TSDF_VOLUME_H
#define DVO_SCALABLE_TSDF_VOLUME_H

#include <memory>
#include <unordered_map>
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/polygon_mesh.h>
#include <dvo/surface/helper.h>
#include <dvo/core/rgbd_image.h>

namespace dvo
{
	class UniformTSDFVolume;
	class DVO_EXPORTS ScalableTSDFVolume
	{
	public:
		typedef pcl::PointXYZRGBNormal PointT;
		typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudT;
		struct VolumeUnit
		{
			VolumeUnit() :volume_(NULL){}

			boost::shared_ptr<UniformTSDFVolume> volume_;
			Eigen::Vector3i index_;
		};
		void setTsdfThreshold(float tsdf_threshold)
		{
			tsdf_threshold_ = tsdf_threshold;
		}

	public:
		ScalableTSDFVolume(float voxel_length, float sdf_trunc, bool with_color, int volume_unit_resolution = 16, int depth_sampling_stride = 4);
		~ScalableTSDFVolume();



		void reset();
		void integrate(RgbdImage& image, const IntrinsicMatrix& intrinsic, const Eigen::Matrix4f& extrinsic);
		void integrate(const PointCloudT& cloud, const Eigen::Matrix4f& extrinsic);
		void extractPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);
		void extractVoxelPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);
		void extractPolygonMesh(pcl::PolygonMesh &mesh);
	private:
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createPointCloudFromDepthImage(const cv::Mat &depth, const IntrinsicMatrix& K, const Eigen::Matrix4f& extrinsic, int step);

	public:
		float voxel_length_;
		float sdf_trunc_;
		bool with_color_;

		int volume_unit_resolution_;
		double volume_unit_length_;
		int depth_sampling_stride_;

		std::unordered_map<Eigen::Vector3i, VolumeUnit, hash_eigen::hash<Eigen::Vector3i> > volume_units_;
	private:
		float tsdf_threshold_;
	private:
		Eigen::Vector3i locateVolumeUnit(const Eigen::Vector3f &point)
		{
			return Eigen::Vector3i((int)std::floor(point(0) / volume_unit_length_), (int)std::floor(point(1) / volume_unit_length_), (int)std::floor(point(2) / volume_unit_length_));
		}

		boost::shared_ptr<UniformTSDFVolume> openVolumeUnit(const Eigen::Vector3i& index);

		Eigen::Vector3f getNormalAt(const Eigen::Vector3f &p);

		float getTSDFAt(const Eigen::Vector3f &p);
	};
}
#endif
