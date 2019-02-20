#ifndef DVO_UNIFORM_TSDF_VOLUME_H
#define DVO_UNIFORM_TSDF_VOLUME_H

#include <pcl/PolygonMesh.h>
#include <pcl/geometry/polygon_mesh.h>
#include <dvo/core/rgbd_image.h>

namespace dvo
{
	class DVO_EXPORTS  UniformTSDFVolume
	{
	public:
		typedef pcl::PointXYZRGBNormal PointT;
		UniformTSDFVolume(float length, int resolution, float sdf_trunc, bool with_color, const Eigen::Vector3f &origon = Eigen::Vector3f::Zero());
		~UniformTSDFVolume();
	public:
		void reset();
		void integrate(const RgbdImage& image, const IntrinsicMatrix& intrinsic, const Eigen::Matrix4f& extrinsic);
		void integrate(const PointT& point, const Eigen::Matrix4f& extrinsic);
		void integrateWithDepthToCameraDistanceMultiplier(const RgbdImage &image, const IntrinsicMatrix &intrinsic, const Eigen::Matrix4f &extrinsic, const cv::Mat& depth_to_camera_distance_multiplier);
		void extractPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);
		void extractVoxelPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);
		void extractPolygonMesh(pcl::PolygonMesh &mesh);

		int indexOf(int x, int y, int z) const
		{
			return x * resolution_ * resolution_ + y * resolution_ + z;
		}
		int indexOf(const Eigen::Vector3i &xyz) const
		{
			return indexOf(xyz(0), xyz(1), xyz(2));
		}


	public:
		float voxel_length_;
		float sdf_trunc_;
		bool with_color_;

		Eigen::Vector3f origon_;
		float length_;
		int resolution_;
		int voxel_num_;
		std::vector<float> tsdf_;
		std::vector<Eigen::Vector3f> color_;
		std::vector<float> weight_;
	private:
		Eigen::Vector3f getNormalAt(const Eigen::Vector3f &p);
		double getTSDFAt(const Eigen::Vector3f &p);

	};
	typedef boost::shared_ptr<UniformTSDFVolume> UniformTSDFVolumePtr;
}
#endif
