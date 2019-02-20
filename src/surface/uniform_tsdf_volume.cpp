
#include <memory>
#include <unordered_map>
#include <pcl/surface/marching_cubes.h>
#include <dvo/surface/helper.h>
#include <dvo/surface/uniform_tsdf_volume.h>

namespace dvo
{

	UniformTSDFVolume::UniformTSDFVolume(float length, int resolution, float sdf_trunc, bool with_color, const Eigen::Vector3f &origon /*= Eigen::Vector3f::Zero()*/)
		:voxel_length_(length / resolution), sdf_trunc_(sdf_trunc), with_color_(with_color),origon_(origon), length_(length), resolution_(resolution),
		voxel_num_(resolution*resolution*resolution), tsdf_(voxel_num_, 0), color_(with_color ? voxel_num_ : 0, Eigen::Vector3f::Zero()), weight_(voxel_num_,0.f)
	{

	}

	UniformTSDFVolume::~UniformTSDFVolume()
	{

	}

	void UniformTSDFVolume::reset()
	{
		std::memset(tsdf_.data(), 0, voxel_num_ * 4);
		std::memset(weight_.data(), 0, voxel_num_ * 4);
		if (with_color_)
			std::memset(color_.data(), 0, voxel_num_ * 12);
	}

	void UniformTSDFVolume::integrate(const RgbdImage& image, const IntrinsicMatrix& intrinsic, const Eigen::Matrix4f& extrinsic)
	{
		cv::Mat depth2CameraDistance = createDepthToCameraDistanceMultiplierImage(intrinsic, image.width, image.height);
		integrateWithDepthToCameraDistanceMultiplier(image, intrinsic, extrinsic, depth2CameraDistance);
	}


	void UniformTSDFVolume::integrate(const PointT& point, const Eigen::Matrix4f& extrinsic)
	{
		float half_voxel_length = 0.5 * voxel_length_;
		float sdf_trunc_inverse = 1.0f / sdf_trunc_;
		Eigen::Matrix4f extrinsic_scaled = extrinsic * voxel_length_;
		for (int x = 0; x < resolution_; ++x)
		{
			for (int y = 0; y < resolution_; ++y)
			{
				int idx_shift = x * resolution_ * resolution_ + y * resolution_;
				float *p_tsdf = (float *)tsdf_.data() + idx_shift;
				float *p_weight = (float *)weight_.data() + idx_shift;
				float *p_color = (float *)color_.data() + idx_shift * 3;

				Eigen::Vector4f voxel_pt_camera = extrinsic * Eigen::Vector4f(half_voxel_length + x * voxel_length_ + origon_(0),
					half_voxel_length + y * voxel_length_ + origon_(1), half_voxel_length + origon_(2), 1.0f);
				for (int z = 0; z < resolution_; ++z,
					voxel_pt_camera(0) += extrinsic_scaled(0, 2),
					voxel_pt_camera(1) += extrinsic_scaled(1, 2),
					voxel_pt_camera(2) += extrinsic_scaled(2, 2),
					p_tsdf++, p_weight++, p_color += 3)
				{
					float sdf = point.getNormalVector3fMap().dot(voxel_pt_camera.head(3) - point.getVector3fMap());
//					 sdf = (voxel_pt_camera.head(3) - point.getVector3fMap()).norm();
					if (sdf > -sdf_trunc_)
					{
							// integrate
						float tsdf = std::min(1.0f, sdf * sdf_trunc_inverse);
						*p_tsdf = ((*p_tsdf) * (*p_weight) + tsdf) / (*p_weight + 1.0f);
						if (with_color_)
						{
							float grey_color = static_cast<float>(point.getRGBVector3i().mean());

							p_color[0] = (p_color[0] * (*p_weight) + grey_color) / (*p_weight + 1.0f);
							p_color[1] = (p_color[1] * (*p_weight) + grey_color) / (*p_weight + 1.0f);
							p_color[2] = (p_color[2] * (*p_weight) + grey_color) / (*p_weight + 1.0f);
						}

						*p_weight += 1.0f;


					}
				}

			}
		}
	}

	void UniformTSDFVolume::extractPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud)
	{
		float half_voxel_length = 0.5 * voxel_length_;
		for (int x = 1; x < resolution_ - 1; ++x)
		{
			for (int y = 1; y < resolution_ - 1; ++y)
			{
				for (int z = 1; z < resolution_ - 1; ++z)
				{
					Eigen::Vector3i idx0(x, y, z);
					float w0 = weight_[indexOf(idx0)];
					float f0 = tsdf_[indexOf(idx0)];
					if (w0 != 0.0f && f0 < 0.98f && f0 >= -0.98f)
					{
						Eigen::Vector3f p0(half_voxel_length + voxel_length_ * x, half_voxel_length + voxel_length_*y, half_voxel_length + voxel_length_ *z);
						for (int i = 0; i < 3; ++i)
						{
							Eigen::Vector3f p1 = p0;
							p1(i) += voxel_length_;
							Eigen::Vector3i idx1 = idx0;
							idx1(i) += 1;
							if (idx1(i) < resolution_ -1)
							{
								float w1 = weight_[indexOf(idx1)];
								float f1 = tsdf_[indexOf(idx1)];
								if (w1 != 0.0f && f1 < 0.98f && f1 >= -0.98f && f0 * f1 < 0)
								{
									float r0 = std::fabs(f0);
									float r1 = std::fabs(f1);
									Eigen::Vector3f p = p0;
									p(i) = (p0(i)* r1 + p1(i)*r0) / (r0 + r1);
									pcl::PointXYZRGBNormal point;
									point.getVector3fMap() = p + origon_;
								}
							}
						}
					}
				}// end of z loop
			}// end of y loop
		}// end of x loop
	}

	void UniformTSDFVolume::extractVoxelPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud)
	{
		float half_voxel_length = voxel_length_ * 0.5;
		float *p_tsdf = (float *)tsdf_.data();
		float *p_weight = (float *)weight_.data();
		float *p_color = (float *)color_.data();
		for (int x = 0; x < resolution_; ++x)
		{
			for (int y = 0; y < resolution_; ++y)
			{
				Eigen::Vector3f pt(half_voxel_length + voxel_length_ * x, half_voxel_length + voxel_length_*y, half_voxel_length);
				for (int z = 0; z < resolution_; ++z, pt(2) += voxel_length_,p_tsdf++,p_weight++,p_color += 3)
				{
					if (*p_weight != 0.0f && *p_tsdf < 0.1f && *p_tsdf >= -0.1f)
					{
						pcl::PointXYZRGBNormal point;
						point.getVector3fMap() = pt + origon_;
						cloud.push_back(point);
					}

				}
			}
		}
	}

	void UniformTSDFVolume::extractPolygonMesh(pcl::PolygonMesh &mesh)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;

		float half_voxel_length = 0.5 * voxel_length_;

		std::unordered_map<Eigen::Vector4i, int, hash_eigen::hash<Eigen::Vector4i> > edgeindex_to_vertexindex;
		int edge_to_index[12];

		for (int x = 0; x < resolution_ - 1; ++x)
		{
			for (int y = 0; y < resolution_ - 1; ++y)
			{
				for (int z = 0; z < resolution_ - 1; ++z)
				{
					int cube_index = 0;
					float f[8];
					Eigen::Vector3f c[8];
					for (int i = 0; i < 8; ++i)
					{
						Eigen::Vector3i idx = Eigen::Vector3i(x, y, z) + shift[i];
						if (weight_[indexOf(idx)] == 0.f)
						{
							cube_index = 0;
							break;
						}
						else{
							f[i] = tsdf_[indexOf(idx)];
							if (f[i] < 0.f)
								cube_index |= (1 << i);
							if (with_color_)
								c[i] = color_[indexOf(idx)] / 255.0;

						}
					}
					if (cube_index == 0 || cube_index == 255)
						continue;
					for (int i = 0; i < 12; ++i)
					{
						if (pcl::edgeTable[cube_index] & (1 << i))
						{
							Eigen::Vector4i edge_index = Eigen::Vector4i(x, y, z, 0) + edge_shift[i];
							if (edgeindex_to_vertexindex.find(edge_index) == edgeindex_to_vertexindex.end())
							{
								edge_to_index[i] = static_cast<int>(cloud.points.size());
								edgeindex_to_vertexindex[edge_index] = static_cast<int>(cloud.points.size());

								Eigen::Vector3f pt(half_voxel_length + edge_index(0) * voxel_length_,
									half_voxel_length + edge_index(1) * voxel_length_,
									half_voxel_length + edge_index(2)*voxel_length_);

								float f0 = std::abs(f[edge_to_vert[i][0]]);
								float f1 = std::abs(f[edge_to_vert[i][1]]);

								pt(edge_index(3)) += f0 * voxel_length_ / (f0 + f1);
								pcl::PointXYZ point;
								point.getVector3fMap() = pt + origon_;
								cloud.points.push_back(point);

							}
							else{
								edge_to_index[i] = edgeindex_to_vertexindex.find(edge_index)->second;
							}
						}
					}

					for (int i = 0; pcl::triTable[cube_index][i] != -1; i+=3)
					{
						pcl::Vertices v;

						v.vertices.push_back(edge_to_index[pcl::triTable[cube_index][i]]);
						v.vertices.push_back(edge_to_index[pcl::triTable[cube_index][i+2]]);
						v.vertices.push_back(edge_to_index[pcl::triTable[cube_index][i + 1]]);
						mesh.polygons.push_back(v);
					}
				}
			}
		}

		pcl::toPCLPointCloud2(cloud, mesh.cloud);

	}

	void UniformTSDFVolume::integrateWithDepthToCameraDistanceMultiplier(const RgbdImage &image, const IntrinsicMatrix &intrinsic, const Eigen::Matrix4f &extrinsic, const cv::Mat& depth_to_camera_distance_multiplier)
	{
		float fx = intrinsic.fx();
		float fy = intrinsic.fy();
		float cx = intrinsic.cx();
		float cy = intrinsic.cy();
		float half_voxel_length = 0.5 * voxel_length_;
		float sdf_trunc_inverse = 1.0f / sdf_trunc_;

		Eigen::Matrix4f extrinsic_scaled = extrinsic * voxel_length_;

		float safe_width = image.width - 0.0001f;
		float safe_height = image.height - 0.0001f;
		for (int x = 0; x < resolution_; ++x)
		{
			for (int y = 0; y < resolution_; ++y)
			{
				int idx_shift = x * resolution_ * resolution_ + y * resolution_;
				float *p_tsdf = (float *)tsdf_.data() + idx_shift;
				float *p_weight = (float *)weight_.data() + idx_shift;
				float *p_color = (float *)color_.data() + idx_shift * 3;

				Eigen::Vector4f voxel_pt_camera = extrinsic * Eigen::Vector4f(half_voxel_length + x * voxel_length_ + origon_(0),
					half_voxel_length + y * voxel_length_ + origon_(1), half_voxel_length + origon_(2), 1.0f);
				for (int z = 0; z < resolution_; ++z,
					voxel_pt_camera(0) += extrinsic_scaled(0,2),
					voxel_pt_camera(1) += extrinsic_scaled(1,2),
					voxel_pt_camera(2) += extrinsic_scaled(2,2),
					p_tsdf++, p_weight++, p_color += 3)
				{
					if (voxel_pt_camera(2) > 0)
					{
						float x_f = voxel_pt_camera(0) * fx / voxel_pt_camera(2) + cx + 0.5f;
						float y_f = voxel_pt_camera(1) * fy / voxel_pt_camera(2) + cy + 0.5f;
						if (x_f >= 0.0001f && x_f < safe_width && y_f >= 0.0001f && y_f < safe_height)
						{
							int x = static_cast<int>(x_f);
							int y = static_cast<int>(y_f);
							float d = image.depth.at<float>(y, x);
							if (std::isfinite(d))
							{
								float sdf = (d - voxel_pt_camera(2)) * depth_to_camera_distance_multiplier.at<float>(y, x);
								if (sdf > -sdf_trunc_)
								{
									// integrate
									float tsdf = std::min(1.0f, sdf * sdf_trunc_inverse);
									*p_tsdf = ((*p_tsdf) * (*p_weight) + tsdf) / (*p_weight + 1.0f);
									if (with_color_)
									{
										float grey_color = image.intensity.at<float>(y, x);

										p_color[0] = (p_color[0] * (*p_weight) + grey_color) / (*p_weight + 1.0f);
										p_color[1] = (p_color[1] * (*p_weight) + grey_color) / (*p_weight + 1.0f);
										p_color[2] = (p_color[2] * (*p_weight) + grey_color) / (*p_weight + 1.0f);
									}

									*p_weight += 1.0f;
								}
							}
						}
					}
				}
			}
		}
	}

}
