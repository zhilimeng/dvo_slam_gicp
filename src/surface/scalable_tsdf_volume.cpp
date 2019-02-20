#include <unordered_set>
#include <pcl/common/transforms.h>
#include <pcl/surface/marching_cubes.h>
#include <dvo/surface/helper.h>
#include <dvo/surface/uniform_tsdf_volume.h>
#include <dvo/surface/scalable_tsdf_volume.h>

namespace dvo
{
	ScalableTSDFVolume::ScalableTSDFVolume(float voxel_length, float sdf_trunc, bool with_color, int volume_unit_resolution /* = 16 */, int depth_sampling_stride /* = 4 */)
		:voxel_length_(voxel_length), sdf_trunc_(sdf_trunc), with_color_(with_color), volume_unit_resolution_(volume_unit_resolution),
		volume_unit_length_(voxel_length*volume_unit_resolution), depth_sampling_stride_(depth_sampling_stride), tsdf_threshold_(0.98f)
	{

	}
	ScalableTSDFVolume::~ScalableTSDFVolume()
	{

	}
	void ScalableTSDFVolume::reset()
	{
		volume_units_.clear();
	}

	void ScalableTSDFVolume::integrate(RgbdImage& image, const IntrinsicMatrix& intrinsic, const Eigen::Matrix4f& extrinsic)
	{
		cv::Mat depth2CameraDistance = createDepthToCameraDistanceMultiplierImage(intrinsic, image.width, image.height);

		//image.buildPointCloud();
		//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		//Eigen::Matrix4f trans = extrinsic.inverse();
		//pcl::transformPointCloud(*image.pointcloud, *pointcloud, trans);
		//image.buildPointCloud();
		//PointCloudT::Ptr pointcloud = image.pointcloud;

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointcloud = createPointCloudFromDepthImage(image.depth, intrinsic, extrinsic, 4);
		std::unordered_set<Eigen::Vector3i, hash_eigen::hash<Eigen::Vector3i> > touched_volume_units_;

		Eigen::Matrix4f pose = extrinsic.inverse();
		for (const auto &point : pointcloud->points)
		{
			//pcl::PointXYZRGBNormal pt_w;
			//pt_w.getVector4fMap() = pose * point.getVector4fMap();
			//pt_w.getNormalVector4fMap() = pose * point.getNormalVector4fMap();
//			pt_w.getIntensityAndDepthWithDerivativesVec8f() = point.getIntensityAndDepthWithDerivativesVec8f();
			Eigen::Vector3i min_bound = locateVolumeUnit(point.getVector3fMap() - Eigen::Vector3f(sdf_trunc_, sdf_trunc_, sdf_trunc_));
			Eigen::Vector3i max_bound = locateVolumeUnit(point.getVector3fMap() + Eigen::Vector3f(sdf_trunc_, sdf_trunc_, sdf_trunc_));
			for (int x = min_bound(0); x <= max_bound(0); ++x)
			{
				for (int y = min_bound(1); y <= max_bound(1); ++y)
				{
					for (int z = min_bound(2); z <= max_bound(2); ++z)
					{
						Eigen::Vector3i loc(x, y, z);
						if (touched_volume_units_.find(loc) == touched_volume_units_.end())
						{
							touched_volume_units_.insert(loc);

							UniformTSDFVolumePtr volume = openVolumeUnit(loc);
							volume->integrateWithDepthToCameraDistanceMultiplier(image, intrinsic, extrinsic, depth2CameraDistance);

						}

					}
				}
			}
		}
	}

	void ScalableTSDFVolume::integrate(const PointCloudT& cloud, const Eigen::Matrix4f& extrinsic)
	{
		std::unordered_set<Eigen::Vector3i, hash_eigen::hash<Eigen::Vector3i> > touched_volume_units_;

		Eigen::Matrix4f pose = extrinsic.inverse();
		for (const auto &point : cloud.points)
		{
			PointT pt_w;
			pt_w.getVector4fMap() = pose * point.getVector4fMap();
			pt_w.getNormalVector4fMap() = pose * point.getNormalVector4fMap();
			Eigen::Vector3i min_bound = locateVolumeUnit(pt_w.getVector3fMap() - Eigen::Vector3f(sdf_trunc_, sdf_trunc_, sdf_trunc_));
			Eigen::Vector3i max_bound = locateVolumeUnit(pt_w.getVector3fMap() + Eigen::Vector3f(sdf_trunc_, sdf_trunc_, sdf_trunc_));
			for (int x = min_bound(0); x <= max_bound(0); ++x)
			{
				for (int y = min_bound(1); y <= max_bound(1); ++y)
				{
					for (int z = min_bound(2); z <= max_bound(2); ++z)
					{
						Eigen::Vector3i loc(x, y, z);
						if (touched_volume_units_.find(loc) == touched_volume_units_.end())
						{
							touched_volume_units_.insert(loc);

							UniformTSDFVolumePtr volume = openVolumeUnit(loc);
							//							volume->integrateWithDepthToCameraDistanceMultiplier(image, intrinsic, extrinsic, depth2CameraDistance);
							volume->integrate(point, extrinsic);

						}

					}
				}
			}
		}
	}

	void ScalableTSDFVolume::extractPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud)
	{
		float half_voxel_length = 0.5 * voxel_length_;
		float w0, w1, f0, f1;
		Eigen::Vector3f c0, c1;
		for (const auto& unit : volume_units_)
		{
			if (unit.second.volume_)
			{
				const UniformTSDFVolume& volume0 = *unit.second.volume_;
				const Eigen::Vector3i& index0 = unit.second.index_;
				for (int x = 0; x < volume0.resolution_; ++x)
				{
					for (int y = 0; y < volume0.resolution_; ++y)
					{
						for (int z = 0; z < volume0.resolution_; ++z)
						{
							Eigen::Vector3i idx0(x, y, z);
							w0 = volume0.weight_[volume0.indexOf(idx0)];
							f0 = volume0.tsdf_[volume0.indexOf(idx0)];
							if (with_color_)
								c0 = volume0.color_[volume0.indexOf(idx0)];
							if (w0 != 0.0f && f0 < tsdf_threshold_ && f0 >= -tsdf_threshold_)
							{
								Eigen::Vector3f p0 = Eigen::Vector3f(half_voxel_length + voxel_length_ *x, half_voxel_length + voxel_length_ * y, half_voxel_length + voxel_length_*z)
									+ index0.cast<float>() * volume_unit_length_;
								for (int i = 0; i < 3; ++i)
								{
									Eigen::Vector3f p1 = p0;
									Eigen::Vector3i idx1 = idx0;
									Eigen::Vector3i index1 = index0;
									p1(i) += voxel_length_;
									idx1(i) += 1;
									if (idx1(i) < volume0.resolution_)
									{
										w1 = volume0.weight_[volume0.indexOf(idx1)];
										f1 = volume0.tsdf_[volume0.indexOf(idx1)];
										if (with_color_)
											c1 = volume0.color_[volume0.indexOf(idx1)];
									}
									else{
										idx1(i) -= volume0.resolution_;
										index1(i) += 1;

										auto unit_itr = volume_units_.find(index1);
										if (unit_itr == volume_units_.end())
										{
											w1 = 0.0f; f1 = 0.0f;
										}
										else{
											const UniformTSDFVolume& volume1 = *(unit_itr->second.volume_);
											w1 = volume1.weight_[volume1.indexOf(idx1)];
											f1 = volume1.tsdf_[volume1.indexOf(idx1)];
											if (with_color_)
												c1 = volume1.color_[volume1.indexOf(idx1)];
										}
									}

									if (w1 != 0.0f && f1 < tsdf_threshold_ && f1 >= -tsdf_threshold_ && f0 * f1 < 0)
									{
										float r0 = std::fabs(f0);
										float r1 = std::fabs(f1);
										Eigen::Vector3f p = p0;
										p(i) = (p0(i)*r1 + p1(i)*r0) / (r0 + r1);

										pcl::PointXYZRGBNormal pt;
										pt.getVector3fMap() = p;
										if (with_color_)
										{
											pt.getRGBVector3i() = ((c0 * r1 + c1 * r0) / (r0 + r1)).cast<int>();
										}
										pt.getNormalVector3fMap() = getNormalAt(p);
										cloud.push_back(pt);

									}
								}
							}
						} // end of z loop
					} // end of y loop
				} // end of x loop

			}
		} // end of volume unit loop
	}

	void ScalableTSDFVolume::extractVoxelPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud)
	{
		for (auto &unit : volume_units_)
		{
			if (unit.second.volume_)
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal> unit_cloud;
				unit.second.volume_->extractVoxelPointCloud(unit_cloud);
				cloud += unit_cloud;
			}
		}

	}

	void ScalableTSDFVolume::extractPolygonMesh(pcl::PolygonMesh &mesh)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;

		float half_voxel_length = 0.5 * voxel_length_;

		std::unordered_map<Eigen::Vector4i, int, hash_eigen::hash<Eigen::Vector4i> > edgeindex_to_vertexindex;
		int edge_to_index[12];
		for (const auto& unit: volume_units_)
		{
			if (unit.second.volume_)
			{
				const auto& volume0 = *unit.second.volume_;
				const auto& index0 = unit.second.index_;
				for (int x = 0; x < volume0.resolution_; ++x)
				{
					for (int y = 0; y < volume0.resolution_; ++y)
					{
						for (int z = 0; z < volume0.resolution_; ++z)
						{
							Eigen::Vector3i idx0(x, y, z);
							int cube_index = 0;
							float w[8];
							float f[8];
							Eigen::Vector3f c[8];
							for (int i = 0; i < 8; ++i)
							{
								Eigen::Vector3i index1 = index0;
								Eigen::Vector3i idx1 = idx0 + shift[i];
								if (idx1(0) < volume_unit_resolution_ && idx1(1) < volume_unit_resolution_ && idx1(2) < volume_unit_resolution_)
								{
									w[i] = volume0.weight_[volume0.indexOf(idx1)];
									f[i] = volume0.tsdf_[volume0.indexOf(idx1)];
								}
								else{
									for (int j = 0; j < 3; ++j)
									{
										if (idx1(j) >= volume_unit_resolution_)
										{
											idx1(j) -= volume_unit_resolution_;
											index1(j) += 1;
										}
									}
									auto unit_iter1 = volume_units_.find(index1);
									if (unit_iter1 == volume_units_.end())
									{
										w[i] = 0.0f;
										f[i] = 0.0f;
									}
									else
									{
										const auto &volume1 = *unit_iter1->second.volume_;
										w[i] = volume1.weight_[volume1.indexOf(idx1)];
										f[i] = volume1.tsdf_[volume1.indexOf(idx1)];
									}
								}
								if (w[i] == 0.0f)
								{
									cube_index = 0;
									break;
								}
								else
								{
									if (f[i] < 0.0f)
										cube_index |= (1 << i);
								}
							}

							if (cube_index == 0 || cube_index == 255)
								continue;
							for (int i = 0; i < 12; ++i)
							{
								if (pcl::edgeTable[cube_index] & (1 << i))
								{
									Eigen::Vector4i edge_index = Eigen::Vector4i(index0(0), index0(1), index0(2), 0) * volume_unit_resolution_
										+ Eigen::Vector4i(x, y, z, 0) + edge_shift[i];
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
										point.getVector3fMap() = pt ;
										cloud.points.push_back(point);
									}
									else
									{
										edge_to_index[i] = edgeindex_to_vertexindex[edge_index];
									}
								}
							}
							for (int i = 0; pcl::triTable[cube_index][i] != -1; i+=3)
							{
								pcl::Vertices v;

								v.vertices.push_back(edge_to_index[pcl::triTable[cube_index][i]]);
								v.vertices.push_back(edge_to_index[pcl::triTable[cube_index][i + 2]]);
								v.vertices.push_back(edge_to_index[pcl::triTable[cube_index][i + 1]]);
								mesh.polygons.push_back(v);
							}
						}// end of z loop
					}// end of y loop
				}// end of x loop
			}
		}

		pcl::toPCLPointCloud2(cloud, mesh.cloud);
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ScalableTSDFVolume::createPointCloudFromDepthImage(const cv::Mat &depth, const IntrinsicMatrix& K, const Eigen::Matrix4f& extrinsic, int step)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		Eigen::Matrix4f camera_pose = extrinsic.inverse();
		float fx = K.fx();
		float fy = K.fy();
		float cx = K.cx();
		float cy = K.cy();
		for (int y = 0; y < depth.rows; y += step)
		{
			for (int x = 0; x < depth.cols; x += step)
			{
				const float* p = depth.ptr<const float>(y, x);
				if (std::isfinite(*p))
				{
					pcl::PointXYZRGBNormal point;
					float z = *p;
					float xx = (x - cx)*z / fx;
					float yy = (y - cy)*z / fy;

					Eigen::Vector4f pt = camera_pose * Eigen::Vector4f(xx, yy, z, 1.0);

					point.getVector4fMap() = pt;
					cloud->push_back(point);
				}
			}
		}
		return cloud;
	}

	boost::shared_ptr<UniformTSDFVolume> ScalableTSDFVolume::openVolumeUnit(const Eigen::Vector3i& index)
	{
		VolumeUnit& unit = volume_units_[index];
		if (!unit.volume_)
		{
			unit.volume_.reset(new UniformTSDFVolume(volume_unit_length_, volume_unit_resolution_, sdf_trunc_, with_color_, index.cast<float>()*volume_unit_length_));
			unit.index_ = index;
		}
		return unit.volume_;
	}

	Eigen::Vector3f ScalableTSDFVolume::getNormalAt(const Eigen::Vector3f &p)
	{
		Eigen::Vector3f n;
		const float half_gap = 0.99 * voxel_length_;
		for (int i = 0; i < 3; ++i)
		{
			Eigen::Vector3f p0 = p;
			p0(i) -= half_gap;
			Eigen::Vector3f p1 = p;
			p1(i) += half_gap;
			n(i) = getTSDFAt(p1) - getTSDFAt(p0);
		}
		return n.normalized();
	}

	float ScalableTSDFVolume::getTSDFAt(const Eigen::Vector3f &p)
	{
		Eigen::Vector3f p_locate = p - Eigen::Vector3f(0.5, 0.5, 0.5) * voxel_length_;

		Eigen::Vector3i index0 = locateVolumeUnit(p_locate);
		auto unit_itr = volume_units_.find(index0);
		if (unit_itr == volume_units_.end())
			return 0.0;

		const UniformTSDFVolume& volume0 = *(unit_itr->second.volume_);
		Eigen::Vector3i idx0;
		Eigen::Vector3f p_grid = (p_locate - index0.cast<float>() * volume_unit_length_) / voxel_length_;
		for (int i = 0; i < 3; ++i)
		{
			idx0(i) = static_cast<int>(std::floor(p_grid(i)));
			if (idx0(i) < 0)
				idx0(i) = 0;
			if (idx0(i) >= volume_unit_resolution_)
				idx0(i) = volume_unit_resolution_ - 1;
		}

		Eigen::Vector3f r = p_grid - idx0.cast<float>();
		float f[8];
		for (int i = 0; i < 8; ++i)
		{
			Eigen::Vector3i index1 = index0;
			Eigen::Vector3i idx1 = idx0 + shift[i];
			if (idx1(0) < volume_unit_resolution_ && idx1(1) < volume_unit_resolution_ && idx1(2) < volume_unit_resolution_)
			{
				f[i] = volume0.tsdf_[volume0.indexOf(idx1)];
			}
			else{
				for (int j = 0; j < 3; ++j)
				{
					if (idx1(j) >= volume_unit_resolution_)
					{
						idx1(j) -= volume_unit_resolution_;
						index1(j) += 1;
					}
				}
				auto unit_itr1 = volume_units_.find(index1);
				if (unit_itr1 == volume_units_.end())
				{
					f[i] = 0.0f;
				}
				else
				{
					const UniformTSDFVolume& volume1 = *(unit_itr1->second.volume_);
					f[i] = volume1.tsdf_[volume1.indexOf(idx1)];
				}
			}
		}

		return (1 - r(0)) * ((1 - r(1)) * ((1 - r(2)) * f[0] + r(2) * f[4]) +
			r(1) * ((1 - r(2)) * f[3] + r(2) * f[7])) +
			r(0) * ((1 - r(1)) * ((1 - r(2)) * f[1] + r(2) * f[5]) +
			r(1) * ((1 - r(2)) * f[2] + r(2) * f[6]));
	}

}
