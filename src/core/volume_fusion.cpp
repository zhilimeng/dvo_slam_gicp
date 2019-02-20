#include <dvo/core/volume_fusion.h>
#include <unordered_set>
namespace dvo
{
VolumeFusion::VolumeFusion():voxel_length_(0.005)
{
}

VolumeFusion::~VolumeFusion()
{

}

void VolumeFusion::integrate(const PointCloud &cloud, const Eigen::Matrix4d& pose)
{
	Eigen::Matrix4f pose_f = pose.cast<float>();
	for (const auto& point : cloud.points)
	{
		PointT pt_w = point;
		pt_w.getVector4fMap() = pose_f * point.getVector4fMap();
		pt_w.getNormalVector4fMap() = pose_f * point.getNormalVector4fMap();
		Eigen::Vector3i idx = locateVolumeUnit(pt_w.getVector3fMap());
			
		auto iter = volume_units_.find(idx);
		if (iter == volume_units_.end())
		{
			volume_units_[idx] = pt_w;
		}
		else
		{
			PointT pt_new;
			pt_new.getVector4fMap() = (pt_w.getVector4fMap() + iter->second.getVector4fMap())*0.5;
			pt_new.getNormalVector4fMap() = (pt_w.getNormalVector4fMap() + iter->second.getNormalVector4fMap())*0.5;
			pt_new.getRGBVector3i() = (pt_w.getRGBVector3i() + iter->second.getRGBVector3i()) / 2;
			volume_units_[idx] = pt_new;
		}
	}
}


void VolumeFusion::extractPointCloud(pcl::PointCloud<PointT>& cloud)
{
	for (const auto &unit :volume_units_)
	{
		cloud.points.push_back(unit.second);
	}

}

}