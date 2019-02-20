#include <dvo/registration/point_tracking.h>
#include <dvo/registration/icp.h>
namespace dvo
{
	
bool PointTracker::align(IPointCloud::Ptr &target_cloud, IPointCloud::Ptr &source_cloud, Matrix4& final_trans, const Matrix4& init_trans /*= Matrix4::Identity()*/)
{
	Result result;
	bool success = align(target_cloud, source_cloud, result, init_trans);
	final_trans = result.transformation;
	return success;
}

bool PointTracker::align(IPointCloud::Ptr &target_cloud, IPointCloud::Ptr &source_cloud, Result& result, const Matrix4& init_trans /*= Matrix4::Identity()*/)
{
	ICPWithNormals icp;
	icp.setPrecision(cfg_.precision);
	icp.setMaxIterations(cfg_.max_iterations);
	icp.setMaxCorrespondenceDistance(cfg_.corr_dist_threshold);
	icp.setMaxAngle(cfg_.max_angle);

	icp.setInputSource(source_cloud->cloud());
	icp.setInputTarget(target_cloud->cloud());
	icp.setTargetTree(target_cloud->tree());
	icp.setSourceIndices(source_cloud->indices());

	bool res = icp.align(init_trans);
	result.transformation = icp.getFinalTransformation();
	result.information = icp.getInformation();
	result.corrs = icp.getCorrespondences();
	return res;
}

}// namespace dvo