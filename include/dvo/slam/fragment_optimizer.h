/*!
 * \file fragment_optimizer.h
 *
 * \author mengzhili
 * \date 2019/01/22 
 * 
 */
#ifndef DVO_SLAM_FRAGMENT_OPTIMIZER_H
#define DVO_SLAM_FRAGMENT_OPTIMIZER_H
#include <dvo/registration/feature_tracking.h>
#include <dvo/registration/point_tracking.h>
#include <dvo/slam/config.h>
#include <dvo/slam/local_map.h>
#include <dvo/slam/fragment.h>
#include <dvo/slam/local_tracker.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_offset.h>

namespace dvo
{
class FragmentOptimizer
{
public:
	typedef pcl::PointXYZRGBNormal PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

	struct Config 
	{
		int group_keyframe_size;
		int group_sample_num;
		float voxel_length;
		float min_group_overlap;
		float search_radius;
		float max_corr_dist;
	};
	FragmentOptimizer();
	~FragmentOptimizer();

	void configure(const FragmentOptimizer::Config &cfg) { cfg_ = cfg; }
	void configurePointTracking(const PointTracker::Config &point_cfg) { point_tracker_.configure(point_cfg); }

	void initialize();

	void add(const LocalMap::Ptr &m, bool force_keyframe);

	void optimize();

	void getFinalPoses(std::vector<Eigen::Matrix4d> &final_poses);

	void saveFragmentPoints(const std::string &folder);
private:
	struct CorrConstraint 
	{
		int source_id;
		int target_id;
		pcl::CorrespondencesPtr corrs;
	};
	g2o::VertexSE3* addVertex(int id);

	void findPosssibleConstraints(const std::vector<FragmentPtr> &groups, const FragmentPtr &cur_group, std::vector<FragmentPtr> &candidates);
	void findCorrConstraints(std::vector<CorrConstraint> &ccs);

	void insertNewFragment(const FragmentPtr &cur_group);

	void addCorrConstraint(const CorrConstraint &cc);

private:
	int current_keyframe_size_;
	int fragment_id_;
	FragmentPtr current_group_;
	std::vector<FragmentPtr> groups_;
private:
	Config cfg_;
	PointTracker point_tracker_;
	g2o::SparseOptimizer optimizer_;

};
} // namespace dvo
#endif
