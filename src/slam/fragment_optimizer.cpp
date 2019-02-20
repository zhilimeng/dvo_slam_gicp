#include <dvo/slam/fragment_optimizer.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/icp/types_icp.h>
namespace dvo
{
namespace internal
{
g2o::SE3Quat toSE(const Eigen::Matrix4d &t)
{
	return g2o::SE3Quat(t.block<3, 3>(0, 0), t.block<3, 1>(0, 3));
}
} // namespace internal

FragmentOptimizer::FragmentOptimizer() :
	current_keyframe_size_(0), fragment_id_(0),
	current_group_(new Fragment)
{

}
FragmentOptimizer::~FragmentOptimizer()
{

}

void FragmentOptimizer::initialize()
{
	current_group_->id(fragment_id_++);
	optimizer_.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
		new g2o::BlockSolverX(
			new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>)));
	optimizer_.setVerbose(true);
}

void FragmentOptimizer::add(const LocalMap::Ptr &m, bool force_keyframe)
{
	m->optimize();
	if (current_keyframe_size_ >= cfg_.group_keyframe_size)
	{
		std::cout << "\tcurrent group: " << fragment_id_ << std::endl;
		insertNewFragment(current_group_);
		current_group_.reset(new Fragment);
		current_group_->id(fragment_id_++);
		current_group_->addLocalMap(m);
		current_keyframe_size_ = 0;
	}
	else {
		current_group_->addLocalMap(m);
		current_keyframe_size_++;
	}
	if (force_keyframe)
	{
		insertNewFragment(current_group_);
	}
}

void FragmentOptimizer::optimize()
{
	for (const auto &group:groups_)
	{
		group->buildPointCloud(cfg_.voxel_length);
	}
	std::vector<CorrConstraint> ccs;
	findCorrConstraints(ccs);

	// create optimizer
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
	g2o::BlockSolverX::LinearSolverType *linear_solver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linear_solver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// set up vertex pose
	std::cout << "set up vertex...\n";
	for (size_t i = 0; i < groups_.size(); ++i)
	{
		int id = groups_[i]->id();
		g2o::VertexSE3* v = new g2o::VertexSE3();
		v->setId(id);
		v->setEstimate(internal::toSE(groups_[i]->getPose()));
		if (id == 0)
		{
			v->setFixed(true);
		}
		optimizer.addVertex(v);
	}

	// set up constraints
	std::cout << "set up constraints...\n";
	for (const auto &cc: ccs)
	{
		int tgt_id = cc.target_id;
		int src_id = cc.source_id;

		g2o::VertexSE3* vp0 = dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(tgt_id)->second);
		g2o::VertexSE3* vp1 = dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(src_id)->second);

		PointCloudPtr tgt_cloud = groups_[tgt_id]->getPointCloud();
		PointCloudPtr src_cloud = groups_[src_id]->getPointCloud();
		for (size_t i = 0; i < cc.corrs->size(); ++i)
		{
			const pcl::Correspondence &corr = cc.corrs->at(i);
			const PointT &tgt_pt = tgt_cloud->points[corr.index_match];
			const PointT &src_pt = src_cloud->points[corr.index_query];

			Eigen::Vector3d pt0, pt1, nm0, nm1;
			pt0 = tgt_pt.getVector3fMap().cast<double>();
			pt1 = src_pt.getVector3fMap().cast<double>();

			nm0 = tgt_pt.getNormalVector3fMap().cast<double>();
			nm1 = src_pt.getNormalVector3fMap().cast<double>();

			g2o::Edge_V_V_GICP *e = new g2o::Edge_V_V_GICP();
			e->setVertex(0, vp0);
			e->setVertex(1, vp1);
			g2o::EdgeGICP meas;
			meas.pos0 = pt0;
			meas.pos1 = pt1;
			meas.normal0 = nm0;
			meas.normal1 = nm1;

			e->setMeasurement(meas);
			e->information() = meas.prec0(0.01);
			optimizer.addEdge(e);
		}

	}
	std::cout << "optimize...\n";
	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	optimizer.setVerbose(true);
	optimizer.optimize(10);

	// update poses
	for (size_t i = 0; i < groups_.size(); ++i)
	{
		g2o::VertexSE3 *v = static_cast<g2o::VertexSE3*>(optimizer.vertex(groups_[i]->id()));
		groups_[i]->updatePose(v->estimate().matrix());
	}
}

void FragmentOptimizer::getFinalPoses(std::vector<Eigen::Matrix4d> &final_poses)
{
	for (const auto &group: groups_)
	{
		for (size_t i =0; i < group->size(); ++i)
		{
			final_poses.push_back(group->getFrame(i)->pose());
		}
	}
}

void FragmentOptimizer::saveFragmentPoints(const std::string &folder)
{
	for (const auto &group : groups_)
	{
		group->saveFragmentPoints(folder);
	}
}

g2o::VertexSE3* FragmentOptimizer::addVertex(int id)
{
	g2o::VertexSE3 *v = new g2o::VertexSE3();
	v->setId(id);
	if (id == 0)
	{
		v->setFixed(true);
	}
	optimizer_.addVertex(v);
	return v;
}

void FragmentOptimizer::findPosssibleConstraints(const std::vector<FragmentPtr> &groups, const FragmentPtr &cur_group, std::vector<FragmentPtr> &candidates)
{
	pcl::PointXYZ search_point;
	search_point.x = cur_group->getPose()(0, 3);
	search_point.y = cur_group->getPose()(1, 3);
	search_point.z = cur_group->getPose()(2, 3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr group_points(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i  < groups.size() -1; ++i)
	{
		pcl::PointXYZ p;
		p.x = groups[i]->getPose()(0, 3);
		p.y = groups[i]->getPose()(1, 3);
		p.z = groups[i]->getPose()(2, 3);

		group_points->points.push_back(p);
	}
	std::vector<int> nn_indices;
	std::vector<float> nn_dists;
	pcl::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(group_points);
	kdtree->radiusSearch(search_point, cfg_.search_radius, nn_indices, nn_dists);
	for (const auto &idx : nn_indices)
	{
		candidates.push_back(groups[idx]);
	}
}

void FragmentOptimizer::findCorrConstraints(std::vector<CorrConstraint> &ccs)
{
	std::vector<FragmentPtr> constraint_candidates;
	
	for (const auto &cur_group : groups_)
	{
		std::cout << "current group: " << cur_group->id() << std::endl;
		if(cur_group->id() == 0)
			continue;
		pcl::PointXYZ search_point;
		search_point.x = cur_group->getPose()(0, 3);
		search_point.y = cur_group->getPose()(1, 3);
		search_point.z = cur_group->getPose()(2, 3);

		pcl::PointCloud<pcl::PointXYZ>::Ptr group_points(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t i = 0; i < cur_group->id(); ++i)
		{
			pcl::PointXYZ p;
			p.x = groups_[i]->getPose()(0, 3);
			p.y = groups_[i]->getPose()(1, 3);
			p.z = groups_[i]->getPose()(2, 3);

			group_points->points.push_back(p);
		}
		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		pcl::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
		kdtree->setInputCloud(group_points);
		kdtree->radiusSearch(search_point, cfg_.search_radius, nn_indices, nn_dists);
		std::vector<FragmentPtr> candidates;
		std::cout << "\t candidate loop: " << std::endl;
		for (const auto &idx : nn_indices)
		{
			std::cout << idx << " ";
			candidates.push_back(groups_[idx]);
		}
		std::cout << std::endl;
		for (const auto &ref_group : candidates)
		{
			ref_group->buildPointCloud(cfg_.voxel_length);

			Eigen::Matrix4d init_trans = ref_group->getPose().inverse() * cur_group->getPose();

			IPointCloudPtr cloud_tgt = IPointCloud::create(ref_group->getPointCloud());
			IPointCloudPtr cloud_src = IPointCloud::create(cur_group->getPointCloud());
			cloud_src->samplePointCloud(cfg_.group_sample_num);
			float overlap = cloud_tgt->computeOverlap(cloud_src, init_trans, cfg_.max_corr_dist);
			if (overlap < cfg_.min_group_overlap)
			{
				std::cout << "\t overlap too little\n";
				continue;
			}
			PointTracker::Result icp_result;
			if (point_tracker_.align(cloud_tgt, cloud_src, icp_result, init_trans))
			{
				std::cout << "\t add " << icp_result.corrs->size() << " constraints: " << cur_group->id() << " -> " << ref_group->id() << std::endl;
				CorrConstraint cc;
				cc.source_id = cur_group->id();
				cc.target_id = ref_group->id();
				cc.corrs = icp_result.corrs;
				ccs.push_back(cc);
			}
			else if(cur_group->id() - ref_group->id() == 1)
			{
				CorrConstraint cc;
				cc.source_id = cur_group->id();
				cc.target_id = ref_group->id();
				cc.corrs = icp_result.corrs;
				ccs.push_back(cc);
				std::cout << "\t add " << icp_result.corrs->size() << " constraints: " << cur_group->id() << " -> " << ref_group->id() << std::endl;
			}

		}
	}
}

void FragmentOptimizer::insertNewFragment(const FragmentPtr &cur_group)
{
	cur_group->buildPointCloud(cfg_.voxel_length);
	groups_.push_back(cur_group);
	g2o::VertexSE3 *v = addVertex(cur_group->id());
	v->setEstimate(internal::toSE(cur_group->getPose()));
	if (cur_group->id() == 0)
		return;

	std::vector<FragmentPtr> constraint_candidates;
	findPosssibleConstraints(groups_, cur_group, constraint_candidates);

	int max_distance = -1;
	for (const auto &ref_group: constraint_candidates)
	{
		std::cout << "\t candidate: " << ref_group->id() << std::endl;
		ref_group->buildPointCloud(cfg_.voxel_length);

		Eigen::Matrix4d init_trans = ref_group->getPose().inverse() * cur_group->getPose();

		IPointCloudPtr cloud_tgt = IPointCloud::create(ref_group->getPointCloud());
		IPointCloudPtr cloud_src = IPointCloud::create(cur_group->getPointCloud());
		cloud_src->samplePointCloud(cfg_.group_sample_num);
		float overlap = cloud_tgt->computeOverlap(cloud_src, init_trans, cfg_.max_corr_dist);
		if (overlap < cfg_.min_group_overlap)
		{
			std::cout << "\t overlap too little\n";
			continue;
		}
		PointTracker::Result icp_result;
		if (point_tracker_.align(cloud_tgt,cloud_src,icp_result,init_trans))
		{
			std::cout << "\t add " << icp_result.corrs->size() << " constraints: " << cur_group->id() << " -> " << ref_group->id() << std::endl;

			max_distance = std::max(max_distance, std::abs(cur_group->id() - ref_group->id()));
			CorrConstraint cc;
			cc.source_id = cur_group->id();
			cc.target_id = ref_group->id();
			cc.corrs = icp_result.corrs;
			addCorrConstraint(cc);
		}
		else
		{
			std::cout << "\t icp fail\n";
		}

	}

	if (max_distance >= 3)
	{
		optimizer_.initializeOptimization();
		optimizer_.computeActiveErrors();
		optimizer_.optimize(10);
		// update group pose
		for (size_t i =0; i < optimizer_.vertices().size(); ++i)
		{
			g2o::VertexSE3* v = static_cast<g2o::VertexSE3*>(optimizer_.vertex(groups_[i]->id()));
			groups_[i]->updatePose(v->estimate().matrix());
		}
	}
		
}

void FragmentOptimizer::addCorrConstraint(const CorrConstraint &cc)
{
	int tgt_id = cc.target_id;
	int src_id = cc.source_id;

	g2o::VertexSE3* vp0 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertices().find(tgt_id)->second);
	g2o::VertexSE3* vp1 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertices().find(src_id)->second);

	PointCloudPtr tgt_cloud = groups_[tgt_id]->getPointCloud();
	PointCloudPtr src_cloud = groups_[src_id]->getPointCloud();
	for (size_t i = 0; i < cc.corrs->size(); ++i)
	{
		const pcl::Correspondence &corr = cc.corrs->at(i);
		const PointT &tgt_pt = tgt_cloud->points[corr.index_match];
		const PointT &src_pt = src_cloud->points[corr.index_query];

		Eigen::Vector3d pt0, pt1, nm0, nm1;
		pt0 = tgt_pt.getVector3fMap().cast<double>();
		pt1 = src_pt.getVector3fMap().cast<double>();

		nm0 = tgt_pt.getNormalVector3fMap().cast<double>();
		nm1 = src_pt.getNormalVector3fMap().cast<double>();

		g2o::Edge_V_V_GICP *e = new g2o::Edge_V_V_GICP();
		e->setVertex(0, vp0);
		e->setVertex(1, vp1);
		g2o::EdgeGICP meas;
		meas.pos0 = pt0;
		meas.pos1 = pt1;
		meas.normal0 = nm0;
		meas.normal1 = nm1;

		e->setMeasurement(meas);
		e->information() = meas.prec0(0.01);
		optimizer_.addEdge(e);
	}
}

} // namespace dvo