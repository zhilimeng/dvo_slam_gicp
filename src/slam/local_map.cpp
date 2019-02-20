

#include <dvo/slam/local_map.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_offset.h>

namespace dvo
{

namespace internal
{

Eigen::Isometry3d toIsometry(const Eigen::Matrix4d& pose)
{
	Eigen::Isometry3d p(pose.block<3,3>(0,0));
	p.translation() = pose.block<3,1>(0,3);

	return p;
}

static Eigen::Affine3d toAffine(const Eigen::Isometry3d& pose)
{
	Eigen::Affine3d p(pose.rotation());
	p.translation() = pose.translation();

	return p;
}

} /* namespace internal */
class LocalMap::Impl
{
public:
	typedef g2o::BlockSolver_6_3 BlockSolver;
	typedef g2o::LinearSolverDense<BlockSolver::PoseMatrixType> LinearSolver;
	FramePtr keyframe_, current_;
	g2o::VertexSE3 *keyframe_vertex_, *previous_vertex_, *current_vertex_;

	g2o::SparseOptimizer graph_;
	int  max_edge_id_;

	Impl(const FramePtr& keyframe, const Eigen::Matrix4d& keyframe_pose) :
		keyframe_(keyframe),
		keyframe_vertex_(0),
		previous_vertex_(0),
		current_vertex_(0),
		max_edge_id_(1)
	{
		// g2o setup
		graph_.setAlgorithm(
			new g2o::OptimizationAlgorithmLevenberg(
				new BlockSolver(
					new LinearSolver()
				)
			)
		);
		graph_.setVerbose(false);
		keyframe_->setKeyframe();
		keyframe_vertex_ = addFrameVertex(keyframe_->id());
		keyframe_vertex_->setFixed(true);
		keyframe_vertex_->setEstimate(internal::toIsometry(keyframe_pose));
	}

	g2o::VertexSE3* addFrameVertex(int id)
	{
		g2o::VertexSE3* frame_vertex = new g2o::VertexSE3();
		frame_vertex->setId(id);

		graph_.addVertex(frame_vertex);

		return frame_vertex;
	}

	g2o::EdgeSE3* addTransformationEdge(g2o::VertexSE3 *from, g2o::VertexSE3 *to, const Eigen::Matrix4d& transform, const Matrix6d& information)
	{
		assert(from != 0 && to != 0);

		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		edge->setId(max_edge_id_++);
		edge->resize(2);
		edge->setVertex(0, from);
		edge->setVertex(1, to);
		edge->setMeasurement(internal::toIsometry(transform));
		edge->setInformation(information);

		graph_.addEdge(edge);

		return edge;
	}
};

LocalMap::Ptr LocalMap::create(const FramePtr& keyframe, const Matrix4& keyframe_pose)
{
	LocalMap::Ptr result(new LocalMap(keyframe, keyframe_pose));
	return result;
}

LocalMap::LocalMap(const FramePtr& keyframe, const Matrix4& keyframe_pose) :
	impl_(new Impl(keyframe, keyframe_pose))
{
	frames_.push_back(keyframe);
}

LocalMap::~LocalMap()
{
}

FramePtr LocalMap::getKeyframe()
{
	return impl_->keyframe_;
}

FramePtr LocalMap::getCurrentFrame()
{
	return impl_->current_;
}

void LocalMap::getCurrentFramePose(Matrix4& current_pose)
{
	current_pose = impl_->current_vertex_->estimate().matrix();
}

void LocalMap::setKeyframePose(const Matrix4& keyframe_pose)
{
	impl_->keyframe_vertex_->setEstimate(internal::toIsometry(keyframe_pose));

	g2o::OptimizableGraph::EdgeSet& edges = impl_->keyframe_vertex_->edges();

	for (g2o::OptimizableGraph::EdgeSet::iterator it = edges.begin(); it != edges.end(); ++it)
	{
		g2o::EdgeSE3 *e = (g2o::EdgeSE3*)(*it);

		assert(e->vertex(0) == impl_->keyframe_vertex_);

		g2o::VertexSE3 *v = (g2o::VertexSE3*)e->vertex(1);
		v->setEstimate(impl_->keyframe_vertex_->estimate() * e->measurement());
	}
}


void LocalMap::addFrame(const FramePtr& frame)
{
	impl_->current_ = frame;
	impl_->previous_vertex_ = impl_->current_vertex_;
	impl_->current_vertex_ = impl_->addFrameVertex(frame->id());
	frames_.push_back(frame);
}

void LocalMap::addOdometryMeasurement(const Matrix4 &pose, const Matrix6 &information)
{
	impl_->addTransformationEdge(impl_->previous_vertex_, impl_->current_vertex_, pose, information);
}

void LocalMap::addKeyframeMeasurement(const Matrix4 &pose, const Matrix6 &information)
{
	impl_->addTransformationEdge(impl_->keyframe_vertex_, impl_->current_vertex_, pose, information);
	impl_->current_vertex_->setEstimate(impl_->keyframe_vertex_->estimate() * internal::toIsometry(pose));
}

void LocalMap::optimize()
{
	impl_->graph_.initializeOptimization();
	impl_->graph_.computeInitialGuess();
	impl_->graph_.optimize(50);

	// update frame pose

	for (size_t i = 0; i < frames_.size(); ++i)
	{
		g2o::VertexSE3 *v = static_cast<g2o::VertexSE3*>(impl_->graph_.vertex(frames_[i]->id()));

		frames_[i]->pose(v->estimate().matrix());
	}
}

} /* namespace dvo_slam */
