
#include <Eigen/Eigenvalues>
#include <dvo/registration/dense_tracking.h>
namespace dvo
{
	DenseConfig::DenseConfig():
		firstLevel(3),
		lastLevel(0),
		maxIterationsPerLevel(100),
		precision(5e-7),
		mu(0),
		num_sample(0)
	{

	}
	size_t DenseConfig::getNumLevels() const
	{
		return firstLevel + 1;
	}

	bool DenseConfig::useEstimateSmoothing() const
	{
		return mu > 1e-6;
	}

	bool DenseConfig::isSane() const
	{
		return firstLevel >= lastLevel;
	}

	DenseTracker::IterationContext::IterationContext(const DenseConfig& cfg) :
		cfg(cfg)
	{

	}
	bool DenseTracker::IterationContext::isFirstIteration() const
	{
		return isFirstLevel() && isFisrtIterationOnLevel();
	}
	bool DenseTracker::IterationContext::isFisrtIterationOnLevel() const
	{
		return iteration == 0;
	}

	bool DenseTracker::IterationContext::isFirstLevel() const
	{
		return cfg.firstLevel == level;
	}

	bool DenseTracker::IterationContext::isLastLevel() const
	{
		return cfg.lastLevel == level;
	}
	double DenseTracker::IterationContext::errorDiff() const
	{
		return lastError - error;
	}

	bool DenseTracker::IterationContext::iterationExceeded() const
	{
		return iteration >= cfg.maxIterationsPerLevel;
	}
	bool DenseTracker::Result::isNaN() const
	{
		return !std::isfinite(transformation.sum());
	}

	DenseTracker::Result::Result() 
	{
		double nan = std::numeric_limits<double>::quiet_NaN();
		transformation.setConstant(nan);
	}

	void DenseTracker::Result::setIdensity()
	{
		transformation.setIdentity();

	}
}