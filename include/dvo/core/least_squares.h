
#ifndef  DVO_LEAST_SQUARES_H
#define  DVO_LEAST_SQUARES_H
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <dvo/core/data_types.h>
#include <dvo/core/math_sse.h>
#include <dvo/core/dvo_exports.h>
namespace dvo
{
	class DVO_EXPORTS LeastSquaresInterface
	{
	public:
		virtual ~LeastSquaresInterface(){};
		virtual void initialize(const size_t maxnum_constraints) = 0;
		virtual void update(const Vector6& J, const NumType& res, const NumType& weight = 1.0f) = 0;
		virtual void update(const Eigen::Matrix<NumType, 2, 6>& J, const Eigen::Matrix<NumType, 2, 1>& res, const Eigen::Matrix<NumType, 2, 2>& weight){};
		virtual void finish() = 0;
		virtual void solve(Vector6& x) = 0;
	};

	/**
	* Basic interface for algorithms solving 1 step of non-linear least squares where jacobians can be precomputed and don't vary between iterations.
	*/
	class PrecomputedLeastSquaresInterface
	{
	public:
		virtual ~PrecomputedLeastSquaresInterface() {};
		virtual void initialize(const size_t maxnum_constraints) = 0;
		// add a jacobian to the cache
		virtual void addConstraint(const size_t& idx, const Vector6& J) = 0;

		// resets internal state created for one iteration
		virtual void reset() = 0;

		virtual void next() = 0;

		virtual void ignoreConstraint(const size_t& idx) = 0;
		virtual bool setResidualForConstraint(const size_t& idx, const NumType& res, const NumType& weight = 1.0f) = 0;
		virtual void finish() = 0;
		virtual void solve(Vector6& x) = 0;
	};

	/**
	* Builds normal equations and solves them with Cholesky decomposition.
	*/
	class DVO_EXPORTS NormalEquationsLeastSquares : public LeastSquaresInterface
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		OptimizedSelfAdjointMatrix6x6f A_opt;
		Matrix6x6 A;
		Vector6 b;

		double error;
		size_t maxnum_constraints, num_constraints;

		virtual ~NormalEquationsLeastSquares();

		virtual void initialize(const size_t maxnum_constraints);
		virtual void update(const Vector6& J, const NumType& res, const NumType& weight = 1.0f);
		virtual void update(const Eigen::Matrix<NumType, 2, 6>& J, const Eigen::Matrix<NumType, 2, 1>& res, const Eigen::Matrix<NumType, 2, 2>& weight);
		virtual void finish();
		virtual void solve(Vector6& x);

		void combine(const NormalEquationsLeastSquares& other);
	};

	/**
	* Builds normal equations and solves them with Cholesky decomposition.
	*/
	class PrecomputedNormalEquationsLeastSquares : public PrecomputedLeastSquaresInterface
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Matrix6x6 A;
		Vector6 b;

		double error;

		size_t maxnum_constraints, num_constraints;

		virtual ~PrecomputedNormalEquationsLeastSquares();

		virtual void initialize(const size_t maxnum_constraints);
		virtual void addConstraint(const size_t& idx, const Vector6& J);

		virtual void reset();
		virtual void next();

		virtual void ignoreConstraint(const size_t& idx);

		virtual bool setResidualForConstraint(const size_t& idx, const NumType& res, const NumType& weight = 1.0f);
		virtual void finish();
		virtual void solve(Vector6& x);

		const uchar* mask_ptr_;
	private:

		Matrix6x6 hessian_;
		Matrix6x6 hessian_error_;
		Eigen::Matrix<NumType, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_cache_;
		cv::Mat1b mask_;
	};

	/**
	* Same as NormalEquationsLeastSquares, but solves normal equations with EigenValueDecomposition.
	*/
	class EvdLeastSquares : public NormalEquationsLeastSquares
	{
	public:
		virtual ~EvdLeastSquares();

		virtual void solve(Vector6& x);
	};

	class SvdLeastSquares : public LeastSquaresInterface
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Eigen::Matrix<NumType, Eigen::Dynamic, 6> J;
		Eigen::Matrix<NumType, Eigen::Dynamic, 1> residuals;

		virtual ~SvdLeastSquares();

		virtual void initialize(const size_t maxnum_constraints);
		virtual void update(const Vector6& J, const NumType& res, const NumType& weight = 1.0f);
		virtual void finish();
		virtual void solve(Vector6& x);
	private:
		int current;
	};
}
#endif