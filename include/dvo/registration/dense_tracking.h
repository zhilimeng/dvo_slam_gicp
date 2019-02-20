/*!
 * \file dense_tracking.h
 * \brief rgbd dense tracking from dvo slam
 * 
 * \author mengzhili
 * \date 2018/11/20 15:01
 *
 * 
 */
#ifndef  DVO_DENSE_TRACKING_H
#define  DVO_DENSE_TRACKING_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <dvo/core/rgbd_image.h>
#include <dvo/core/least_squares.h>

namespace dvo
{
	struct DVO_EXPORTS DenseConfig
	{
		int firstLevel;
		int lastLevel;
		int maxIterationsPerLevel;
		double precision;
		double mu;
		int num_sample;

		DenseConfig();
		size_t getNumLevels() const;
		bool useEstimateSmoothing() const;
		bool isSane() const;
	};

	class DVO_EXPORTS DenseTracker
	{
	public:
		typedef boost::shared_ptr<DenseTracker> Ptr;
		typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > ResidualVectorType;
		typedef std::vector<float> WeightVectorType;
		typedef Eigen::Matrix4d Matrix4;
	public:
		struct TerminationCriteria
		{
			enum Enum
			{
				IterationsExceeded,
				IncrementTooSmall,
				LogLikelihoodDecreased,
				TooFewConstraints,
				NumCriteria
			};
		};


		struct DVO_EXPORTS Result
		{
			Matrix4 transformation;
			Matrix6d information;
			TerminationCriteria::Enum terminationCriterion;
			Result();
			bool isNaN() const;
			void setIdensity();
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		};

		DenseTracker(const DenseConfig& cfg = DenseConfig());

		void configure(const DenseConfig& cfg);
		DenseConfig& configuration()
		{
			return cfg_;
		}

		bool match(RgbdImagePyramid &reference, RgbdImagePyramid &current, DenseTracker::Matrix4& transformation, const Matrix4& init_trans = Matrix4::Identity());
		bool match(RgbdImagePyramid &reference, RgbdImagePyramid &current, DenseTracker::Result& result, const Matrix4& init_trans = Matrix4::Identity());


	public:
		static inline void computeJacobianOfProjectionAndTransformation(const Vector4& p, Matrix2x6& j);

		static inline void compute3rdRowOfJacobianOfTransformation(const Vector4& p,Vector6& j);
		
	private:
		struct IterationContext
		{
			const DenseConfig& cfg;
			int level;
			int iteration;
			double error, lastError;
			IterationContext(const DenseConfig& cfg);

			// return true if this is the first iteration
			bool isFirstIteration() const;

			// return true if this is the first iteration on the current level
			bool isFisrtIterationOnLevel() const;

			// returns true if this is the first level
			bool isFirstLevel() const;

			// returns true if this is the last level
			bool isLastLevel() const;

			bool iterationExceeded() const;

			// returns lastError - error
			double errorDiff() const;
		};
	private:
		DenseConfig cfg_;
		IterationContext itctx_;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}
#endif