
#include <assert.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <dvo/core/revertable.h>
#include <dvo/registration/dense_tracking.h>
#include <dvo/registration/dense_tracking_impl.h>
#include <pcl/filters/normal_space.h>
#include <pcl/common/time.h>
#include <fstream>

namespace dvo
{
	DenseTracker::DenseTracker(const DenseConfig& cfg /* = DenseConfig() */) :
		itctx_(cfg_)
	{
		configure(cfg);
	}

	void DenseTracker::configure(const DenseConfig& cfg)
	{
		assert(cfg.isSane());
		cfg_ = cfg;
	}
	bool DenseTracker::match(RgbdImagePyramid &reference, RgbdImagePyramid &current, Matrix4& final_trans, const Matrix4& init_trans /*= Matrix4::Identity()*/)
	{
		Result result;

		bool success = match(reference, current, result, init_trans);
		final_trans = result.transformation;
		return success;
	}


	bool DenseTracker::match(RgbdImagePyramid &reference, RgbdImagePyramid &current, DenseTracker::Result& result, const Matrix4& init_trans /*= Matrix4::Identity()*/)
	{
		reference.build(cfg_.getNumLevels());
		current.build(cfg_.getNumLevels());

		result.setIdensity();

		// compute the tranformation from reference to current,so we need inverse init_trans
		Eigen::Affine3d init_inverse;
		init_inverse.matrix() = init_trans.inverse();
		// our first increment is the given guess
		Sophus::SE3d inc(init_inverse.rotation(), init_inverse.translation());

		Revertable<Sophus::SE3d> initial(inc);
		Revertable<Sophus::SE3d> estimate;

		bool success = true;
		bool accept = true;
		Eigen::Vector2f mean;
		mean.setZero();
		Eigen::Matrix2f precision;
		precision.setZero();
		for (itctx_.level = cfg_.firstLevel; itctx_.level >= cfg_.lastLevel; --itctx_.level)
		{
			mean.setZero();
			precision.setZero();

			// reset error after every pyramid level because errors from different levels are not comparable
			itctx_.iteration = 0;
			itctx_.error = std::numeric_limits<double>::max();

			RgbdImage& cur = current.level(itctx_.level);
			RgbdImage& ref = reference.level(itctx_.level);
			const IntrinsicMatrix& K = cur.camera().intrinsics();

			// build point cloud
			cur.buildPointCloud();
			ref.buildPointCloud();

			// sample point cloud
			if (cfg_.num_sample != 0)
				ref.samplePointCloud(cfg_.num_sample);

			// compute point's jacobian matrix
			ref.computePointsJwJz();

			Vector8f wcur, wref;
			float wcur_id = 0.5f, wref_id = 0.5f, wcur_zd = 1.0f, wref_zd = 0.0f;

			// i z idx idy zdx zdy
			wcur << 1.0f / 255.0f, 1.0f, wcur_id * K.fx() / 255.0f, wcur_id * K.fy() / 255.0f, wcur_zd * K.fx(), wcur_zd * K.fy(), 0.0f, 0.0f;
			wref << -1.0f / 255.0f, -1.0f, wref_id * K.fx() / 255.0f, wref_id * K.fy() / 255.0f, wref_zd * K.fx(), wref_zd * K.fy(), 0.0f, 0.0f;

			NormalEquationsLeastSquares ls;
			Matrix6d A;
			Vector6d x, b;
			x = inc.log();

			size_t validPixels = ref.indices.size();
			IntensityAndDepthVector residual_result(validPixels);
			std::vector<int> valid_indices(validPixels);
			std::vector<float> weight(validPixels,1.0);

			do
			{
				double total_error = 0.0f;
				Eigen::Affine3f transformf;

				inc = Sophus::SE3d::exp(x);
				initial.update() = inc.inverse() * initial();
				estimate.update() = inc * estimate();

				transformf = estimate().matrix().cast<float>();

				//compute intensity and depth residuals
				int n = computeResiduals(ref, cur, K, transformf, wref, wcur,valid_indices, residual_result);
				if (n < 6)
				{
					initial.revert();
					estimate.revert();
					result.terminationCriterion = TerminationCriteria::TooFewConstraints;
					success = false;
					break;
				}
				if (!itctx_.isFisrtIterationOnLevel())
					computeWeights(residual_result, n, mean, precision, weight);
				precision = computeScale(residual_result, n, weight, mean).inverse();

				float ll = computeCompleteDataLogLikelihood(residual_result, n, weight, mean, precision);
				total_error = -ll;
				itctx_.lastError = itctx_.error;
				itctx_.error = total_error;

				accept = itctx_.error < itctx_.lastError;
				if (!accept)
				{
					initial.revert();
					estimate.revert();
					result.terminationCriterion = TerminationCriteria::LogLikelihoodDecreased;
					break;
				}

				Matrix2x6 J, Jw;
				Eigen::Vector2f Ji;
				Vector6 Jz;
				ls.initialize(1);

				for (int i = 0; i < n; ++i)
				{
					const IntensityAndDepth& pt_error = residual_result[i];
					//computeJacobianOfProjectionAndTransformation(pt_error.getVector4fMap(), Jw);
					//compute3rdRowOfJacobianOfTransformation(pt_error.getVector4fMap(), Jz);
					Jw = ref.jws[valid_indices[i]];
					Jz = ref.jzs[valid_indices[i]];
					J.row(0) = pt_error.getIntensityDerivativeVec2f().transpose() *Jw;
					J.row(1) = pt_error.getDepthDerivativeVec2f().transpose()*Jw - Jz.transpose();

					ls.update(J, pt_error.getIntensityAndDepthVec2f(), weight[i] * precision);
				}
				ls.finish();
				A = ls.A.cast<double>() + cfg_.mu * Matrix6d::Identity();
				b = ls.b.cast<double>() + cfg_.mu * initial().log();
				x = A.ldlt().solve(b);
				itctx_.iteration++;
			} while (accept && x.lpNorm<Eigen::Infinity>() > cfg_.precision && !itctx_.iterationExceeded());

			if (x.lpNorm<Eigen::Infinity>() <= cfg_.precision)
				result.terminationCriterion = TerminationCriteria::IncrementTooSmall;
			if (itctx_.iterationExceeded())
				result.terminationCriterion = TerminationCriteria::IterationsExceeded;
			result.information = A;
		}
		result.transformation = estimate().inverse().matrix().matrix();
		return success;
	}

inline void DenseTracker::computeJacobianOfProjectionAndTransformation(const Vector4& p, Matrix2x6& j)
	{
		NumType z = 1.0f / p(2);
		NumType z_sqr = 1.0f / (p(2) * p(2));

		j(0, 0) = z;
		j(0, 1) = 0.0f;
		j(0, 2) = -p(0) * z_sqr;
		j(0, 3) = j(0, 2) * p(1);//j(0, 3) = -p(0) * p(1) * z_sqr;
		j(0, 4) = 1.0f - j(0, 2) * p(0);//j(0, 4) =  (1.0 + p(0) * p(0) * z_sqr);
		j(0, 5) = -p(1) * z;

		j(1, 0) = 0.0f;
		j(1, 1) = z;
		j(1, 2) = -p(1) * z_sqr;
		j(1, 3) = -1.0f + j(1, 2) * p(1); //j(1, 3) = -(1.0 + p(1) * p(1) * z_sqr);
		j(1, 4) = -j(0, 3); //j(1, 4) =  p(0) * p(1) * z_sqr;
		j(1, 5) = p(0) * z;
	}

	inline void DenseTracker::compute3rdRowOfJacobianOfTransformation(const Vector4& p, Vector6& j)
	{
		j(0) = 0.0;
		j(1) = 0.0;
		j(2) = 1.0;
		j(3) = p(1);
		j(4) = -p(0);
		j(5) = 0.0;
	}
}
