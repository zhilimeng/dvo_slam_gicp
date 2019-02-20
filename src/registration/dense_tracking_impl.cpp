
#include <dvo/registration/dense_tracking_impl.h>
#ifdef __CDT_PARSER__
#define __SSE3__
#endif

#include <immintrin.h>
#include <pmmintrin.h>

namespace dvo
{
	int computeResiduals(const RgbdImage &reference, const RgbdImage &current, const IntrinsicMatrix& intrinsics, const Eigen::Affine3f& transform, const Vector8f& reference_weight, const Vector8f& current_weight, std::vector<int> &valid_indices, IntensityAndDepthVector &result)
	{
		Eigen::Matrix<float, 3, 3> K;
		K << intrinsics.fx(), 0, intrinsics.cx(),
			0, intrinsics.fy(), intrinsics.cy(),
			0, 0, 1;

		Eigen::Matrix<float, 3, 4> KT = K * transform.matrix().block<3, 4>(0, 0);
		Eigen::Vector4f transformed_point;
		transformed_point.setConstant(1);

		int n_valid = 0;
		for (size_t i = 0; i < reference.indices.size(); ++i)
		{
			size_t idx = reference.indices[i];
			const pcl::PointXYZRGBNormal& ref_pt = reference.pointcloud->points[idx];
			transformed_point.head<3>() = KT * ref_pt.getVector4fMap();
			float projected_x = transformed_point(0) / transformed_point(2);
			float projected_y = transformed_point(1) / transformed_point(2);
			if (!current.inImage(projected_x, projected_y) || !current.inImage(projected_x + 1, projected_y + 1))
				continue;

			float x0 = std::floor(projected_x);
			float y0 = std::floor(projected_y);
			float x0w, x1w, y0w, y1w;
			x1w = projected_x - x0;
			x0w = 1.0f - x1w;
			y1w = projected_y - y0;
			y0w = 1.0f - y1w;

			Vector8f x0y0, x1y0, x0y1, x1y1;

			x0y0 = current.accelerationAt(int(x0), int(y0));
			x0y1 = current.accelerationAt(int(x0), int(y0) + 1);
			x1y0 = current.accelerationAt(int(x0) + 1, int(y0));
			x1y1 = current.accelerationAt(int(x0) + 1, int(y0) + 1);

			Vector8f interpolated = (x0y0 * x0w + x1y0 * x1w) * y0w + (x0y1 * x0w + x1y1 * x1w) * y1w;

			if (!std::isfinite(interpolated(1)) || !std::isfinite(interpolated(4)) || !std::isfinite(interpolated(5)))
				continue;
			valid_indices[n_valid] = i;

			Vector8f ref_idg = reference.acceleration[idx].getIntensityAndDepthWithDerivativesVec8f();

			ref_idg(1) = transformed_point(2);
			result[n_valid].getIntensityAndDepthWithDerivativesVec8f() = current_weight.cwiseProduct(interpolated) + reference_weight.cwiseProduct(ref_idg);
			n_valid++;
		}
		//		result.resize(n_valid);
		return n_valid;
	}


	void computeWeights(const IntensityAndDepthVector &result, int n, const Eigen::Vector2f& mean, const Eigen::Matrix2f& precision, std::vector<float> &weights)
	{
		Eigen::Vector2f diff;
		for (int i = 0; i < n; ++i)
		{
			diff = result[i].getIntensityAndDepthVec2f() - mean;
			weights[i] = (2.0 + 5.0f) / (5.0f + diff.transpose()*precision*diff);
		}
	}

	Eigen::Matrix2f computeScale(const IntensityAndDepthVector &result, int n, const std::vector<float> &weights, const Eigen::Vector2f &mean)
	{
		Eigen::Matrix2f covariance;
		covariance.setZero();

		float scale = 1.0f / (n - 2 - 1);

		Eigen::Vector2f diff;
		for (int i = 0; i < n; ++i)
		{
			diff = result[i].getIntensityAndDepthVec2f() - mean;
			covariance += scale *(weights[i] * diff * diff.transpose());
		}
		return covariance;
	}

	float computeCompleteDataLogLikelihood(const IntensityAndDepthVector &result, int n, const std::vector<float> &weights, const Eigen::Vector2f &mean, const Eigen::Matrix2f& precision)
	{
		int c = 1;
		double error_sum = 0.0;
		double error_acc = 1.0;

		for (int i = 0; i < n; ++i, ++c)
		{
			error_acc *= (1.0 + 0.2 *(result[i].getIntensityAndDepthVec2f().transpose() * precision * result[i].getIntensityAndDepthVec2f())(0, 0));
			if ((c % 50) == 0)
			{
				error_sum += std::log(error_acc);
				error_acc = 1.0;
			}
		}

		return 0.5 * n * std::log(precision.determinant()) - 0.5 *(5.0 + 2.0) * error_sum;
	}

}