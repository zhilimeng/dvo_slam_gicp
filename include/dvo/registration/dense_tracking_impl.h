#ifndef  DVO_DENSE_TRACKING_IMPL_H
#define DVO_DENSE_TRACKING_IMPL_H


#include <dvo/registration/dense_tracking.h>
namespace dvo
{

	int computeResiduals(const RgbdImage &reference, const RgbdImage &current, const IntrinsicMatrix& intrinsics,
		const Eigen::Affine3f& transform, const Vector8f& reference_weight, const Vector8f& current_weight,std::vector<int> &valid_indices, IntensityAndDepthVector &result);

	void computeWeights(const IntensityAndDepthVector &result, int n, const Eigen::Vector2f& mean, const Eigen::Matrix2f& precision, std::vector<float> &weights);

	Eigen::Matrix2f computeScale(const IntensityAndDepthVector &result, int n, const std::vector<float> &weights, const Eigen::Vector2f &mean);

	float computeCompleteDataLogLikelihood(const IntensityAndDepthVector &result, int n, const std::vector<float> &weights, const Eigen::Vector2f &mean, const  Eigen::Matrix2f& precision);

}
#endif