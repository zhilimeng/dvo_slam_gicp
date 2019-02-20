/*!
 * \file eigen.h
 * \brief this is from open3d changed to a template version
 * 
 * \author mengzhili
 * \date 2018/11/20 19:52
 *
 * 
 */

#ifndef DVO_EIGEN_H
#define DVO_EIGEN_H 
#include <tuple>
#include <vector>
#include <Eigen/Core>
namespace Eigen
{
	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	typedef Eigen::Matrix<float, 6, 6> Matrix6f;
	typedef Eigen::Matrix<float, 6, 1> Vector6f;
}
namespace dvo
{
	/// Function to transform 6D motion vector to 4D motion matrix
	/// Reference:
	/// https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html#TutorialGeoTransform

	template<typename Scalar> Eigen::Matrix<Scalar, 4, 4> 
		transformVector6ToMatrix4(const Eigen::Matrix<Scalar, 6, 1> &input);

/// Function to transform 4D motion matrix to 6D motion vector
	/// this is consistent with the matlab function in
	/// the Aerospace Toolbox
	/// Reference: https://github.com/qianyizh/ElasticReconstruction/blob/master/Matlab_Toolbox/Core/mrEvaluateRegistration.m
	template<typename Scalar> Eigen::Matrix<Scalar, 6, 1> 
		transformMatrix4ToVector6(const Eigen::Matrix<Scalar, 4, 4> &input);

	template<typename MatType, typename VecType,typename Scalar> std::tuple<bool,VecType>
		solveLinearSystem(const MatType &A, const VecType &b, bool check_det = true);

	/// Function to solve Jacobian system
	/// Input: 6x6 Jacobian matrix and 6-dim residual vector.
	/// Output: tuple of is_success, 4x4 extrinsic matrices.
	template<typename Scalar> std::tuple<bool, Eigen::Matrix<Scalar,4,4> >
		solveJacobianSystemAndObtainExtrinsicMatrix(const Eigen::Matrix<Scalar, 6, 6> &JTJ, const Eigen::Matrix<Scalar, 6, 1> &JTr);

/// Function to compute JTJ and Jtr
	/// Input: function pointer f and total number of rows of Jacobian matrix
	/// Output: JTJ and JTr
	/// Note: f takes index of row, and outputs corresponding residual and row vector.
    template<typename MatType, typename VecType, typename Scalar> std::tuple<MatType, VecType> 
		computeJTJandJTr(std::function<void(int, VecType &, Scalar &)> f, int iteration_num);

/// Function to compute JTJ and Jtr
	/// Input: function pointer f and total number of rows of Jacobian matrix
	/// Output: JTJ and JTr
	/// Note: f takes index of row, and outputs corresponding residual and row vector.
	template<typename MatType, typename VecType, typename Scalar> std::tuple<MatType, VecType>
		vectorComputeJTJandJTr(std::function<void(int, std::vector<VecType> &, std::vector<Scalar> &)> f, int iteration_num);
}

#include <dvo/core/impl/eigen.hpp>
#endif