
#ifndef DVO_EIGEN_HPP
#define DVO_EIGEN_HPP

#include <iostream>
#include <Eigen/Geometry>
#include <dvo/core/eigen.h>
#ifdef _OPENMP
#include <omp.h>
#endif
//////////////////////////////////////////////////////////////////////////
template<typename Scalar> Eigen::Matrix<Scalar, 4, 4>
	dvo::transformVector6ToMatrix4(const Eigen::Matrix<Scalar, 6, 1> &input)
	{
		typedef typename Eigen::Matrix<Scalar, 4, 4> Matrix4;
		typedef typename Eigen::Matrix<Scalar, 3, 1> Vector3;
		typedef typename Eigen::AngleAxis<Scalar> AngleAxis;
		Matrix4 output;
		output.setIdentity();
		output.block<3, 3>(0, 0) =
			(AngleAxis(input(2), Vector3::UnitZ()) *
			AngleAxis(input(1), Vector3::UnitY()) *
			AngleAxis(input(0), Vector3::UnitX())).matrix();
		output.block<3, 1>(0, 3) = input.block<3, 1>(3, 0);
		return output;
	}

//////////////////////////////////////////////////////////////////////////
template<typename Scalar> Eigen::Matrix<Scalar, 6, 1>
	dvo::transformMatrix4ToVector6(const Eigen::Matrix<Scalar, 4, 4> &input)
	{
		typedef typename  Eigen::Matrix<Scalar, 6, 1> Matrix6;
		typedef typename Eigen::Matrix<Scalar, 3, 3> Matrix3;
		Matrix6 output;
		Matrix3 R = input.block<3, 3>(0, 0);
		double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
		if (!(sy < 1e-6)) {
			output(0) = atan2(R(2, 1), R(2, 2));
			output(1) = atan2(-R(2, 0), sy);
			output(2) = atan2(R(1, 0), R(0, 0));
		}
		else {
			output(0) = atan2(-R(1, 2), R(1, 1));
			output(1) = atan2(-R(2, 0), sy);
			output(2) = 0;
		}
		output.block<3, 1>(3, 0) = input.block<3, 1>(0, 3);
		return output;
	}

template<typename MatType, typename VecType, typename Scalar> std::tuple<bool, VecType>
	dvo::solveLinearSystem(const MatType &A, const VecType &b, bool check_det /*= true*/)
{
	if (check_det)
	{
		bool solution_exist = true;
		Scalar det = A.determinant();
		if (fabs(det) < 1e-6 || std::isnan(det) || std::isinf(det))
			solution_exist = false;
		if (solution_exist) {
			// Robust Cholesky decomposition of a matrix with pivoting.
			VecType x = A.ldlt().solve(b);
			return std::make_tuple(solution_exist, std::move(x));
		}
		else {
			return std::make_tuple(false,
				std::move(VecType::Zero(b.rows())));
		}
	}
	else {
		VecType x = A.ldlt().solve(b);
		return std::make_tuple(true, std::move(x));
	}
}

//////////////////////////////////////////////////////////////////////////
template<typename Scalar> std::tuple<bool, Eigen::Matrix<Scalar, 4, 4> >
	dvo::solveJacobianSystemAndObtainExtrinsicMatrix(const Eigen::Matrix<Scalar, 6, 6> &JTJ, const Eigen::Matrix<Scalar, 6, 1> &JTr)
{
	typedef typename Eigen::Matrix<Scalar, 4, 4> Matrix4;
	typedef typename Eigen::Matrix<Scalar, 6, 6> Matrix6;
	typedef typename Eigen::Matrix<Scalar, 6, 1> Vector6;
	std::vector<Matrix4> output_matrix_array;
	output_matrix_array.clear();

	bool solution_exist;
	Vector6 x;
	std::tie(solution_exist, x) = solveLinearSystem<Matrix6,Vector6,Scalar>(JTJ, -JTr);

	if (solution_exist) {
		Matrix4 extrinsic = transformVector6ToMatrix4(x);
		return std::make_tuple(solution_exist, std::move(extrinsic));
	}
	else {
		return std::make_tuple(false, std::move(Matrix4::Identity()));
	}
}
//////////////////////////////////////////////////////////////////////////

template<typename MatType, typename VecType, typename Scalar> std::tuple<MatType, VecType>
	dvo::computeJTJandJTr(std::function<void(int, VecType &, Scalar &)> f, int iteration_num)
{
	MatType JTJ;
	VecType JTr;
	Scalar r2_sum = 0.0;
	JTJ.setZero();
	JTr.setZero();
#ifdef _OPENMP
#pragma omp parallel
	{
#endif
		MatType JTJ_private;
		VecType JTr_private;
		Scalar r2_sum_private = 0.0;
		JTJ_private.setZero();
		JTr_private.setZero();
		VecType J_r;
		Scalar r;
#ifdef _OPENMP
#pragma omp for nowait
#endif
		for (int i = 0; i < iteration_num; i++) 
		{
			f(i, J_r, r);
			JTJ_private.noalias() += J_r * J_r.transpose();
			JTr_private.noalias() += J_r * r;
			r2_sum_private += r * r;
		}
#ifdef _OPENMP
#pragma omp critical
		{
#endif
			JTJ += JTJ_private;
			JTr += JTr_private;
			r2_sum += r2_sum_private;
#ifdef _OPENMP
		}
	}
#endif
	r2_sum /= (Scalar)iteration_num;
	//PrintDebug("Residual : %.2e (# of elements : %d)\n", r2_sum,
	//	iteration_num);
	return std::make_tuple(std::move(JTJ), std::move(JTr));
}
//////////////////////////////////////////////////////////////////////////
template<typename MatType, typename VecType, typename Scalar> std::tuple<MatType, VecType>
	dvo::vectorComputeJTJandJTr(std::function<void(int, std::vector<VecType> &, std::vector<Scalar> &)> f, int iteration_num)
{
	MatType JTJ;
	VecType JTr;
	Scalar r2_sum = 0.0;
	JTJ.setZero();
	JTr.setZero();
#ifdef _OPENMP
#pragma omp parallel
	{
#endif
		MatType JTJ_private;
		VecType JTr_private;
		double r2_sum_private = 0.0;
		JTJ_private.setZero();
		JTr_private.setZero();
		std::vector<Scalar> r;
		std::vector<VecType> J_r;
#ifdef _OPENMP
#pragma omp for nowait
#endif
		for (int i = 0; i < iteration_num; i++) 
		{
			f(i, J_r, r);
			for (int j = 0; j < (int)r.size(); j++) 
			{
				JTJ_private.noalias() += J_r[j] * J_r[j].transpose();
				JTr_private.noalias() += J_r[j] * r[j];
				r2_sum_private += r[j] * r[j];
			}
		}
#ifdef _OPENMP
#pragma omp critical
		{
#endif
			JTJ += JTJ_private;
			JTr += JTr_private;
			r2_sum += r2_sum_private;
#ifdef _OPENMP
		}
	}
#endif
	r2_sum /= (Scalar)iteration_num;
	return std::make_tuple(std::move(JTJ), std::move(JTr));
}

#endif // !DVO_EIGEN_HPP
