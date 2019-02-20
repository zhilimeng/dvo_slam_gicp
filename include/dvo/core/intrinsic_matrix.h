/*!
 * \file intrinsic_matrix.h
 * \brief 
 *
 * \author mengzhili
 * \date 2018/11/21 13:56
 *
 * 
 */
#ifndef  DVO_INTRINSIC_MATRIX_H
#define  DVO_INTRINSIC_MATRIX_H

#include <Eigen/Core>
#include <dvo/core/dvo_exports.h>

namespace dvo
{
struct DVO_EXPORTS IntrinsicMatrix 
{ 
	static IntrinsicMatrix create(float fx, float fy, float cx, float cy);
	IntrinsicMatrix(){}
	IntrinsicMatrix(const IntrinsicMatrix& other);

	float fx() const;
	float fy() const;
	float cx() const;
	float cy() const;

	void invertOffset();
	void scale(float factor);
	Eigen::Matrix3f data;
};
} // namespace dvo
#endif