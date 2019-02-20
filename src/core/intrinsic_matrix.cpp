#include <dvo/core/intrinsic_matrix.h>

namespace dvo
{
	IntrinsicMatrix IntrinsicMatrix::create(float fx, float fy, float cx, float cy)
	{
		IntrinsicMatrix result;
		result.data.setZero();
		result.data(0, 0) = fx;
		result.data(1, 1) = fy;
		result.data(2, 2) = 1.0f;
		result.data(0, 2) = cx;
		result.data(1, 2) = cy;
		return result;
	}

	IntrinsicMatrix::IntrinsicMatrix(const IntrinsicMatrix& other) :data(other.data)
	{

	}
	float IntrinsicMatrix::fx() const
	{
		return data(0, 0);
	}
	float IntrinsicMatrix::fy() const
	{
		return data(1, 1);
	}
	float IntrinsicMatrix::cx() const
	{
		return data(0, 2);
	}
	float IntrinsicMatrix::cy() const
	{
		return data(1, 2);
	}

	void IntrinsicMatrix::invertOffset()
	{
		data(0, 2) *= -1;
		data(1, 2) *= -1;
	}
	void IntrinsicMatrix::scale(float factor)
	{
		data *= factor;
	}
}