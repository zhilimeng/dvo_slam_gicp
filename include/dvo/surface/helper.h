#ifndef DVO_HELPER_H
#define DVO_HELPER_H

#include <dvo/core/rgbd_image.h>
#include <dvo/core/hash_eigen.h>

namespace dvo
{
	cv::Mat createDepthToCameraDistanceMultiplierImage(const IntrinsicMatrix& intrinsic, int width, int height);

	const Eigen::Vector3i shift[8] = {
		Eigen::Vector3i(0, 0, 0),
		Eigen::Vector3i(1, 0, 0),
		Eigen::Vector3i(1, 1, 0),
		Eigen::Vector3i(0, 1, 0),
		Eigen::Vector3i(0, 0, 1),
		Eigen::Vector3i(1, 0, 1),
		Eigen::Vector3i(1, 1, 1),
		Eigen::Vector3i(0, 1, 1),
	};

	const Eigen::Vector4i edge_shift[12] = {
		Eigen::Vector4i(0, 0, 0, 0),
		Eigen::Vector4i(1, 0, 0, 1),
		Eigen::Vector4i(0, 1, 0, 0),
		Eigen::Vector4i(0, 0, 0, 1),
		Eigen::Vector4i(0, 0, 1, 0),
		Eigen::Vector4i(1, 0, 1, 1),
		Eigen::Vector4i(0, 1, 1, 0),
		Eigen::Vector4i(0, 0, 1, 1),
		Eigen::Vector4i(0, 0, 0, 2),
		Eigen::Vector4i(1, 0, 0, 2),
		Eigen::Vector4i(1, 1, 0, 2),
		Eigen::Vector4i(0, 1, 0, 2),
	};

	const int edge_to_vert[12][2] = {
		{ 0, 1 },
		{ 1, 2 },
		{ 3, 2 },
		{ 0, 3 },
		{ 4, 5 },
		{ 5, 6 },
		{ 7, 6 },
		{ 4, 7 },
		{ 0, 4 },
		{ 1, 5 },
		{ 2, 6 },
		{ 3, 7 },
	};
}
#endif
