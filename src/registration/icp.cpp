#ifndef DVO_ICP_HPP
#define DVO_ICP_HPP

#include <dvo/registration/icp.h>

#include <dvo/core/revertable.h>
#include <dvo/core/eigen.h>
namespace dvo
{
//////////////////////////////////////////////////////////////////////////
bool ICPWithNormals::initCompute()
{
	const size_t n_min = 4;
	if (input_->size() < n_min || target_->size() < n_min)
	{
		std::cerr << "Error in icp.cpp: Not enough input point!\n";
		return false;
	}
	if (!tree_)
	{
		tree_.reset(new pcl::KdTreeFLANN<PointT>());
		tree_->setInputCloud(target_);
	}
	if (source_indices_->empty())
	{
		source_indices_->resize(input_->size());
		for (size_t i = 0; i < input_->size(); ++i)
		{
			(*source_indices_)[i] = i;
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////
bool ICPWithNormals::align(const Matrix4d &init_transformation)
{
	if (!initCompute())
		return false;

	// Convergence and registration failure
	const size_t n_min = 6;
	float current_error = std::numeric_limits <float>::max();
	float previous_error = std::numeric_limits <float>::max();
	float mean = 0.f;
	float precision = 0.f;

	// Outlier rejection
	float squared_distance_threshold = corr_dist_threshold_ * corr_dist_threshold_;
	float dot_min = std::cos(max_angle_ * 17.45329252e-3);
	bool success = true;
	bool accept = true;

	size_t num_valid_pts = source_indices_->size();
	std::vector<PointT> tgt_corr_cloud(num_valid_pts);
	std::vector<PointT> src_corr_cloud(num_valid_pts);
	Vec4Xf xyz_s(num_valid_pts);
	Vec4Xf xyz_t(num_valid_pts);
	Vec4Xf nor_t(num_valid_pts);

	pcl::CorrespondencesPtr corrs(new pcl::Correspondences);
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);

	std::vector<float> residuals(num_valid_pts, 0.f);
	std::vector<float> weights(num_valid_pts, 1.0f);

	Matrix4d inc = Matrix4d::Identity();					// 位姿增量
	Revertable<Matrix4d> estimate(init_transformation);	// 估计位姿，初始值为初始位姿
	Revertable<pcl::CorrespondencesPtr> estimate_corr(corrs);	//一致性点对

	Matrix6d ATA;
	Vector6d ATb;
	Vector6d x;
	ATA.setZero();
	ATb.setZero();
	x.setZero();

	PointT pt_d;
	unsigned int iter = 1;
	do
	{
		estimate.update() = inc * estimate();
		Eigen::Matrix4f transformf = estimate().cast<float>();
		// find correspondences
		int n_valid_corr = 0;
		corrs->clear();
		for (const auto& idx : *source_indices_)
		{
			// tranform source point
			pt_d = input_->points[idx];
			pt_d.getVector4fMap() = transformf * pt_d.getVector4fMap();
			pt_d.getNormalVector4fMap() = transformf * pt_d.getNormalVector4fMap();

			tree_->nearestKSearch(pt_d, 1, nn_indices, nn_dists);

			// check distance threshold
			if (nn_dists[0] < squared_distance_threshold)
			{
				const PointT& pt_m = target_->points[nn_indices[0]];
				// check angle threshold
				if (pt_m.getNormalVector4fMap().dot(pt_d.getNormalVector4fMap()) > dot_min)
				{
					tgt_corr_cloud[n_valid_corr] = pt_m;
					src_corr_cloud[n_valid_corr] = pt_d;
					pcl::Correspondence corr(idx, nn_indices[0], nn_dists[0]);
					corrs->push_back(corr);
					residuals[n_valid_corr] = std::sqrt(nn_dists[0]);
					n_valid_corr++;
				} // end check angle
			}// end check distance
		}// end for loop

		estimate_corr.update() = corrs;
		if (n_valid_corr < n_min)
		{
			estimate.revert();
			estimate_corr.revert();
			success = false;
			break;
		}
		if (iter != 1)
			computeWeights(residuals, n_valid_corr, mean, precision, weights);
		precision = computeScale(residuals, n_valid_corr, weights, mean);
		float ll = computeCompleteDataLogLikelihood(residuals, n_valid_corr, weights, mean, precision);
		previous_error = current_error;
		current_error = -ll;
		accept = current_error < previous_error;
		if (!accept)
		{
			estimate.revert();
			estimate_corr.revert();
			break;
		}

		// compute cenroid and demean point cloud
		Eigen::Vector4f c_s, c_t;
		float factor = 0.f;
		dvo::compute3DCentroid(src_corr_cloud, n_valid_corr, 10, c_s);
		dvo::compute3DCentroid(tgt_corr_cloud, n_valid_corr, 10, c_t);
		float accum = 0.f;
		for (int i = 0; i < n_valid_corr; ++i)
		{
			xyz_s[i] = src_corr_cloud[i].getVector4fMap() - c_s;
			xyz_t[i] = tgt_corr_cloud[i].getVector4fMap() - c_t;
			nor_t[i] = tgt_corr_cloud[i].getNormalVector4fMap();
			accum += xyz_s[i].head<3>().norm() + xyz_t[i].head<3>().norm();
		}
		factor = 2.f * static_cast<float>(n_valid_corr) / accum;

		// compute ATA and ATb
		computeATAandATb(xyz_s, xyz_t, nor_t, n_valid_corr, factor, ATA, ATb);
		// solve C * x = b with a Cholesky factorization with pivoting
		// x = [alpha;beta;gamma;trans_x;trans_y;trans_z]
		x = ATA.selfadjointView<Eigen::Lower>().ldlt().solve(ATb);
		Matrix4d TT = dvo::transformVector6ToMatrix4(x);

		// Transformation matrixes into the local coordinate systems of model/data
		Matrix4d T_s, T_t;

		T_s << factor, 0.f, 0.f, -c_s.x() * factor,
			0.f, factor, 0.f, -c_s.y() * factor,
			0.f, 0.f, factor, -c_s.z() * factor,
			0.f, 0.f, 0.f, 1.f;

		T_t << factor, 0.f, 0.f, -c_t.x() * factor,
			0.f, factor, 0.f, -c_t.y() * factor,
			0.f, 0.f, factor, -c_t.z() * factor,
			0.f, 0.f, 0.f, 1.f;

		// Output transformation T
		inc = T_t.inverse() * TT * T_s;
		iter++;
	} while (accept && x.lpNorm<Eigen::Infinity>() > precision_ && iter < max_iterations_);

	final_transformation_ = estimate();
	information_ = ATA;
	corrs_ = estimate_corr();
	return success;
}

//////////////////////////////////////////////////////////////////////////
void ICPWithNormals::computeWeights(const std::vector<float>& residuals, int n, float mean, float precison, std::vector<float>& weights)
{
	for (int i = 0; i < n; ++i)
	{
		float diff = residuals[i] - mean;
		weights[i] = (2.0 + 5.0) / (5.0f + diff * precison * diff);
	}
}
//////////////////////////////////////////////////////////////////////////
float ICPWithNormals::computeScale(const std::vector<float>& residuals, int n, const std::vector<float>& weights, float mean)
{
	float precision = 0.f;
	float scale = 1.f / (n - 2 - 1);
	for (int i = 0; i < n; ++i)
	{
		float diff = residuals[i] - mean;
		precision += scale * (weights[i] * diff *diff);
	}
	return 1.f / precision;
}
//////////////////////////////////////////////////////////////////////////
float ICPWithNormals::computeCompleteDataLogLikelihood(const std::vector<float>& residuals, int n, const std::vector<float>& weights, float mean, float precision)
{
	int c = 1;
	double error_sum = 0.0;
	double error_acc = 1.0;
	for (int i = 0; i < n; ++i, ++c)
	{
		error_acc *= (1.0 + 0.2 *(residuals[i] * precision * residuals[i]));
		if ((c % 50) == 0)
		{
			error_sum += std::log(error_acc);
			error_acc = 1.0;
		}
	}
	return 0.5 * n * std::log(precision) - 0.5 * (5.0 + 2.0) * error_sum;
}
//////////////////////////////////////////////////////////////////////////
void ICPWithNormals::computeATAandATb(const Vec4Xf& xyz_s, const Vec4Xf& xyz_t, const Vec4Xf& nor_t, int n, float scale, Matrix6d& ATA, Vector6d& ATb)
{
	const float factor = scale;
	const float factor_squared = factor * factor;

	// Covariance matrix C
	Eigen::Matrix <float, 6, 6> C;

	// Right hand side vector b
	Eigen::Matrix <float, 6, 1> b;

	// For Eigen vectorization: use 4x4 submatrixes instead of 3x3 submatrixes
	// -> top left 3x3 matrix will form the final C
	// Same for b
	Eigen::Matrix4f C_tl = Eigen::Matrix4f::Zero(); // top left corner
	Eigen::Matrix4f C_tr_bl = Eigen::Matrix4f::Zero(); // top right / bottom left
	Eigen::Matrix4f C_br = Eigen::Matrix4f::Zero(); // bottom right

	Eigen::Vector4f b_t = Eigen::Vector4f::Zero(); // top
	Eigen::Vector4f b_b = Eigen::Vector4f::Zero(); // bottom

	Eigen::Vector4f cross;
	float dot;
	for (int i = 0; i < n; ++i)
	{
		cross = xyz_s[i].cross3(nor_t[i]);

		C_tl += cross     * cross.transpose();
		C_tr_bl += cross     * nor_t[i].transpose();
		C_br += nor_t[i] * nor_t[i].transpose();

		dot = (xyz_t[i] - xyz_s[i]).dot(nor_t[i]);

		b_t += cross     * dot;
		b_b += nor_t[i] * dot;
	}

	// Scale with the factor and copy the 3x3 submatrixes into C and b
	C_tl *= factor_squared;
	C_tr_bl *= factor;

	ATA << C_tl(0, 0), C_tl(0, 1), C_tl(0, 2), C_tr_bl(0, 0), C_tr_bl(0, 1), C_tr_bl(0, 2),
		C_tl(1, 0), C_tl(1, 1), C_tl(1, 2), C_tr_bl(1, 0), C_tr_bl(1, 1), C_tr_bl(1, 2),
		C_tl(2, 0), C_tl(2, 1), C_tl(2, 2), C_tr_bl(2, 0), C_tr_bl(2, 1), C_tr_bl(2, 2),
		C_tr_bl(0, 0), C_tr_bl(1, 0), C_tr_bl(2, 0), C_br(0, 0), C_br(0, 1), C_br(0, 2),
		C_tr_bl(0, 1), C_tr_bl(1, 1), C_tr_bl(2, 1), C_br(1, 0), C_br(1, 1), C_br(1, 2),
		C_tr_bl(0, 2), C_tr_bl(1, 2), C_tr_bl(2, 2), C_br(2, 0), C_br(2, 1), C_br(2, 2);

	ATb << b_t(0)*factor_squared, b_t(1)*factor_squared, b_t(2)*factor_squared,
		b_b(0)*factor, b_b(1)*factor, b_b(2)*factor;
}
} // namespace dvo

#endif