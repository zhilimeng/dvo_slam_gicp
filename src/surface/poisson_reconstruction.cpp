#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <stdarg.h>
#ifdef _WIN32
#include <Windows.h>
#include <Psapi.h>
#endif // _WIN32
#include <dvo/surface/poisson8/marching_cubes.h>
#include <dvo/surface/poisson8/sparse_matrix.h>
#include <dvo/surface/poisson8/octree.h>
#include <dvo/surface/poisson8/multi_grid_octree_data.h>

#include <dvo/surface/poisson8/ppolynomial.h>
#include <dvo/surface/poisson8/ply.h>
#ifdef _OPENMP
#include "omp.h"
#endif
#include <dvo/surface/poisson8/surface_trimmer.h>
#include <dvo/surface/poisson_reconstruction.h>
#include <pcl/common/time.h>
float exp_linear(const float &x)
{
	float x2 = x * x;
	return static_cast<float>(1 + x + 0.5 * x2 + 0.1666667 * x * x2 + 0.0416667 * x2 * x2);
}

/* * \brief this is for parallelization of kd tree search function  */
class Compute_k_neighbors
{
	typedef pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr Tree;
	int m_k;
	const pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr &m_tree;
	std::vector<std::vector<int> > &m_neighbors_indices;
	std::vector<std::vector<float> > &m_neighbors_dists;
	std::vector<float> &m_neighbors_aver_dists;
public:
	Compute_k_neighbors(int k, const Tree &tree, std::vector<std::vector<int> > &neighbors_indices,
		std::vector<std::vector<float> > &neighbors_dists, std::vector<float> &neighbors_aver_dists)
		:m_k(k), m_tree(tree), m_neighbors_indices(neighbors_indices), m_neighbors_dists(neighbors_dists), m_neighbors_aver_dists(neighbors_aver_dists){}

	void operator()(const tbb::blocked_range<size_t>& r) const
	{
		for (size_t i = r.begin(); i != r.end(); ++i)
		{
			m_tree->nearestKSearch(i, m_k + 1, m_neighbors_indices[i], m_neighbors_dists[i]);
			double dist_sum = 0.0;
			for (int k = 1; k < m_k + 1; ++k)
			{
				dist_sum += std::sqrt(m_neighbors_dists[i][k]);
			}
			m_neighbors_aver_dists[i] = static_cast<float>(dist_sum / m_k);
		}
	}
};

class Outlier_removal
{
	int m_k;
	float m_std_mul;
	const std::vector<std::vector<int> > &m_neighbors_indices;
	const std::vector<std::vector<float> > &m_neighbors_dists;
	const std::vector<float> &m_neighbors_aver_dists;
	std::vector<bool> &m_indices_flag;
public:
	Outlier_removal(int k, float std_mul, const std::vector<std::vector<int> > &neighbors_indices, const std::vector<std::vector<float> > &neighbor_dists,
		const std::vector<float> &neighbors_aver_dists, std::vector<bool> &indices_flag)
		:m_k(k), m_std_mul(std_mul), m_neighbors_indices(neighbors_indices), m_neighbors_dists(neighbor_dists), m_neighbors_aver_dists(neighbors_aver_dists), m_indices_flag(indices_flag){}

	void operator()(const tbb::blocked_range<size_t>& r) const
	{
		for (size_t i = r.begin(); i != r.end(); ++i)
		{
			double sum = 0, sq_sum = 0;
			for (size_t k = 1; k < m_neighbors_indices[i].size(); ++k)
			{
				int idx = m_neighbors_indices[i][k];
				sum += m_neighbors_aver_dists[idx];
				sq_sum += m_neighbors_aver_dists[idx] * m_neighbors_aver_dists[idx];
			}
			double mean = sum / m_k;
			double variance = (sq_sum - sum * sum / static_cast<double>(m_k)) / (static_cast<double>(m_k)-1);
			double stddev = std::sqrt(variance);

			double distance_threshold = mean + m_std_mul * stddev;
			if (m_neighbors_aver_dists[i] < distance_threshold)
				m_indices_flag[i] = true;
		}
	}
};

class Bilateral_smooth
{
	float  m_radius2;
	float m_sharpness_bandwith;
	const std::vector<std::vector<int> > &m_neighbors_indices;
	const std::vector<std::vector<float> > &m_neighbors_dists;
	const std::vector<bool> &m_indices_flag;;
	const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &m_input;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &m_processed_input;
public:
	Bilateral_smooth(float radius2, float sharpness_bandwith, const std::vector<std::vector<int> > &neighbors_indices,
		const std::vector<std::vector<float> > &neighbors_dists, const std::vector<bool> &indices_flag,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &processed_input)
		:m_radius2(radius2), m_sharpness_bandwith(sharpness_bandwith), m_neighbors_indices(neighbors_indices), m_neighbors_dists(neighbors_dists),
		m_indices_flag(indices_flag), m_input(input), m_processed_input(processed_input){}

	void operator()(const tbb::blocked_range<size_t>& r) const
	{
		for (size_t i = r.begin(); i != r.end(); ++i)
		{
			if (m_indices_flag[i])
			{
				float weight = 0.f;
				float project_dist_sum = 0.f;
				float project_weight_sum = 0.f;
				float iradius16 = -4.0f / m_radius2;

				Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();
				const Eigen::Vector3f &query_point = m_input->points[i].getVector3fMap();
				const Eigen::Vector3f &query_normal = m_input->points[i].getNormalVector3fMap();

				for (size_t k = 1; k < m_neighbors_indices[i].size(); ++k)
				{
					float dist2 = m_neighbors_dists[i][k];
					int idx = m_neighbors_indices[i][k];

					if (m_indices_flag[idx])
					{
						const Eigen::Vector3f &np = m_input->points[idx].getVector3fMap();
						const Eigen::Vector3f &nn = m_input->points[idx].getNormalVector3fMap();
						if (dist2 < m_radius2)
						{
							float theta = std::exp(dist2 * iradius16);
//							float theta = exp_linear(dist2 * iradius16);
							float angle = 1 - query_normal.dot(nn);
							float psi = std::exp(-angle * angle / m_sharpness_bandwith);
//							float psi = exp_linear(-angle * angle / m_sharpness_bandwith);
							weight = theta * psi;
							project_dist_sum += ((query_point - np).dot(nn))*weight;
							project_weight_sum += weight;
							normal_sum += nn * weight;
						}
					}
				}
				Eigen::Vector3f update_normal = normal_sum / project_weight_sum;
				update_normal.normalize();

				Eigen::Vector3f update_point = query_point - update_normal *(project_dist_sum / project_weight_sum);

				pcl::PointXYZRGBNormal pt;
				pt.x = update_point(0);
				pt.y = update_point(1);
				pt.z = update_point(2);
				pt.r = m_input->points[i].r;
				pt.g = m_input->points[i].g;
				pt.b = m_input->points[i].b;
				pt.normal_x = update_normal(0);
				pt.normal_y = update_normal(1);
				pt.normal_z = update_normal(2);

				m_processed_input->push_back(pt);
			}
		}

	}

};
PoissonReconstruction::PoissonReconstruction() :use_confidence_(false), use_normal_weights_(false), use_nonManifold_(false), use_dirichlet_(false), use_density_(true),
use_linear_fit_(false), use_primal_voxel_(false), use_polygon_mesh_(false), use_double_(false), make_complete_(false), degree_(2), cg_depth_(0), kernel_depth_(8), adaptive_exponent_(1), iters_(8),
voxel_depth_(-1), full_depth_(5), min_depth_(0), max_solve_depth_(10), num_threads_(omp_get_num_procs()), with_color_(false),with_init_compute_(true), color_(16.f), samples_per_node_(1.5f), scale_factor_(1.05f), cs_solver_accuracy_(1e-3), point_weight_(4.0),
smooth_iterations_(5), hole_scale_(0.001), aRatio_(0.001f), k_neighbor_(120), average_point_dist_(0.001), resolution_(0.001), sharpness_angle_(25.0),std_mul_(1.5),progress_bar_value_(0), kdtree_()
{

}

PoissonReconstruction::~PoissonReconstruction()
{

}
bool PoissonReconstruction::init()
{
	pcl::StopWatch sw;
	int point_num = static_cast<int>(input_->points.size());
	if (point_num <= 0)
	{
		std::cerr << "input point cloud is empty!\n";
		return false;
	}
	kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>(true));
	kdtree_->setInputCloud(input_);
	processed_input_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	double build_tree_time = sw.getTimeSeconds();
	progress_bar_value_ += 3;
	progress_bar_sig_(*this, progress_bar_value_);
	sw.reset();
	printf("\t build kd tree time: %fs \n", build_tree_time);
	return true;
}
void PoissonReconstruction::initCompute()
{
	pcl::StopWatch sw;
	int point_num = static_cast<int>(input_->points.size());

	kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>(true));
	kdtree_->setInputCloud(input_);
	processed_input_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pcl::IndicesPtr indices(new std::vector<int>);
	indices->resize(point_num);
	std::vector<int> nn_indices;
	std::vector<float> nn_dists;
	double build_tree_time = sw.getTimeSeconds();
	std::cout << "build kd tree time: " << build_tree_time << std::endl;
	sw.reset();
	std::vector<std::vector<int> > pt_neighbor_indices(point_num);
	std::vector<float> distances(point_num);
	// compute k neighbor mean dist for every point
	for (int i = 0; i < point_num; ++i)
	{
		//the first nearest k point is the i, so we search k_neighbor + 1 points
		kdtree_->nearestKSearch(i, k_neighbor_ + 1, nn_indices, nn_dists);
		double dist_sum = 0.0;
		for (int k = 1; k < k_neighbor_ + 1; ++k)
			dist_sum += std::sqrt(nn_dists[k]);
		distances[i] = static_cast<float>(dist_sum / k_neighbor_);
		pt_neighbor_indices[i] = nn_indices;
	}
	double neighbor_k_search_time = sw.getTimeSeconds();
	sw.reset();
	std::cout << "k neighbor search time: " << neighbor_k_search_time << std::endl;
	std::cout << "average point dist: " << average_point_dist_ << std::endl;
	// compute distance threshold for each point and classify
	int oii = 0;
	for (int i = 0; i < point_num; ++i)
	{
		// estimate the mean and standard deviation of the neighbor distance vector
		double sum = 0, sq_sum = 0;
		for (size_t k = 1; k < pt_neighbor_indices[i].size(); ++k)
		{
			sum += distances[pt_neighbor_indices[i][k]];
			sq_sum += distances[pt_neighbor_indices[i][k]] * distances[pt_neighbor_indices[i][k]];
		}
		double mean = sum / static_cast<double>(k_neighbor_);
		double variance = (sq_sum - sum * sum / static_cast<double>(k_neighbor_)) / (static_cast<double>(k_neighbor_)-1);
		double stddev = std::sqrt(variance);

		double distance_threshold = mean + std_mul_ * stddev;
		if (distances[i] < distance_threshold)
		{
			(*indices)[oii++] = i;
		}
	}
	indices->resize(oii);
	double filter_time = sw.getTimeSeconds();
	std::cout << "statistic removal time: " << filter_time << std::endl;
	std::cout << "filter points num: " << indices->size()<< std::endl;

	kdtree_->setInputCloud(input_, indices);
	std::vector<std::vector<int> > neighbor_indices(indices->size());
	std::vector<std::vector<float> > neighbor_dists(indices->size());
	// bilateral smoothing
	sw.reset();
	float guess_neighbor_radius = 0.0f;
	for (size_t i = 0; i < indices->size(); ++i)
	{
		kdtree_->nearestKSearch(i, k_neighbor_, nn_indices, nn_dists);
		neighbor_indices[i] = nn_indices;
		neighbor_dists[i] = nn_dists;
		float max_spacing = std::sqrt(nn_dists.back());
		guess_neighbor_radius = std::max<float>(max_spacing, guess_neighbor_radius);
	}
	guess_neighbor_radius = 0.95 * guess_neighbor_radius;
	double bs_k_neighbor_search_time = sw.getTimeSeconds();
	sw.reset();
	std::cout << "bs k neighbor search time: " << bs_k_neighbor_search_time << std::endl;
	std::cout << "guess neighbor radius:" << guess_neighbor_radius << std::endl;
	float radius2 = guess_neighbor_radius * guess_neighbor_radius;
	float iradius16 = -4.0f / radius2;
	float sharpness_angle = sharpness_angle_;
	float cos_sigma = std::cos(sharpness_angle / 180.0 * 3.141592653);
	float sharpness_bandwith = std::pow(std::max<float>(1e-8, 1 - cos_sigma), 2);
	processed_input_->points.resize(indices->size());
	for (size_t i = 0; i < indices->size(); ++i)
	{
		float weight = 0.f;
		float project_dist_sum = 0.f;
		float project_weight_sum = 0.f;
		Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();

		Eigen::Vector3f query_point = input_->points[(*indices)[i]].getVector3fMap();
		Eigen::Vector3f query_normal = input_->points[(*indices)[i]].getNormalVector3fMap();

		for (int k = 1; k < neighbor_indices[i].size(); ++k)
		{
			float dist2 = neighbor_dists[i][k];

			const Eigen::Vector3f& np = input_->points[neighbor_indices[i][k]].getVector3fMap();
			const Eigen::Vector3f& nn = input_->points[neighbor_indices[i][k]].getNormalVector3fMap();
			if (dist2 < radius2)
			{
				float theta = std::exp(dist2 * iradius16);
				float psi = std::exp(-std::pow(1 - query_normal.dot(nn), 2) / sharpness_bandwith);
				weight = theta * psi;
				project_dist_sum += ((query_point - np).dot(nn)) * weight;
				project_weight_sum += weight;
				normal_sum = normal_sum + nn * weight;
			}
		}

		Eigen::Vector3f update_normal = normal_sum / project_weight_sum;
		update_normal.normalize();

		Eigen::Vector3f update_point = query_point - update_normal * (project_dist_sum / project_weight_sum);

		processed_input_->points[i].x = update_point(0);
		processed_input_->points[i].y = update_point(1);
		processed_input_->points[i].z = update_point(2);
		processed_input_->points[i].normal_x = update_normal(0);
		processed_input_->points[i].normal_y = update_normal(1);
		processed_input_->points[i].normal_z = update_normal(2);
	}
	double bs_smooth_time = sw.getTimeSeconds();
	std::cout << "bs smooth time: " << bs_smooth_time << std::endl;
//	dvo::savePointXYZNormal("D:/testData/chair_bs.asc", *processed_input_);
	// compute scale
	Eigen::Array3f min_p, max_p;
	min_p.setConstant(FLT_MAX);
	max_p.setConstant(-FLT_MAX);

	for (const auto& point : processed_input_->points)
	{
		pcl::Array3fMapConst p = point.getArray3fMap();
		for (int i = 0; i < 3; ++i)
		{
			if (p[i] < min_p[i]) min_p[i] = p[i];
			if (p[i] > max_p[i]) max_p[i] = p[i];
		}
	}
    scale_ = std::max<float>(max_p[0] - min_p[0], std::max<float>(max_p[1] - min_p[1], max_p[2] - min_p[1]));
	center_ = (min_p + max_p) / 2;
	std::cout << "scale: " << scale_ << std::endl;
	std::cout << "center: " << center_[0] << " " << center_[1] << " " << center_[2] << std::endl;
	depth_ = static_cast<int>(-std::ceil(std::log2f(resolution_ / scale_)));
	// compute scale factor
	//	scale_factor_ = scale / (std::pow(2.0, depth_) * resolution_);
	//	std::cout << "scale factor: " << scale_factor_ << std::endl;
	std::cout << "octree depth: " << depth_ << std::endl;
}

void PoissonReconstruction::initComputeTbb()
{
	pcl::StopWatch sw;
	int point_num = static_cast<int>(input_->points.size());
	std::vector<int> nn_indices;
	std::vector<float> nn_dists;
	std::vector<std::vector<int> > pt_neighbor_indices(point_num);
	std::vector<std::vector<float> > pt_neighbor_dists(point_num);
	std::vector<float> distances(point_num);
	float guess_sqr_neighbor_radius = 0.0f;
	Compute_k_neighbors f(k_neighbor_, kdtree_, pt_neighbor_indices, pt_neighbor_dists,distances);
	tbb::parallel_for(tbb::blocked_range<size_t>(0, point_num), f);

//	std::cout << "compute k neighbors time: " << sw.getTimeSeconds() << std::endl;
	double compute_k_neighbor_time = sw.getTimeSeconds();
	progress_bar_value_ += 15;
	progress_bar_sig_(*this, progress_bar_value_);
	sw.reset();
	std::vector<bool> indices_flag(point_num, false);
	Outlier_removal f1(k_neighbor_, std_mul_, pt_neighbor_indices, pt_neighbor_dists, distances, indices_flag);
	tbb::parallel_for(tbb::blocked_range<size_t>(0, point_num), f1);

//	std::cout << "outlier removal time: " << sw.getTimeSeconds() << std::endl;
	double remove_outlier_time = sw.getTimeSeconds();
	progress_bar_value_ += 2;
	progress_bar_sig_(*this, progress_bar_value_);
	sw.reset();
//	float average_point_dist_sum = 0.0;
	int filter_point_num = 0;
	for (int i = 0; i < point_num; ++i)
	{
		if (indices_flag[i])
		{
			guess_sqr_neighbor_radius = std::max<float>(guess_sqr_neighbor_radius, pt_neighbor_dists[i].back());
//			average_point_dist_sum += pt_neighbor_dists[i][1];
			filter_point_num++;
		}

	}
//	average_point_dist_ = std::sqrt(average_point_dist_sum / filter_point_num);
//	std::cout << "guess neighbor radius time: " << sw.getTimeSeconds() << std::endl;
	std::cout << "guess neighbor radius: " << std::sqrt(guess_sqr_neighbor_radius) << std::endl;
	std::cout << "average point dist: " << average_point_dist_ << std::endl;
	sw.reset();
	processed_input_->reserve(point_num);
	float radius2 = 0.95  * guess_sqr_neighbor_radius;
	float iradius16 = -4.0f / radius2;
	float sharpness_angle = sharpness_angle_;
	float cos_sigma = std::cos(sharpness_angle / 180.0 * 3.141592653);
	float sharpness_bandwith = std::pow(std::max<float>(1e-8, 1 - cos_sigma), 2);

	Bilateral_smooth f2(radius2, sharpness_bandwith, pt_neighbor_indices, pt_neighbor_dists, indices_flag, input_, processed_input_);
	tbb::parallel_for(tbb::blocked_range<size_t>(0, point_num), f2);

//	std::cout << "bilateral smooth time: " << sw.getTimeSeconds() << std::endl;
	double bs_time = sw.getTimeSeconds();
	progress_bar_value_ += 10;
	progress_bar_sig_(*this, progress_bar_value_);
	std::cout << "point num after filtering: " << processed_input_->size() << std::endl;
	//Eigen::Array3f min_p, max_p;
	//min_p.setConstant(FLT_MAX);
	//max_p.setConstant(-FLT_MAX);

	//for (const auto& point : processed_input_->points)
	//{
	//	pcl::Array3fMapConst p = point.getArray3fMap();
	//	for (int i = 0; i < 3; ++i)
	//	{
	//		if (p[i] < min_p[i]) min_p[i] = p[i];
	//		if (p[i] > max_p[i]) max_p[i] = p[i];
	//	}
	//}
	//scale_ = std::max<float>(max_p[0] - min_p[0], std::max<float>(max_p[1] - min_p[1], max_p[2] - min_p[1]));
	//center_ = (min_p + max_p) / 2;
	//std::cout << "scale: " << scale_ << std::endl;
	////std::cout << "center: " << center_[0] << " " << center_[1] << " " << center_[2] << std::endl;
	//depth_ = static_cast<int>(-std::ceil(std::log2f(resolution_ / scale_)));
	// compute scale factor
	//	scale_factor_ = scale / (std::pow(2.0, depth_) * resolution_);
	//	std::cout << "scale factor: " << scale_factor_ << std::endl;
	std::cout << "octree depth: " << depth_ << std::endl;
	printf("\t compute k neighbor time: %fs \n", compute_k_neighbor_time);
	printf("\t outlier removal time: %fs \n", remove_outlier_time);
	printf("\t bilateral smooth time: %fs \n", bs_time);
}


void PoissonReconstruction::initComputeOpenMP()
{
	pcl::StopWatch sw;
	int point_num = static_cast<int>(input_->points.size());

	//point's k neighbor mean distance
	std::vector<float> k_mean_dists(point_num);
	//point's max k neighbor distance
	std::vector<float> k_max_dists(point_num);
	int k_neighbor_1 = k_neighbor_ + 1;
	int k_neighbor_2 = k_neighbor_ - 1;
#pragma  omp parallel for schedule(dynamic,1)
	for (int i = 0; i < point_num; ++i)
	{
		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		//the first nearest k point is the i, so we search k_neighbor + 1 points
		kdtree_->nearestKSearch(i, k_neighbor_1, nn_indices, nn_dists);
		float dist_sum = 0.f;
		for (int k = 1; k < k_neighbor_1; ++k)
			dist_sum += nn_dists[k];
		k_mean_dists[i] = std::sqrt(dist_sum / (float)k_neighbor_);
		k_max_dists[i] = nn_dists[k_neighbor_2];
	}
	progress_bar_value_ += 7;
	progress_bar_sig_(*this, progress_bar_value_);

	// compute distance threshold for each point and classify
	std::vector<bool> indices_flag(point_num, false); // if the points are reserved, flag set true;
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < point_num; ++i)
	{
		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		kdtree_->nearestKSearch(i, k_neighbor_1, nn_indices, nn_dists);
		// estimate the mean and standard deviation of the neighbor distance vector
		double sum = 0, sq_sum = 0;
		for (int k = 1; k < k_neighbor_1; ++k)
		{
			float dist = k_mean_dists[nn_indices[k]];
			sum += dist;
			sq_sum += dist * dist;
		}
		double mean = sum / static_cast<double>(k_neighbor_);
		double variance = (sq_sum - sum * sum / static_cast<double>(k_neighbor_)) / (static_cast<double>(k_neighbor_)-1);
		double stddev = std::sqrt(variance);

		double distance_threshold = mean + std_mul_ * stddev;
		if (k_mean_dists[i] < distance_threshold)
		{
			indices_flag[i] = true;
		}
	}
	progress_bar_value_ += 10;
	progress_bar_sig_(*this, progress_bar_value_);
	double t_filter = sw.getTimeSeconds();
	sw.reset();

	float guess_neighbor_radius = 0.f;
	int filter_point_num = 0;
	for (int i = 0; i < point_num; ++i)
	{
		if (indices_flag[i])
		{
			guess_neighbor_radius = std::max<float>(k_max_dists[i], guess_neighbor_radius);
			filter_point_num++;
		}
	}
	float radius2 = 0.95 * guess_neighbor_radius;
	float iradius16 = -4.0f / radius2;
	float sharpness_angle = sharpness_angle_;
	float cos_sigma = std::cos(sharpness_angle / 180.0 * 3.141592653);
	float sharpness_bandwith = std::pow(std::max<float>(1e-8, 1 - cos_sigma), 2);

	processed_input_->points.resize(filter_point_num);
	int oii = 0;
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < point_num; ++i)
	{
		if (indices_flag[i])
		{
			std::vector<int> nn_indices;
			std::vector<float> nn_dists;
			kdtree_->nearestKSearch(i, k_neighbor_1, nn_indices, nn_dists);

			float weight = 0.f;
			float project_dist_sum = 0.f;
			float project_weight_sum = 0.f;
			Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();

			pcl::PointXYZRGBNormal* pni = &(input_->points[i]);
			Eigen::Vector3f query_point = pni->getVector3fMap();
			Eigen::Vector3f query_normal = pni->getNormalVector3fMap();

			for (int k = 1; k < k_neighbor_1; ++k)
			{
				if (indices_flag[nn_indices[k]])
				{
					float dist2 = nn_dists[k];
					pcl::PointXYZRGBNormal* pnk = &(input_->points[nn_indices[k]]);
					const Eigen::Vector3f& np = pnk->getVector3fMap();
					const Eigen::Vector3f& nn = pnk->getNormalVector3fMap();
					if (dist2 < radius2)
					{
						float theta = std::exp(dist2 * iradius16);
						float psi = std::exp(-std::pow(1 - query_normal.dot(nn), 2) / sharpness_bandwith);
						//float theta = exp_linear(dist2 * iradius16);
						//float psi = exp_linear(-std::pow(1 - query_normal.dot(nn), 2) / sharpness_bandwith);
						weight = theta * psi;
						project_dist_sum += ((query_point - np).dot(nn)) * weight;
						project_weight_sum += weight;
						normal_sum = normal_sum + nn * weight;
					}
				}
			}
			Eigen::Vector3f update_normal = normal_sum / project_weight_sum;
			update_normal.normalize();

			Eigen::Vector3f update_point = query_point - update_normal * (project_dist_sum / project_weight_sum);
			pcl::PointXYZRGBNormal* pn = &(processed_input_->points[oii++]);
			pn->x = update_point(0);
			pn->y = update_point(1);
			pn->z = update_point(2);
			pn->normal_x = update_normal(0);
			pn->normal_y = update_normal(1);
			pn->normal_z = update_normal(2);
		}
	}
	progress_bar_value_ += 10;
	progress_bar_sig_(*this, progress_bar_value_);
	double t_bs_smooth = sw.getTimeSeconds();

	std::cout << "point num after filtering: " << processed_input_->size() << std::endl;
	std::cout << "guess neighbor radius: " << std::sqrt(guess_neighbor_radius) << std::endl;
	printf("\t outlier removal time: %fs \n", t_filter);
	printf("\t bilateral smooth time: %fs \n", t_bs_smooth);
}

bool PoissonReconstruction::reconstruct(pcl::PolygonMesh &output)
{
	size_t max_point_num = 10000000;
	pcl::StopWatch sw;
	// initialization:
	if (!init())
	{
		std::cout << "init failure\n";
		return false;
	}
	if (with_init_compute_)
	{
		if (input_->size() > max_point_num)
			initComputeOpenMP();
		else
			initComputeTbb();
//		initComputeOpenMP();
	}
	else
	{
		progress_bar_value_ += 27;
		progress_bar_sig_(*this, progress_bar_value_);
		processed_input_ = input_;
	}
	double time = sw.getTimeSeconds();
//	std::cout << "init compute time: " << time << std::endl;
	printf("\t init compute total time: %fs\n", time);
	sw.reset();
	if (with_color_)
		if (use_double_)
			execute<double, PlyColorAndValueVertex<float> >(output);
		else
			execute<float, PlyColorAndValueVertex<float> >(output);
	else
		if (use_double_)
			execute<double, PlyValueVertex<float> >(output);
		else
			execute<float, PlyValueVertex<float> >(output);
//	std::cout << "poisson reconstruction time: " << sw.getTimeSeconds() << std::endl;
	printf("\t poisson reconstruction total time: %fs \n", sw.getTimeSeconds());
	return true;
}

template<class Real, class Vertex>
int PoissonReconstruction::execute(pcl::PolygonMesh &output)
{
	switch (degree_)
	{
	case 1:
		return execute_<Real, 1, Vertex>(output);
	case 2:
		return execute_<Real, 2, Vertex>(output);
	case 3:
		return execute_<Real, 3, Vertex>(output);
	case 4:
		return execute_<Real, 4, Vertex>(output);
	default:
		break;
	}
	return 0;
}


template<class Real, int Degree, class Vertex>
int PoissonReconstruction::execute_(pcl::PolygonMesh &output)
{
	Reset< Real >();


	XForm4x4< Real > xForm, iXForm;
	xForm = XForm4x4< Real >::Identity();
	iXForm = xForm.inverse();

	Real isoValue = 0;
	std::cout << "num threads: " << num_threads_ << std::endl;
	Octree< Real > tree;
	tree.threads = num_threads_;
//	tree.threads = omp_get_num_procs();
	max_solve_depth_ = depth_;
	//tree.setScale(scale_);
	//tree.setCenter(center_);

	OctNode< TreeNodeData >::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);

	int kernelDepth = depth_ - 2;

	double maxMemoryUsage;
	tree.maxMemoryUsage = 0;
	SparseNodeData< PointData< Real >, 0 >* pointInfo = new SparseNodeData< PointData< Real >, 0 >();
	SparseNodeData< Point3D< Real >, NORMAL_DEGREE >* normalInfo = new SparseNodeData< Point3D< Real >, NORMAL_DEGREE >();
	SparseNodeData< Real, WEIGHT_DEGREE >* densityWeights = new SparseNodeData< Real, WEIGHT_DEGREE >();
	SparseNodeData< Real, NORMAL_DEGREE >* nodeWeights = new SparseNodeData< Real, NORMAL_DEGREE >();
	int pointCount;
	typedef  ProjectiveData< Point3D< Real > > ProjectiveColor;
	SparseNodeData< ProjectiveColor, DATA_DEGREE >* colorData = NULL;

	pcl::StopWatch sw;
	std::cout << "set tree...\n";
	if (with_color_)
	{
		colorData = new SparseNodeData< ProjectiveColor, DATA_DEGREE >();
		OrientedPointStreamWithData< float, Point3D< unsigned char > >* pointStream = new CloudOrientedPointStreamWithData<float, Point3D<unsigned char> >(processed_input_->size(), processed_input_);
		pointCount = tree.template SetTree< float, NORMAL_DEGREE, WEIGHT_DEGREE, DATA_DEGREE, Point3D< unsigned char > >(pointStream, min_depth_, depth_, full_depth_, kernelDepth, samples_per_node_, scale_factor_, use_confidence_, use_normal_weights_, true, adaptive_exponent_, *densityWeights, *pointInfo, *normalInfo, *nodeWeights, colorData, xForm, use_dirichlet_, make_complete_);
		delete pointStream;

		for (const OctNode< TreeNodeData >* n = tree.tree().nextNode(); n != NULL; n = tree.tree().nextNode(n))
		{
			int idx = colorData->index(n);
			if (idx >= 0) colorData->data[idx] *= (Real)pow(color_, n->depth());
		}
	}
	else
	{
		OrientedPointStream< float >* pointStream = new CloudOrientedPointStream<float>(processed_input_->size(), processed_input_);
		pointCount = tree.template SetTree< float, NORMAL_DEGREE, WEIGHT_DEGREE, DATA_DEGREE, Point3D< unsigned char > >(pointStream, min_depth_, depth_, full_depth_, kernelDepth, samples_per_node_, scale_factor_, use_confidence_, use_normal_weights_, true, adaptive_exponent_, *densityWeights, *pointInfo, *normalInfo, *nodeWeights, colorData, xForm, use_dirichlet_, make_complete_);
		delete pointStream;
	}
	double set_tree_time = sw.getTimeSeconds();
	progress_bar_value_ += 15;
	progress_bar_sig_(*this, progress_bar_value_);
	sw.reset();
	printf("slove system...\n");
	sw.reset();
	{
		std::vector< int > indexMap;
		if (NORMAL_DEGREE > Degree) tree.template EnableMultigrid< NORMAL_DEGREE >(&indexMap);
		else                       tree.template EnableMultigrid<        Degree >(&indexMap);
		if (pointInfo) pointInfo->remapIndices(indexMap);
		if (normalInfo) normalInfo->remapIndices(indexMap);
		if (densityWeights) densityWeights->remapIndices(indexMap);
		if (nodeWeights) nodeWeights->remapIndices(indexMap);
		if (colorData) colorData->remapIndices(indexMap);
	}


	maxMemoryUsage = tree.maxMemoryUsage;
	tree.maxMemoryUsage = 0;
	DenseNodeData< Real, Degree > constraints = tree.template SetLaplacianConstraints< Degree >(*normalInfo);
	delete normalInfo;
	maxMemoryUsage = std::max< double >(maxMemoryUsage, tree.maxMemoryUsage);

	tree.maxMemoryUsage = 0;
	DenseNodeData< Real, Degree > solution = tree.SolveSystem(*pointInfo, constraints, false, iters_, max_solve_depth_, cg_depth_, cs_solver_accuracy_);
	delete pointInfo;
	constraints.resize(0);
	double slove_system_time = sw.getTimeSeconds();
	progress_bar_value_ += 35;
	progress_bar_sig_(*this, progress_bar_value_);
	sw.reset();
	maxMemoryUsage = std::max< double >(maxMemoryUsage, tree.maxMemoryUsage);

	//	CoredFileMeshData< Vertex > mesh;
	CoredVectorMeshData<Vertex> mesh;
	isoValue = tree.GetIsoValue(solution, *nodeWeights);
	delete nodeWeights;
	std::cout << "get mc iso surface...\n";
	tree.template GetMCIsoSurface< Degree, WEIGHT_DEGREE, DATA_DEGREE >(densityWeights, colorData, solution, isoValue, mesh, !use_linear_fit_, !use_nonManifold_, use_polygon_mesh_);
	printf("Vertices / Polygons: %d / %d\n", mesh.outOfCorePointCount() + mesh.inCorePoints.size(), mesh.polygonCount());
	double get_iso_surface_time = sw.getTimeSeconds();
	progress_bar_value_ += 15;
	progress_bar_sig_(*this, progress_bar_value_);
	sw.reset();
	//	PlyWritePolygons("D:/testData/poisson.ply", &mesh, PLY_ASCII, NULL, 0, iXForm);
	solution.resize(0);
	if (colorData){ delete colorData; colorData = NULL; }

	// surface trimming
	std::vector<Vertex> vertices;
	std::vector<std::vector<int> > polygons;
	//	trimSurface(mesh, vertices, polygons);
	vertices.resize(mesh.inCorePoints.size() + mesh.outOfCorePointCount());
	//int num = 0;
	//for (size_t i = 0; i < mesh.inCorePoints.size(); ++i)
	//{
	//	vertices[num++] = mesh.inCorePoints[i];
	//}
	//for (size_t i = 0; i < mesh.outOfCorePointCount(); ++i)
	//{
	//	vertices[num++] = mesh.oocPoints[i];
	//}
	Vertex* pVertices = &(vertices[0]);
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < (int)mesh.inCorePoints.size(); ++i)
	{
		pVertices[i] = mesh.inCorePoints[i];
	}
	pVertices = &(vertices[mesh.inCorePoints.size()]);
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < (int)mesh.outOfCorePointCount(); ++i)
	{
		pVertices[i] = mesh.oocPoints[i];
	}
	polygons.resize(mesh.polygonCount());

	int mesh_inCorePoints_size_1 = mesh.inCorePoints.size() - 1;
//	std::vector<int> polygonv;
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < (int)mesh.polygonCount(); ++i)
	{
		std::vector<int> &polygon = mesh.polygons[i];
		std::vector<int> polygonv(polygon.size());
		for (size_t j = 0; j < polygon.size(); ++j)
		{
			if (polygon[j] >= 0)
				polygonv[j] = polygon[j];
			else
				polygonv[j] = -polygon[j] + mesh_inCorePoints_size_1;
		}
		polygons[i] = polygonv;
	}


//	kdtree_->setInputCloud(processed_input_);
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; (int)i < vertices.size(); ++i)
	{
		pcl::PointXYZRGBNormal point;
		point.x = vertices[i].point.coords[0];
		point.y = vertices[i].point.coords[1];
		point.z = vertices[i].point.coords[2];

		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		kdtree_->nearestKSearch(point, 1, nn_indices, nn_dists);
		vertices[i].value = nn_dists[0];
	}
	for (int i = 0; i < smooth_iterations_; i++)
		SmoothPoints< float, Vertex >(vertices, polygons);
	std::unordered_map< long long, int > vertexTable;
	std::vector< std::vector< int > > ltPolygons, gtPolygons;
	std::vector< bool > ltFlags, gtFlags;

	for (size_t i = 0; i < polygons.size(); i++) SplitPolygon(polygons[i], vertices, &ltPolygons, &gtPolygons, &ltFlags, &gtFlags, vertexTable, hole_scale_, average_point_dist_);
	if (aRatio_ > 0)
	{
		std::vector< std::vector< int > > _ltPolygons, _gtPolygons;
		std::vector< std::vector< int > > ltComponents, gtComponents;
		SetConnectedComponents(ltPolygons, ltComponents);
		SetConnectedComponents(gtPolygons, gtComponents);
		std::vector< double > ltAreas(ltComponents.size(), 0.), gtAreas(gtComponents.size(), 0.);
		std::vector< bool > ltComponentFlags(ltComponents.size(), false), gtComponentFlags(gtComponents.size(), false);
		double area = 0.;
		for (size_t i = 0; i < ltComponents.size(); i++)
		{
			for (size_t j = 0; j < ltComponents[i].size(); j++)
			{
				ltAreas[i] += PolygonArea< float, Vertex >(vertices, ltPolygons[ltComponents[i][j]]);
				ltComponentFlags[i] = (ltComponentFlags[i] || ltFlags[ltComponents[i][j]]);
			}
			area += ltAreas[i];
		}
		for (size_t i = 0; i < gtComponents.size(); i++)
		{
			for (size_t j = 0; j < gtComponents[i].size(); j++)
			{
				gtAreas[i] += PolygonArea< float, Vertex >(vertices, gtPolygons[gtComponents[i][j]]);
				gtComponentFlags[i] = (gtComponentFlags[i] || gtFlags[gtComponents[i][j]]);
			}
			area += gtAreas[i];
		}
		for (size_t i = 0; i < ltComponents.size(); i++)
		{
			if (ltAreas[i] < area*aRatio_ && ltComponentFlags[i]) for (size_t j = 0; j < ltComponents[i].size(); j++) _gtPolygons.push_back(ltPolygons[ltComponents[i][j]]);
			else                                                               for (size_t j = 0; j < ltComponents[i].size(); j++) _ltPolygons.push_back(ltPolygons[ltComponents[i][j]]);
		}
		for (size_t i = 0; i < gtComponents.size(); i++)
		{
			if (gtAreas[i] < area*aRatio_ && gtComponentFlags[i]) for (size_t j = 0; j < gtComponents[i].size(); j++) _ltPolygons.push_back(gtPolygons[gtComponents[i][j]]);
			else                                                               for (size_t j = 0; j < gtComponents[i].size(); j++) _gtPolygons.push_back(gtPolygons[gtComponents[i][j]]);
		}
		ltPolygons = _ltPolygons, gtPolygons = _gtPolygons;
	}
	if (!use_polygon_mesh_)
	{
		{
			std::vector< std::vector< int > > polys = ltPolygons;
			Triangulate< float, Vertex >(vertices, ltPolygons, polys), ltPolygons = polys;
		}
		{
			std::vector< std::vector< int > > polys = gtPolygons;
			Triangulate< float, Vertex >(vertices, gtPolygons, polys), gtPolygons = polys;
		}
	}

	RemoveHangingVertices(vertices, gtPolygons);
	polygons.swap(gtPolygons);
	double trim_surface_time = sw.getTimeSeconds();
	printf("Final Vertices / Polygons: %d / %d\n", vertices.size(), polygons.size());
	printf("\t set tree time: %fs \n", set_tree_time);
	printf("\t slove system time: %fs \n", slove_system_time);
	printf("\t get iso surface time: %fs \n", get_iso_surface_time);
	printf("\t trim surface time: %fs \n", trim_surface_time);
	// Write output PolygonMesh
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.resize(vertices.size());
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < (int)cloud.size(); ++i)
	{
		const Point3D<float> &p = vertices[i].point;
		cloud.points[i].x = p.coords[0];
		cloud.points[i].y = p.coords[1];
		cloud.points[i].z = p.coords[2];
	}
	pcl::toPCLPointCloud2(cloud, output.cloud);
	output.polygons.resize(polygons.size());
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < (int)polygons.size(); ++i)
	{
		pcl::Vertices v;
		v.vertices.resize(polygons[i].size());
		for (size_t j = 0; j < polygons[i].size(); ++j)
		{
			v.vertices[j] = polygons[i][j];
		}

		output.polygons[i] = v;
	}
	progress_bar_value_ += 5;
	progress_bar_sig_(*this, progress_bar_value_);
	return 1;
}
/*
template<class Vertex>
int PoissonReconstruction::trimSurface(CoredVectorMeshData<Vertex> &mesh, std::vector<Vertex> &vertices, std::vector<std::vector<int> > &polygons)
{
vertices.resize(mesh.inCorePoints.size() + mesh.outOfCorePointCount());
int num = 0;
for (size_t i = 0; i < mesh.inCorePoints.size(); ++i)
{
vertices[num++] = mesh.inCorePoints[i];
}
for (size_t i = 0; i < mesh.outOfCorePointCount(); ++i)
{
vertices[num++] = mesh.oocPoints[i];
}
polygons.resize(mesh.polygonCount());

std::vector<int> polygonv;
for (size_t i = 0; i < mesh.polygonCount(); ++i)
{
std::vector<int> &polygon = mesh.polygons[i];
polygonv.resize(polygon.size());
for (size_t j = 0; j < polygon.size(); ++j)
{
if (polygon[j] >= 0)
polygonv[j] = polygon[j];
else
polygonv[j] = -polygon[j] - 1 + mesh.inCorePoints.size();
}
polygons[i] = polygonv;
}

float min, max;
for (int i = 0; i < smooth_iterations_; i++) SmoothValues< float, Vertex >(vertices, polygons);
min = max = vertices[0].value;
for (size_t i = 0; i < vertices.size(); i++) min = std::min< float >(min, vertices[i].value), max = std::max< float >(max, vertices[i].value);
printf("Value Range: [%f,%f]\n", min, max);


hash_map< long long, int > vertexTable;
std::vector< std::vector< int > > ltPolygons, gtPolygons;
std::vector< bool > ltFlags, gtFlags;

for (size_t i = 0; i < polygons.size(); i++) SplitPolygon(polygons[i], vertices, &ltPolygons, &gtPolygons, &ltFlags, &gtFlags, vertexTable, trim_);
if (aRatio_ > 0)
{
std::vector< std::vector< int > > _ltPolygons, _gtPolygons;
std::vector< std::vector< int > > ltComponents, gtComponents;
SetConnectedComponents(ltPolygons, ltComponents);
SetConnectedComponents(gtPolygons, gtComponents);
std::vector< double > ltAreas(ltComponents.size(), 0.), gtAreas(gtComponents.size(), 0.);
std::vector< bool > ltComponentFlags(ltComponents.size(), false), gtComponentFlags(gtComponents.size(), false);
double area = 0.;
for (size_t i = 0; i < ltComponents.size(); i++)
{
for (size_t j = 0; j < ltComponents[i].size(); j++)
{
ltAreas[i] += PolygonArea< float, Vertex >(vertices, ltPolygons[ltComponents[i][j]]);
ltComponentFlags[i] = (ltComponentFlags[i] || ltFlags[ltComponents[i][j]]);
}
area += ltAreas[i];
}
for (size_t i = 0; i < gtComponents.size(); i++)
{
for (size_t j = 0; j < gtComponents[i].size(); j++)
{
gtAreas[i] += PolygonArea< float, Vertex >(vertices, gtPolygons[gtComponents[i][j]]);
gtComponentFlags[i] = (gtComponentFlags[i] || gtFlags[gtComponents[i][j]]);
}
area += gtAreas[i];
}
for (size_t i = 0; i < ltComponents.size(); i++)
{
if (ltAreas[i] < area*aRatio_ && ltComponentFlags[i]) for (size_t j = 0; j < ltComponents[i].size(); j++) _gtPolygons.push_back(ltPolygons[ltComponents[i][j]]);
else                                                               for (size_t j = 0; j < ltComponents[i].size(); j++) _ltPolygons.push_back(ltPolygons[ltComponents[i][j]]);
}
for (size_t i = 0; i < gtComponents.size(); i++)
{
if (gtAreas[i] < area*aRatio_ && gtComponentFlags[i]) for (size_t j = 0; j < gtComponents[i].size(); j++) _ltPolygons.push_back(gtPolygons[gtComponents[i][j]]);
else                                                               for (size_t j = 0; j < gtComponents[i].size(); j++) _gtPolygons.push_back(gtPolygons[gtComponents[i][j]]);
}
ltPolygons = _ltPolygons, gtPolygons = _gtPolygons;
}
if (!use_polygon_mesh_)
{
{
std::vector< std::vector< int > > polys = ltPolygons;
Triangulate< float, Vertex >(vertices, ltPolygons, polys), ltPolygons = polys;
}
{
std::vector< std::vector< int > > polys = gtPolygons;
Triangulate< float, Vertex >(vertices, gtPolygons, polys), gtPolygons = polys;
}
}

RemoveHangingVertices(vertices, gtPolygons);

polygons.swap(gtPolygons);

return 0;
}
*/
