/*!
* \file poisson_reconstruction.h
* \date 2018/09/27 11:57
*
* \author meng zhili
* Contact: user@company.com
*
* \brief
*
* TODO: long description
*
* \note
*/

#ifndef POISSON_RECONSTRUCTION
#define  POISSON_RECONSTRUCTION

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/scalable_allocator.h>  

#include <boost/progress.hpp>
#include <boost/signals2.hpp>
#include <dvo/core/dvo_exports.h>
#define DEFAULT_FULL_DEPTH 5


class DVO_EXPORTS  PoissonReconstruction
{
public:
	PoissonReconstruction();
	~PoissonReconstruction();

	typedef boost::signals2::signal<void(const PoissonReconstruction&, const int &value)> ProgressBarSignal;
	typedef ProgressBarSignal::slot_type ProgressBarCallBack;

	boost::signals2::connection addProgressBarCallback(const ProgressBarCallBack& callback){ return progress_bar_sig_.connect(callback); }

	/* * \brief  set input point cloud*/
	void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud){ input_ = cloud; }

	/* * \brief set number of threads for openmp */
	void setThreads(int numThreads){ num_threads_ = std::min<int>(numThreads,num_threads_); }

	/* * \brief set with init compute if true remove oulier and smooth */
	void setWithInitCompute(bool withInitCompute){ with_init_compute_ = withInitCompute; }
	/* * \brief set b spline degree default 3 */
	void setDegree(int degree){ degree_ = degree; }

	/* * \brief set resolution the length of octree voxel */
	void setResolution(float resolution){ resolution_ = resolution; }
	/* * \brief set depth of octree */
	void setDepth(int depth){ depth_ = depth; }

	/* * \brief the weight that point interpolation constraints are given when defining the Poisson system */
	void setPointWeight(float pointWeight){ point_weight_ = pointWeight; }

	/** \brief Set the ratio between the diameter of the cube used for reconstruction and the diameter of the
	* samples' bounding cube.
	* \param[in] scale the given parameter value
	*/
	void setScaleFactor(float scaleFactor){ scale_factor_ = scaleFactor; }

	/* * \brief the minimum number of points that should fall within an octree node
	     for noise-free sample, small values in the range[1.0-5.0] can be used, for more
		 noisy samples, larger values in the range[15.0-20.0] may be used
	*/
	void setSamplesPerNode(float samplesPerNode){ samples_per_node_ = samplesPerNode; }

	/* * \brief if true,the color will be output */
	void setColor(bool withColor){ with_color_ = withColor; }

	/* * \brief the number of umbrella smoothing operations to perform on the signal before trimming  */
	void setSmoothIterations(int smoothIters){ smooth_iterations_ = smoothIters; }

	/* * \brief the hole scale to be filled */
	void setHoleFillScale(float holeScale){ hole_scale_ = holeScale; }

	/* * \brief specify the area ratio that defines a disconnected component as an "island", connected components whose area, relative to the total area of the mesh,are
	smaller than this value will be merged into the output surface to close small holes, and will be discarded from the output surface to remove small disconnected components */
	void setIslandAreaRatio(float aRatio){ aRatio_ = aRatio; }

	/* * \brief Set the standard deviation multiplier for the distance threshold calculation.
	* details The distance threshold will be equal to: mean + stddev_mult * stddev.
	*/
	void setStddevMulThresh(float mulThresh){ std_mul_ = mulThresh; }
	/* * \brief the k nearest neighbor points number to calculate the average distance of point */
	void setKNeighbor(int k){ k_neighbor_ = k; }

	/* * \brief sharpness angle for bilateral smoothing */
	void setSharpnessAngle(float angle){ sharpness_angle_ = angle; }


public:
	/* * \brief reconstruct the input data and store the results into polygon mesh
	* \param[out] output he resultant polygon mesh
	*/
	bool reconstruct(pcl::PolygonMesh &output);
private:
	bool init();
	/* * \brief remove outliers and bilateral smooth the input data */
	void initCompute();
	/* * \brief remove outliers and bilateral smooth the input data */
	void initComputeTbb();
	/* * \brief remove outliers and bilateral smooth the input data */
	void initComputeOpenMP();
private:
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr processed_input_;
	pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree_;
private:
	bool use_confidence_;         // if this flag is enabled, the size of a sample's normals is used
	// as a confidence value, affecting the sample's contribution to the reconstruction process
	bool use_normal_weights_;    // if this flag is enabled, the size of a sample's normals is used as to modulate the interpolation weight
	bool use_nonManifold_;
	bool use_dirichlet_;    // if this flag is enabled, Dirichlet boundary constraints are used for reconstruction
	bool use_density_;      // if this flag is enabled, the sampling density is written out with the vertices
	bool use_linear_fit_;   // if this flag is enabled, the iso-surfacing will be performed using linear fitting
	bool use_primal_voxel_; // if this flag is enabled, voxel sampling is performed at corners rather than centers
	bool use_polygon_mesh_; // if this flag is enabled, the isosurface extraction returns polygons
	bool use_double_;
	bool with_color_;
	bool with_init_compute_;
	bool make_complete_;

	int degree_;
	int depth_;          // maximum reconstruction depth default 10
	int cg_depth_;       // the depth up to which a conjugate-gradients solver should be used
	int kernel_depth_;
	int adaptive_exponent_;
	int iters_;              // this flag specifies the number of solver iterations
	int voxel_depth_;       // depth at which to extract the voxel grid
	int full_depth_;       // this flag specifies the depth up to which the octree should be complete.
	int min_depth_;
	int max_solve_depth_;
	int num_threads_;        // this parameter specifies the number of threads across which the solver should be parallelized;

	float resolution_;        // absolute resolution
	float color_;              // this flag specifies the pull factor for color interpolation
	float samples_per_node_;    // this parameter specifies the minimum number of points that should fall within an octree node
	float scale_factor_;              // specifies the factor of the bounding cube that input
	float cs_solver_accuracy_;
	float point_weight_;       // this value specifies the weight that point interpolation constraints are given when defining the Poisson system

	int smooth_iterations_; // this integer values the number of umbrella smoothing operations to perform on the signal before trimming
	float trim_;          // this floating point values specifies the value for mesh trimming. the subset of the mesh with signal value less than the trim value is discarded
	float hole_scale_;
	float aRatio_;  // specify the area ratio that defines a disconnected component as an "island", connected components whose area, relative to the total area of the mesh, are
	// smaller than this value will be merged into the output surface to close small holes, and will be discarded from the output surface to remove small disconnected components

	int k_neighbor_;
	float sharpness_angle_;
	float std_mul_;
private:
	float average_point_dist_;
	float scale_;
	ProgressBarSignal progress_bar_sig_;
	int progress_bar_value_;
	Eigen::Array3f center_;
	template<class Real, int Degree, class Vertex> int
		execute_(pcl::PolygonMesh &output);

	template<class Real, class Vertex> int
		execute(pcl::PolygonMesh &output);
	//template<class Vertex> int
	//	trimSurface(CoredVectorMeshData<Vertex> &mesh, std::vector<Vertex> &vertices, std::vector<std::vector<int> > &polygons);
};

#endif