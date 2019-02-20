#ifndef  DVO_DATATYPES_H
#define  DVO_DATATYPES_H
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

typedef float IntensityType;
static const IntensityType Invalid = std::numeric_limits<IntensityType>::quiet_NaN();

typedef float DepthType;

static const DepthType InvalidDepth = std::numeric_limits<DepthType>::quiet_NaN();

typedef float NumType;

typedef Eigen::Matrix<NumType, 6, 6> Matrix6x6;
typedef Eigen::Matrix<NumType, 1, 2> Matrix1x2;
typedef Eigen::Matrix<NumType, 2, 6> Matrix2x6;

typedef Eigen::Matrix<NumType, 6, 1> Vector6;
typedef Eigen::Matrix<NumType, 4, 1> Vector4;

typedef Eigen::Transform<NumType, 3, Eigen::Affine> AffineTransform;

typedef Eigen::Affine3d AffineTransformd;
typedef Eigen::Affine3f AffineTransformf;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

typedef Eigen::Matrix<float, 8, 1> Vector8f;


typedef Eigen::Map<Eigen::Vector2f> Vector2fMap;
typedef const Eigen::Map<const Eigen::Vector2f> Vector2fMapConst;
typedef Eigen::Map<Vector8f> Vector8fMap;
typedef const Eigen::Map<const Vector8f> Vector8fMapConst;

//////////////////////////////////////////////////////////////////////////
struct EIGEN_ALIGN16 IntensityAndDepth
{
	typedef std::vector<IntensityAndDepth, Eigen::aligned_allocator<IntensityAndDepth> > VectorType;
	union
	{
		struct  
		{
			float i, d, idx, idy, zdx, zdy, curvature;
		};
		float data_g[8];
	};
	IntensityAndDepth()
	{
		this->i = this->d = std::numeric_limits<float>::quiet_NaN();;
		this->idx = this->idy = this->zdx = this->zdy = this->curvature = std::numeric_limits<float>::quiet_NaN();
		data_g[7] = 0.f;
	}
	IntensityAndDepth(const IntensityAndDepth &other)
	{
		this->i = other.i;
		this->d = other.d;
		this->idx = other.idx;
		this->idy = other.idy;
		this->zdx = other.zdx;
		this->zdy = other.zdy;
		this->curvature = other.curvature;
		this->data_g[7] = other.data_g[7];
	}
	inline Vector2fMap getIntensityDerivativeVec2f(){ return Vector2fMap(data_g + 2); }
	inline Vector2fMapConst getIntensityDerivativeVec2f() const { return Vector2fMapConst(data_g + 2); }

	inline Vector2fMap getDepthDerivativeVec2f(){ return Vector2fMap(data_g + 4); }
	inline Vector2fMapConst getDepthDerivativeVec2f() const { return Vector2fMapConst(data_g + 4); }

	inline Vector2fMap getIntensityAndDepthVec2f(){ return Vector2fMap(data_g); }
	inline Vector2fMapConst getIntensityAndDepthVec2f() const{ return Vector2fMapConst(data_g); }

	inline Vector8fMap getIntensityAndDepthWithDerivativesVec8f(){ return Vector8fMap(data_g); }
	inline Vector8fMapConst getIntensityAndDepthWithDerivativesVec8f() const { return Vector8fMapConst(data_g); }
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef std::vector<IntensityAndDepth, Eigen::aligned_allocator<IntensityAndDepth> > IntensityAndDepthVector;

//////////////////////////////////////////////////////////////////////////
struct EIGEN_ALIGN16 _PointXYZING
{
	PCL_ADD_POINT4D;
	PCL_ADD_NORMAL4D;
	union
	{
		struct
		{
			float gradient_x;
			float gradient_y;
			float gradient_z;
			float intensity;
		};
		float data_g[4];
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointXYZING :public _PointXYZING
{
	inline PointXYZING()
	{
		this->x = this->y = this->z = 0.f;
		this->data[3] = 1.f;

		this->normal_x = this->normal_y = this->normal_z = 0.f;
		this->data_n[3] = 0.f;

		this->intensity = 0.f;
		this->gradient_x = this->gradient_y = this->gradient_z = 0.f;
	}

	inline PointXYZING(const PointXYZING& other)
	{
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		this->data[3] = other.data[3];

		this->normal_x = other.normal_x;
		this->normal_y = other.normal_y;
		this->normal_z = other.normal_z;
		this->data_n[3] = other.data_n[3];
		this->intensity = other.intensity;
		this->gradient_x = other.gradient_x;
		this->gradient_y = other.gradient_y;
		this->gradient_z = other.gradient_z;
	}

	inline PointXYZING(const pcl::PointXYZRGBNormal& other, float gradient_x, float gradient_y, float gradient_z)
	{
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		this->data[3] = other.data[3];

		this->normal_x = other.normal_x;
		this->normal_y = other.normal_y;
		this->normal_z = other.normal_z;
		this->data_n[3] = other.data_n[3];

		this->intensity = (other.r + other.g + other.b) / 765.0f;
		this->gradient_x = gradient_x;
		this->gradient_y = gradient_y;
		this->gradient_z = gradient_z;
	}
	inline pcl::Vector3fMap getGradientVector3fMap() { return (pcl::Vector3fMap(data_g)); }
	inline pcl::Vector3fMapConst getGradientVector3fMap() const { return (pcl::Vector3fMapConst(data_g)); }
};
typedef std::vector<PointXYZING, Eigen::aligned_allocator<PointXYZING> > PointXYZRGBNGVector;
#endif