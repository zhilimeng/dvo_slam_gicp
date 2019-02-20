
#ifndef DVO_POINTCLOUD_IO_H
#define DVO_POINTCLOUD_IO_H

#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace dvo
{
	template<typename PointT>  void loadPointXYZ(const std::string &filename, pcl::PointCloud<PointT> &cloud,float scale = 1.0);
	template<typename PointT>  void savePointXYZ(const std::string &filename, pcl::PointCloud<PointT> &cloud);
	template<typename PointT>  bool savePointXYZBinary(const std::string &filename, pcl::PointCloud<PointT> &cloud);
	template<typename PointT>  bool loadPointXYZBinary(const std::string &filename, pcl::PointCloud<PointT> &cloud);
	template<typename PointT>  void savePointXYZ(const std::string &filename,const std::vector<int>& indices, pcl::PointCloud<PointT> &cloud);
	template<typename PointT>  void savePointXYZ(const std::string &filename, pcl::PointCloud<PointT> &cloud,const Eigen::Matrix4f& trans);

	template<typename PointT>  void loadPointXYZRGB(const std::string &filename, pcl::PointCloud<PointT> &cloud,float scale = 1.0,float color_scale = 1.0);
	template<typename PointT>  void savePointXYZRGB(const std::string &filename, pcl::PointCloud<PointT> &cloud);

	template<typename PointT>  void loadPointXYZNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud,float scale = 1.0);
	template<typename PointT>  void savePointXYZNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud);
	template<typename PointT>  void savePointXYZNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices);
	template<typename PointT>  void savePointXYZNormalPLY(const std::string &filename, pcl::PointCloud<PointT> &cloud);

	template<typename PointT>  void loadPointXYZRGBNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud,float scale = 1.0);
	template<typename PointT>  void loadPointNormalRGB(const std::string &filename, pcl::PointCloud<PointT> &cloud, float scale = 1.0);

	template<typename PointT>  void savePointXYZRGBNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &pose = Eigen::Matrix4f::Identity());

	template<typename PointT> int loadObj2PointCloud(const std::string &filename, pcl::PointCloud<PointT> &cloud,float scale = 1.0);
}



#include <dvo/core/impl/pointcloud_io.hpp>
#endif