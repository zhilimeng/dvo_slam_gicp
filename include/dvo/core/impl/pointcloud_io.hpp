#ifndef DVO_POINTCLOUD_IO_HPP
#define  DVO_POINTCLOUD_IO_HPP
#include <dvo/core/pointcloud_io.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

namespace dvo
{
	template<typename PointT>  
	void loadPointXYZ(const std::string &filename, pcl::PointCloud<PointT> &cloud, float scale /*= 1.0*/)
	{
		FILE* fp;
		int err = fopen_s(&fp, filename.c_str(), "r");
		if (err)
		{
			fprintf(stderr, "failed to open file for reading: %s\n", filename.c_str());
			exit(0);
		}
		PointT point;
		float x, y, z;
		while (fscanf_s(fp, "%f %f %f", &x, &y, &z,50) == 3)
		{
			point.x = scale * x;
			point.y = scale * y;
			point.z = scale * z;
			cloud.push_back(point);
		}
		fclose(fp);
		fp = NULL;
	}
	template<typename PointT> bool savePointXYZBinary(const std::string &filename, pcl::PointCloud<PointT> &cloud)
	{
		std::ofstream file;

		file.open(filename, std::ios::binary);
		if (!file.is_open())
		{
			std::cerr << "Error in offline_integration.cpp: Could not open the file '" << filename << "'\n";
			return (false);
		}
		int point_num = static_cast<int>(cloud.size());
		file.write(reinterpret_cast<char*>(&point_num), sizeof(int));
		for (int i = 0; i < point_num; ++i)
		{
			file.write(reinterpret_cast<char*>(cloud.points[i].data), 4 * sizeof(float));
		}
		file.close();
		return true;
	}
	template<typename PointT> bool loadPointXYZBinary(const std::string &filename, pcl::PointCloud<PointT> &cloud)
	{
		std::ifstream file;

		file.open(filename, std::ios::binary);
		if (!file.is_open())
		{
			std::cerr << "Error in offline_integration.cpp: Could not open the file '" << filename << "'\n";
			return (false);
		}
		int point_num = 0;
		file.read(reinterpret_cast<char*>(&point_num), sizeof(int));
		cloud.resize(point_num);
		for (int i = 0; i < point_num; ++i)
		{
			file.read(reinterpret_cast<char*>(cloud.points[i].data), 4 * sizeof(float));
		}
		file.close();
		return true;
	}
	template<typename PointT>  void savePointXYZ(const std::string &filename, pcl::PointCloud<PointT> &cloud)
	{
		std::ofstream out(filename, std::ofstream::out);
		if (!out)
			return;
		size_t num = cloud.points.size();
		for (size_t i = 0; i < num; ++i)
		{
			out << std::setprecision(8) << std::fixed << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
		}
		out.close();
	}
	template<typename PointT>  void savePointXYZ(const std::string &filename, const std::vector<int>& indices, pcl::PointCloud<PointT> &cloud)
	{
		std::ofstream out(filename, std::ofstream::out);
		if (!out)
			return;
		for (size_t i = 0; i < indices.size(); ++i)
		{
			out << std::setprecision(8) << std::fixed << cloud.points[indices[i]].x << " " << cloud.points[indices[i]].y << " " << cloud.points[indices[i]].z << std::endl;
		}
		out.close();
	}
	template<typename PointT>  void savePointXYZ(const std::string &filename, pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f& trans)
	{
		std::ofstream out(filename, std::ofstream::out);
		if (!out)
			return;
		size_t num = cloud.points.size();
		for (size_t i = 0; i < num; ++i)
		{
			const PointT &src_pt = cloud.points[i];
			PointT tgt;
			tgt.x = trans(0, 0) * src_pt.x + trans(0, 1)*src_pt.y + trans(0, 2)*src_pt.z + trans(0, 3);
			tgt.y = trans(1, 0) * src_pt.x + trans(1, 1)*src_pt.y + trans(1, 2)*src_pt.z + trans(1, 3);
			tgt.z = trans(2, 0) * src_pt.x + trans(2, 1)*src_pt.y + trans(2, 2)*src_pt.z + trans(2, 3);
			out << std::setprecision(8) << std::fixed << tgt.x << " " << tgt.y << " " << tgt.z << std::endl;
		}
		out.close();
	}
	template<typename PointT>  
	void loadPointXYZRGB(const std::string &filename, pcl::PointCloud<PointT> &cloud, float scale /*= 1.0*/, float color_scale  /*= 1.0*/)
	{
		FILE* fp;
		int err = fopen_s(&fp, filename.c_str(), "r");
		if (err)
		{
			fprintf(stderr, "failed to open file for reading: %s\n", filename.c_str());
			exit(0);
		}
		PointT point;
		float x, y, z, r, g, b;
		while (fscanf_s(fp, "%f %f %f %f %f %f", &x, &y, &z, &r, &g, &b, 50) == 6)
		{
			point.x = scale * x;
			point.y = scale * y;
			point.z = scale * z;
			point.r = static_cast<uint8_t>(color_scale*r);
			point.g = static_cast<uint8_t>(color_scale*g);
			point.b = static_cast<uint8_t>(color_scale*b);
			cloud.push_back(point);
		}
		fclose(fp);
		fp = NULL;
	}

	template<typename PointT>  void savePointXYZRGB(const std::string &filename, pcl::PointCloud<PointT> &cloud)
	{
		std::ofstream out(filename, std::ofstream::out);
		if (!out)
			return;
		size_t num = cloud.points.size();
		for (size_t i = 0; i < num; ++i)
		{
			out << std::setprecision(6) << std::fixed << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " "
				<< (int)cloud.points[i].r << " " << (int)cloud.points[i].g << " " << (int)cloud.points[i].b << std::endl;
		}
		out.close();
	}

	template<typename PointT>  void loadPointXYZNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud, float scale /*= 1.0*/)
	{
		FILE* fp;
	    int err = fopen_s(&fp,filename.c_str(), "r");
		if (err)
		{
			fprintf(stderr, "failed to open file for reading: %s\n", filename.c_str());
			exit(0);
		}
		PointT point;
		float x, y, z, nx, ny, nz;
		while (fscanf_s(fp, "%f %f %f %f %f %f", &x, &y, &z, &nx, &ny, &nz,50) == 6)
		{
			point.x = scale * x;
			point.y = scale * y;
			point.z = scale * z;
			point.normal_x = nx;
			point.normal_y = ny;
			point.normal_z = nz;
			cloud.push_back(point);
		}
		fclose(fp);
		fp = NULL;
	}
	template<typename PointT>  void savePointXYZNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud)
	{
		FILE* fp;
		int err = fopen_s(&fp, filename.c_str(), "w");
		if (err)
		{
			fprintf(stderr, "failed to open file for writing: %s\n", filename.c_str());
			exit(0);
		}
		size_t num = cloud.points.size();
		for (size_t i = 0; i < num; ++i)
		{
			if (fprintf(fp, "%.10f %.10f %.10f %.10f %.10f %.10f\n", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,
				cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z) < 0)
			{
				fprintf(stderr, "write failed:unable to write file: %s\n", filename.c_str());
				fclose(fp);
				exit(0);
			}
		}
		fclose(fp);
		fp = NULL;
	}
	template<typename PointT>  
	void savePointXYZNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices)
	{
		FILE* fp;
		int err = fopen_s(&fp, filename.c_str(), "w");
		if (err)
		{
			fprintf(stderr, "failed to open file for writing: %s\n", filename.c_str());
			exit(0);
		}
		size_t num = cloud.points.size();
		for (size_t i = 0; i < indices.size(); ++i)
		{
			if (fprintf(fp, "%.10f %.10f %.10f %.10f %.10f %.10f\n", cloud.points[indices[i]].x, cloud.points[indices[i]].y, cloud.points[indices[i]].z,
				cloud.points[indices[i]].normal_x, cloud.points[indices[i]].normal_y, cloud.points[indices[i]].normal_z) < 0)
			{
				fprintf(stderr, "write failed:unable to write file: %s\n", filename.c_str());
				fclose(fp);
				exit(0);
			}
		}
		fclose(fp);
		fp = NULL;
	}
	template<typename PointT>  void savePointXYZNormalPLY(const std::string &filename, pcl::PointCloud<PointT> &cloud)
	{
		std::ofstream out(filename, std::ofstream::out);
		if (!out)
		{
			return;
		}
		out << "ply\n" << "format ascii " << 1.0 << std::endl;
		out << "comment dvo output\n";
		out << "element vertex " << cloud.points.size() << std::endl;
		out << "property float x\n" << "property float y\n" << "property float z\n";
		out << "property float nx\n" << "property float ny\n" << "property float nz\n";
		out << "element face " << 0 << std::endl;
		out << "end_header\n";
		for (size_t i = 0; i < cloud.points.size(); ++i)
		{
			out << std::setprecision(6) << std::fixed << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " "
				<< cloud.points[i].normal_x << " " << cloud.points[i].normal_y << " " << cloud.points[i].normal_z << std::endl;
		}
		out.close();
	}
	template<typename PointT>  void loadPointXYZRGBNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud, float scale /*= 1.0*/)
	{
		FILE* fp;
		int err = fopen_s(&fp, filename.c_str(), "r");
		if (err)
		{
			fprintf(stderr, "failed to open file for reading: %s\n", filename.c_str());
			exit(0);
		}
		PointT point;
		float x, y, z, nx, ny, nz;
		int r, g, b;
		while (fscanf_s(fp, "%f %f %f %d %d %d %f %f %f", &x, &y, &z, &r, &g, &b, &nx, &ny, &nz, 50) == 9)
		{
			point.x = scale * x;
			point.y = scale * y;
			point.z = scale * z;
			point.r = r;
			point.g = g;
			point.b = b;
			point.normal_x = nx;
			point.normal_y = ny;
			point.normal_z = nz;
			cloud.push_back(point);
		}
		fclose(fp);
		fp = NULL;
	}
	template<typename PointT>
	void loadPointNormalRGB(const std::string &filename, pcl::PointCloud<PointT> &cloud, float scale /*= 1.0*/)
	{
		FILE* fp;
		int err = fopen_s(&fp, filename.c_str(), "r");
		if (err)
		{
			fprintf(stderr, "failed to open file for reading: %s\n", filename.c_str());
			exit(0);
		}
		PointT point;
		float x, y, z, nx, ny, nz;
		int r, g, b;
		while (fscanf_s(fp, "%f %f %f %f %f %f %d %d %d", &x, &y, &z, &nx, &ny, &nz, &r, &g, &b, 50) == 9)
		{
			point.x = scale * x;
			point.y = scale * y;
			point.z = scale * z;
			point.r = r;
			point.g = g;
			point.b = b;
			point.normal_x = nx;
			point.normal_y = ny;
			point.normal_z = nz;
			cloud.push_back(point);
		}
		fclose(fp);
		fp = NULL;
	}

	template<typename PointT>  void savePointXYZRGBNormal(const std::string &filename, pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &pose)
	{
		std::ofstream out(filename, std::ofstream::out);
		if (!out)
			return;
		size_t num = cloud.points.size();
		for (size_t i = 0; i < num; ++i)
		{
			PointT pt_d = cloud.points[i];
			pt_d.getVector4fMap() = pose * pt_d.getVector4fMap();
			pt_d.getNormalVector4fMap() = pose * pt_d.getNormalVector4fMap();
			out << std::setprecision(8) << std::fixed << pt_d.x << " " << pt_d.y << " " << pt_d.z << " "
				<< (int)pt_d.r << " " << (int)pt_d.g << " " << (int)pt_d.b << " "
				<< pt_d.normal_x << " " << pt_d.normal_y << " " << pt_d.normal_z << std::endl;
		}
		out.close();
	}
	template<typename PointT> int loadObj2PointCloud(const std::string &filename, pcl::PointCloud<PointT> &cloud, float scale /*= 1.0*/)
	{
		std::ifstream fs;
		std::string line;

		//Open file in binary mode
		fs.open(filename.c_str(), std::ios::binary);
		if (!fs.is_open()||fs.fail())
		{
			std::cerr << "could not open file\n";
			fs.close();
			return -1;
		}

		size_t nr_point = 0;
		std::vector<std::string> st;
		try
		{
			while (!fs.eof())
			{
				getline(fs, line);

				// ignore empty lines
				if (line == "")
					continue;

				// Tokenize the line
				std::stringstream ss(line);
				ss.imbue(std::locale::classic());
				line = ss.str();
				boost::trim(line);
				boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
				//ignore comments
				if (st.at(0) == "#")
					continue;
				if (st.at(0) == "v")
				{
					++nr_point;
					continue;
				}
			}
		}
		catch (const char* e)
		{
			printf("%s\n", e);
			fs.close();
			return -1;
		}
		fs.close();
		fs.open(filename.c_str(), std::ios::binary);
		if (!fs.is_open() || fs.fail())
		{
			std::cerr << "could not open file\n";
			fs.close();
			return -1;
		}
		// seek at the begin
//		fs.seekg(0,std::ios::beg);
		size_t point_idx = 0;
		size_t normal_idx = 0;
		cloud.resize(nr_point);
		try
		{
			while (!fs.eof())
			{
				getline(fs, line);			
				// ignore empty lines
				if (line == "")
					continue;

				// Tokenize the line
				std::stringstream ss(line);
				ss.imbue(std::locale::classic());
				line = ss.str();
				boost::trim(line);
				boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
				//ignore comments
				if (st.at(0) == "#")
					continue;
				// vertex
				if (st.at(0) == "v")
				{
					for (int i = 1; i < 4; ++i)
					{
						float value = boost::lexical_cast<float>(st[i]);
						cloud.points[point_idx].data[i - 1] = scale*value;
					}
					++point_idx;
					continue;
				}
				// vertex normal
				if (st.at(0) == "vn")
				{
					for (int i = 1; i < 4; ++i)
					{
						float value = boost::lexical_cast<float>(st[i]);
						cloud.points[normal_idx].data_n[i - 1] = value;
					}
					++normal_idx;
					continue;
				}
			}
		}
		catch (const char* e)
		{
			printf("%s \n", e);
			fs.close();
			return -1;
		}

		fs.close();
		return 0;
	}
}



#endif