#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mmintrin.h>
#include <emmintrin.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <dvo/core/utility.h>
namespace dvo
{
	//////////////////////////////////////////////////////////////////////////
	bool getFilesFromDirectory(const std::string &path_dir, const std::string &extension, std::vector<std::string> &files)
	{
		if (path_dir == "" || !boost::filesystem::exists(path_dir))
		{
			std::cerr << "Error: invalid path!\n" << path_dir << "\n";
			return (false);
		}
		boost::filesystem::directory_iterator it_end;
		for (boost::filesystem::directory_iterator it(path_dir); it != it_end; ++it)
		{
			if (!boost::filesystem::is_directory(it->status()) &&
				boost::algorithm::to_upper_copy(boost::filesystem::extension(it->path())) == boost::algorithm::to_upper_copy(extension))
			{
				files.push_back(it->path().string());
			}
		}
		if (files.empty())
		{
			std::cerr << "Error: No " << extension << " files found\n";
			return (false);
		}
		std::sort(files.begin(), files.end(), [&](const std::string& name1, const std::string& name2)->bool
		{
			if (name1.length() == name2.length())
				return name1 < name2;
			else
				return name1.length() < name2.length();

		});
		return (true);
	}

	//////////////////////////////////////////////////////////////////////////
	dvo::RgbdImagePyramidPtr loadRgbdImagePyramid(RgbdCameraPyramid& camera, const std::string &rgb_file, const std::string &depth_file, int image_id, float depth_scale)
	{
		cv::Mat rgb = cv::imread(rgb_file, 1);
		cv::Mat depth = cv::imread(depth_file, -1);
		return createRgbdImagePyramid(camera, rgb, depth, image_id, depth_scale);
	}

	//////////////////////////////////////////////////////////////////////////
	dvo::RgbdImagePyramidPtr createRgbdImagePyramid(RgbdCameraPyramid& camera, cv::Mat &rgb, cv::Mat &depth, int image_id, float depth_scale)
	{
		cv::Mat grey, grey_s16, depth_float;

		// convert rgb type to cv_32fc1
		if (rgb.type() != CV_32FC1)
		{
			if (rgb.type() == CV_8UC3)
				cv::cvtColor(rgb, grey, CV_BGR2GRAY);
			else
				grey = rgb;
			grey.convertTo(grey_s16, CV_32F);
		}
		else
		{
			grey_s16 = rgb;
		}

		// convert depth to float type, zero value replaced by non
		if (depth.type() != CV_32FC1)
			dvo::convertRawDepthImageSse(depth, depth_float, 1.0f / depth_scale);
		else
			depth_float = depth;

		dvo::RgbdImagePyramidPtr result = camera.create(grey_s16, depth_float);
		result->id(image_id);

		return result;
	}

	//////////////////////////////////////////////////////////////////////////
	void convertRawDepthImageSse(const cv::Mat& input, cv::Mat& output, float scale)
	{
		output.create(input.rows, input.cols, CV_32FC1);

		const unsigned short* input_ptr = input.ptr<unsigned short>();
		float* output_ptr = output.ptr<float>();

		__m128 _scale = _mm_set1_ps(scale);
		__m128 _zero = _mm_setzero_ps();
		__m128 _nan = _mm_set1_ps(std::numeric_limits<float>::quiet_NaN());

		for (int idx = 0; idx < input.size().area(); idx += 8, input_ptr += 8, output_ptr += 8)
		{
			__m128 _input, mask;
			__m128i _inputi = _mm_load_si128((__m128i*) input_ptr);

			// load low shorts and convert to float
			_input = _mm_cvtepi32_ps(_mm_unpacklo_epi16(_inputi, _mm_setzero_si128()));

			mask = _mm_cmpeq_ps(_input, _zero);

			// zero to nan
			_input = _mm_or_ps(_input, _mm_and_ps(mask, _nan));
			// scale
			_input = _mm_mul_ps(_input, _scale);
			// save
			_mm_store_ps(output_ptr + 0, _input);

			// load high shorts and convert to float
			_input = _mm_cvtepi32_ps(_mm_unpackhi_epi16(_inputi, _mm_setzero_si128()));

			mask = _mm_cmpeq_ps(_input, _zero);

			// zero to nan
			_input = _mm_or_ps(_input, _mm_and_ps(mask, _nan));
			// scale
			_input = _mm_mul_ps(_input, _scale);
			// save
			_mm_store_ps(output_ptr + 4, _input);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	bool saveRgbdImagePoints(const std::string &filename, RgbdImage& image, const Eigen::Matrix4d& pose, int step)
	{
		std::ofstream out(filename, std::ofstream::out);
		if (!out)
		{
			std::cerr << "can not open file\n";
			return false;
		}
		const PointCloudTemplate &pointcloud = image.camera().pointcloudTemplate();

		int height = image.height;
		int width = image.width;
		for (int y = 0; y < height; y += step)
		{
			for (int x = 0; x < width; x += step)
			{
				const float* depth_ptr = image.depth.ptr<const float>(y, x);
				int idx = y * width + x;
				if (std::isfinite(*depth_ptr))
				{
					float x = pointcloud.col(idx)(0) * (*depth_ptr);
					float y = pointcloud.col(idx)(1) * (*depth_ptr);
					float z = pointcloud.col(idx)(2) * (*depth_ptr);

					Eigen::Vector4d point = pose * Eigen::Vector4d(x, y, z, 1.0);
					out << std::setprecision(10) << std::fixed << point(0) << " " << point(1) << " " << point(2) << std::endl;
				}
			}
		}
		out.close();
		return true;
	}
	//////////////////////////////////////////////////////////////////////////
   bool saveTransformations(const std::string &filename, const std::vector<Eigen::Matrix4d> &poses, float scale)
	{
		std::ofstream fs;
		fs.open(filename.c_str(), std::ios::out);
		if (!fs.is_open() || fs.fail())
		{
			std::cerr << "could not open file\n";
			fs.close();
			return false;
		}
		for (size_t i = 0; i < poses.size(); ++i)
		{
			Eigen::Matrix4d pose = poses[i];
			pose(0, 3) *= scale;
			pose(1, 3) *= scale;
			pose(2, 3) *= scale;
			fs << std::setprecision(8) << std::fixed << pose << std::endl;
			fs << std::endl;
		}
		fs.close();
		return true;
	}

   //////////////////////////////////////////////////////////////////////////
   bool loadTransformations(const std::string &filename, std::vector<Eigen::Matrix4d> &poses, float scale)
   {
	   FILE* fp;
	   int err = fopen_s(&fp, filename.c_str(), "r");
	   if (err)
	   {
		   fprintf(stderr, "failed to open file for reading: %s\n", filename.c_str());
		   return false;
	   }
	   
	   float t00, t01, t02, t03;
	   float t10, t11, t12, t13;
	   float t20, t21, t22, t23;
	   float t30, t31, t32, t33;
	   while (fscanf_s(fp, "%f %f %f %f", &t00, &t01, &t02, &t03, 50) == 4)
	   {
		   fscanf_s(fp, "%f %f %f %f", &t10, &t11, &t12, &t13, 50);
		   fscanf_s(fp, "%f %f %f %f", &t20, &t21, &t22, &t23, 50);
		   fscanf_s(fp, "%f %f %f %f", &t30, &t31, &t32, &t33, 50);

		   Eigen::Matrix4d pose;
		   pose << t00, t01, t02, t03 * scale,
			   t10, t11, t12, t13 * scale,
			   t20, t21, t22, t23 * scale,
			   t30, t31, t32, t33;
		   poses.push_back(pose);
	   }
	   fclose(fp);
	   fp = NULL;
	   return true;
   }

   //////////////////////////////////////////////////////////////////////////
   bool saveTransformationBinary(const std::string &filename, const Eigen::Matrix4d &pose)
   {
	   std::ofstream file;

	   file.open(filename, std::ios::binary);
	   if (!file.is_open())
	   {
		   std::cerr << "Error in offline_integration.cpp: Could not open the file '" << filename << "'\n";
		   return (false);
	   }
	   for (int i = 0; i < 4; ++i)
	   {
		   for (int j = 0; j < 4; ++j)
		   {
			   file.write(reinterpret_cast<const char*>(&pose(i, j)), sizeof(double));
			   if (!file.good())
			   {
				   std::cerr << "Error in offline_integration.cpp: Could not read the transformation from the file.\n";
				   return (false);
			   }
		   }
	   }
	   return true;
   }

   //////////////////////////////////////////////////////////////////////////
   bool loadTransformationBinary(const std::string &filename, Eigen::Matrix4d &pose)
   {
	   std::ifstream file;
	   file.open(filename.c_str(), std::ios::binary);

	   if (!file.is_open())
	   {
		   std::cerr << "Error : Could not open the file '" << filename << "'\n";
		   return (false);
	   }

	   for (int i = 0; i < 4; ++i)
	   {
		   for (int j = 0; j < 4; ++j)
		   {
			   file.read(reinterpret_cast<char*>(&pose(i, j)), sizeof(double));

			   if (!file.good())
			   {
				   std::cerr << "Error in offline_integration.cpp: Could not read the transformation from the file.\n";
				   return (false);
			   }
		   }
	   }

	   return (true);
   }
}