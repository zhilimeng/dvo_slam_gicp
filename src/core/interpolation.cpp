#include <dvo/core/interpolation.h>

namespace dvo
{
	IntensityType Interpolation::none(const cv::Mat& img, float x, float y)
	{
		int x0 = (int)std::floor(x);
		int y0 = (int)std::floor(y);
		return img.at<IntensityType>(y0, x0);
	}

	IntensityType Interpolation::bilinear(const cv::Mat& img, float x, float y)
	{
		int x0 = (int)std::floor(x);
		int y0 = (int)std::floor(y);

		int x1 = x0 + 1;
		int y1 = y0 + 1;
		
		float x_weight = x - x0;
		float y_weight = y - y0;

		float interpolated = 
			img.at<IntensityType>(y0, x0) * x_weight + img.at<IntensityType>(y0, x1) * (1 - x_weight) +
			img.at<IntensityType>(y1, x0) * x_weight + img.at<IntensityType>(y1, x1) * (1 - x_weight) +
			img.at<IntensityType>(y0, x0) * y_weight + img.at<IntensityType>(y1, x0) * (1 - y_weight) +
			img.at<IntensityType>(y0, x1) * y_weight + img.at<IntensityType>(y1, x1) * (1 - y_weight);

		return (IntensityType)(interpolated * 0.25f);
	}

	IntensityType Interpolation::bilinearWithDepthBuffer(const cv::Mat& intensity, const cv::Mat& depth, const float& x, const float& y, const float& z)
	{
		const int x0 = (int)std::floor(x);
		const int y0 = (int)std::floor(y);
		const int x1 = x0 + 1;
		const int y1 = y0 + 1;

		if (x1 >= intensity.cols || y1 >= intensity.rows)
		{
			return Invalid;
		}

		const float x1_weight = x - x0;
		const float x0_weight = 1.0f - x1_weight;
		const float y1_weight = y - y0;
		const float y0_weight = 1.0f - y1_weight;
		const float z_eps = z - 0.05f;

		float val = 0.0f;
		float sum = 0.0f;

		if (std::isfinite(depth.at<float>(y0, x0)) && depth.at<float>(y0, x0) > z_eps)
		{
			val += x0_weight * y0_weight * intensity.at<IntensityType>(y0, x0);
			sum += x0_weight * y0_weight;
		}

		if (std::isfinite(depth.at<float>(y0, x1)) && depth.at<float>(y0, x1) > z_eps)
		{
			val += x1_weight * y0_weight * intensity.at<IntensityType>(y0, x1);
			sum += x1_weight * y0_weight;
		}

		if (std::isfinite(depth.at<float>(y1, x0)) && depth.at<float>(y1, x0) > z_eps)
		{
			val += x0_weight * y1_weight * intensity.at<IntensityType>(y1, x0);
			sum += x0_weight * y1_weight;
		}

		if (std::isfinite(depth.at<float>(y1, x1)) && depth.at<float>(y1, x1) > z_eps)
		{
			val += x1_weight * y1_weight * intensity.at<IntensityType>(y1, x1);
			sum += x1_weight * y1_weight;
		}

		if (sum > 0.0f)
		{
			val /= sum;
		}
		else
		{
			val = Invalid;
		}

		return (IntensityType)val;
	}
}