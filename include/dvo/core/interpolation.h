#ifndef	DVO_INTERPOLATION_H
#define DVO_INTERPOLATION_H

#include <opencv2/opencv.hpp>
#include <dvo/core/data_types.h>
namespace dvo
{
struct Interpolation
{
	static IntensityType none(const cv::Mat& img, float x, float y);
	static IntensityType bilinear(const cv::Mat& img, float x, float y);
	static IntensityType bilinearWithDepthBuffer(const cv::Mat& intensity, const cv::Mat& depth, const float &x, const float &y, const float& z);

	template<typename T> static
	void pyrDownMeanSmooth(const cv::Mat& in, cv::Mat& out)
	{
		out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());

		for (int y = 0; y < out.rows; ++y)
		{
			for (int x = 0; x < out.cols; ++x)
			{
				int x0 = x * 2;
				int x1 = x0 + 1;
				int y0 = y * 2;
				int y1 = y0 + 1;

				out.at<T>(y, x) = (T)((in.at<T>(y0, x0) + in.at<T>(y0, x1) + in.at<T>(y1, x0) + in.at<T>(y1, x1)) / 4.0f);
			}
		}
	}

	template<typename T> static
	void pyrDownMeanSmoothIgnoreInvalid(const cv::Mat& in, cv::Mat& out)
	{
		out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());

		for (int y = 0; y < out.rows; ++y)
		{
			for (int x = 0; x < out.cols; ++x)
			{
				int x0 = x * 2;
				int x1 = x0 + 1;
				int y0 = y * 2;
				int y1 = y0 + 1;

				T total = 0;
				int cnt = 0;

				if (std::isfinite(in.at<T>(y0, x0)))
				{
					total += in.at<T>(y0, x0);
					cnt++;
				}

				if (std::isfinite(in.at<T>(y0, x1)))
				{
					total += in.at<T>(y0, x1);
					cnt++;
				}

				if (std::isfinite(in.at<T>(y1, x0)))
				{
					total += in.at<T>(y1, x0);
					cnt++;
				}

				if (std::isfinite(in.at<T>(y1, x1)))
				{
					total += in.at<T>(y1, x1);
					cnt++;
				}

				if (cnt > 0)
				{
					out.at<T>(y, x) = (T)(total / cnt);
				}
				else
				{
					out.at<T>(y, x) = InvalidDepth;
				}
			}
		}
	}

	template<typename T> static
	void pyrDownMedianSmooth(const cv::Mat& in, cv::Mat& out)
	{
		out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());

		cv::Mat in_smoothed;
		cv::medianBlur(in, in_smoothed, 3);

		for (int y = 0; y < out.rows; ++y)
		{
			for (int x = 0; x < out.cols; ++x)
			{
				out.at<T>(y, x) = in_smoothed.at<T>(y * 2, x * 2);
			}
		}
	}

	template<typename T> static
	void pyrDownSubsample(const cv::Mat& in, cv::Mat& out)
	{
		out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());

		for (int y = 0; y < out.rows; ++y)
		{
			for (int x = 0; x < out.cols; ++x)
			{
				out.at<T>(y, x) = in.at<T>(y * 2, x * 2);
			}
		}
	}
};
}
#endif