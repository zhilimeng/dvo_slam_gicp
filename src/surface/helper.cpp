
#include <dvo/surface/helper.h>
namespace dvo
{
	cv::Mat createDepthToCameraDistanceMultiplierImage(const IntrinsicMatrix& intrinsic, int width, int height)
	{
		cv::Mat image(height, width, CV_32F);

		float fx_inverse = 1.0f / intrinsic.fx();
		float fy_inverse = 1.0f / intrinsic.fy();

		float cx = intrinsic.cx();
		float cy = intrinsic.cy();

		std::vector<float> xx(width);
		std::vector<float> yy(height);
		for (int x = 0; x < width; ++x)
			xx[x] = (x - cx) * fx_inverse;
		for (int y = 0; y < height; ++y)
			yy[y] = (y - cy) * fy_inverse;

		float* image_ptr = image.ptr<float>();
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x, ++image_ptr)
			{
				*image_ptr = std::sqrt(xx[x] * xx[x] + yy[y] * yy[y] + 1.0f);
			}
		}
		return image;
	}
}
