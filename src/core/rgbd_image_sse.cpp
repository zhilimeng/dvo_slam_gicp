#include <dvo/core/data_types.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/interpolation.h>

namespace dvo
{
	void RgbdImage::calculateDerivativeYSseFloat(const cv::Mat& img, cv::Mat& result)
	{
		result.create(img.size(), img.type());

		const float *prev_ptr, *next_ptr;
		float *result_ptr = result.ptr<float>();

		prev_ptr = img.ptr<float>(0); // point to row 0 (should point to -1)
		next_ptr = img.ptr<float>(1); // point to row 1

		const int inc = 4;

		__m128 scale = _mm_set1_ps(0.5f);

		// special loop for first row
		for (int x = 0; x < img.cols; x += inc, prev_ptr += inc, next_ptr += inc, result_ptr += inc)
		{
			_mm_store_ps(result_ptr, _mm_mul_ps(_mm_sub_ps(_mm_load_ps(next_ptr), _mm_load_ps(prev_ptr)), scale));
			//_mm_stream_ps(result_ptr + 4, _mm_sub_ps(_mm_load_ps(next_ptr + 4), _mm_load_ps(prev_ptr + 4)));
		}

		// prev_ptr points to row 1  (should point to 0)
		// next_ptr points to row 2

		prev_ptr -= img.cols; // go 1 row back

		for (int y = 1; y < img.rows - 1; y++)
		{
			for (int x = 0; x < img.cols; x += inc, prev_ptr += inc, next_ptr += inc, result_ptr += inc)
			{
				_mm_store_ps(result_ptr, _mm_mul_ps(_mm_sub_ps(_mm_load_ps(next_ptr), _mm_load_ps(prev_ptr)), scale));
				//_mm_stream_ps(result_ptr + 4, _mm_sub_ps(_mm_load_ps(next_ptr + 4), _mm_load_ps(prev_ptr + 4)));
			}
		}

		// special loop for last row
		next_ptr -= img.cols; // go 1 row back

		for (int x = 0; x < img.cols; x += inc, prev_ptr += inc, next_ptr += inc, result_ptr += inc)
		{
			_mm_store_ps(result_ptr, _mm_mul_ps(_mm_sub_ps(_mm_load_ps(next_ptr), _mm_load_ps(prev_ptr)), scale));
			//_mm_stream_ps(result_ptr + 4, _mm_sub_ps(_mm_load_ps(next_ptr + 4), _mm_load_ps(prev_ptr + 4)));
		}
	}
}