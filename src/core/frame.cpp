#include <dvo/core/frame.h>

namespace dvo
{
//////////////////////////////////////////////////////////////////////////
Frame::~Frame()
{

}

//////////////////////////////////////////////////////////////////////////
Frame::Ptr Frame::create(const dvo::RgbdImagePyramidPtr& image, const dvo::IPointCloudPtr &ipointcloud, const Matrix4& pose /*= Matrix4::Identity()*/)
{
	Frame::Ptr frame(new Frame(image, ipointcloud, pose));
	return frame;
}

//////////////////////////////////////////////////////////////////////////
Frame::Frame(const dvo::RgbdImagePyramidPtr& image, const dvo::IPointCloudPtr &ipointcloud, const Matrix4& pose /*= Matrix4::Identity()*/)
	:image_(image), icloud_(ipointcloud), pose_(pose)
{

}

}
