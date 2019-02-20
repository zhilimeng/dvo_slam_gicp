/*!
 * \file utility.h
 * \brief 
 *
 * \author mengzhili
 * \date 2018/11/20 17:46
 *
 * 
 */
#ifndef DVO_UTILITY_H
#define DVO_UTILITY_H
#include <string>
#include <vector>
#include <dvo/core/rgbd_image.h>

namespace dvo
{
/* brief get filepath from directory
[param in]:path_dir - path directory  ie:"D:/testData/data"
[param in]:extension - file extension  ie:".png"
[param out]:files - filepath vector
[return]:bool
*/
DVO_EXPORTS bool getFilesFromDirectory(const std::string &path_dir, const std::string &extension, std::vector<std::string> &files);

/* brief load rgb and depth image, create RgbdImagePyramidPtr
[param in]:camera - RgbdCameraPyramid
[param in]:rgb_file - rgb file path
[param in]:depth_file - depth file path
[param in]:image_id - RgbdImage id
[param in]:depth_scale - depth scale
[return]:RgbdImagePyramidPtr
*/
DVO_EXPORTS dvo::RgbdImagePyramidPtr loadRgbdImagePyramid(dvo::RgbdCameraPyramid& camera, const std::string &rgb_file, const std::string &depth_file, int image_id, float depth_scale);
/* brief  create RgbdImagePyramidPtr from rgb and depth image,
[param in]:camera - RgbdCameraPyramid
[param in]:rgb - rgb image
[param in]:depth - depth image
[param in]:image_id -  id
[param in]:depth_scale - depth scale
[return]:RgbdImagePyramidPtr
*/
DVO_EXPORTS dvo::RgbdImagePyramidPtr createRgbdImagePyramid(dvo::RgbdCameraPyramid& camera, cv::Mat &rgb, cv::Mat &depth, int image_id, float depth_scale);

/* * \brief convert depth image to CV_32F
	* \param[in]:input - input depth image
	* \param[in]:scale - depth scale to divide
	* \param[out]:output - output CV_32F depth image
	* \return
*/
DVO_EXPORTS void convertRawDepthImageSse(const cv::Mat& input, cv::Mat& output, float scale);

/* * \brief save depth image's points to file
	* \param[in]:filename - output filename
	* \param[in]:image - RgbdImage
	* \param[in]:pose - image pose
	* \param[in]:step - sampe step,default 1;
	* \return
*/
DVO_EXPORTS bool saveRgbdImagePoints(const std::string &filename, RgbdImage& image, const Eigen::Matrix4d& pose, int step = 1);

/* * \brief save transformations to ascii file
	* \param[in]:filename - output filename
	* \param[in]:poses - Eigen::Matrix4d vector
	* \param[in]:scale - pose's translation scale
	* \return:bool - success or not
*/
DVO_EXPORTS bool saveTransformations(const std::string &filename, const std::vector<Eigen::Matrix4d> &poses, float scale = 1.0);

/* * \brief load transformations from ascii file
* \param[in]:filename - input filename
* \param[out]:poses - Eigen::Matrix4d vector
* \param[in]:scale - pose's translation scale
* \return:bool - success or not
*/
DVO_EXPORTS bool loadTransformations(const std::string &filename, std::vector<Eigen::Matrix4d> &poses, float scale = 1.0);

/* * \brief save transformation to binary file
	* \param[in]:filename - output filename
	* \param[in]:pose - Eigen::Matrix4d
	* \return:bool - success or not
*/
DVO_EXPORTS bool saveTransformationBinary(const std::string &filename, const Eigen::Matrix4d &pose);

/* * \brief load transformation from binary file
* \param[in]:filename - input filename
* \param[in]:pose - Eigen::Matrix4d
* \return:bool - success or not
*/
DVO_EXPORTS bool loadTransformationBinary(const std::string &filename, Eigen::Matrix4d &pose);




}
#endif
