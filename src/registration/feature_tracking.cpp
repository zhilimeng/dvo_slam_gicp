#include <dvo/registration/feature_tracking.h>

#include <pcl/common/distances.h>
namespace dvo
{
template<typename PointT1, typename PointT2>
inline float squared2dEuclideanDistance(const PointT1& p1, const PointT2& p2)
{
	float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y;
	return diff_x * diff_x + diff_y * diff_y;
}

bool dvo::FeatureTracker::align(dvo::RgbdImage &ref, dvo::RgbdImage &mov, Matrix4& final_trans, const Matrix4& init_trans /*= Eigen::Matrix4d::Identity()*/)
{
	Result result;
	bool success;
	success = align(ref, mov, result, init_trans);
	final_trans = result.transformation;
	return success;
}

bool dvo::FeatureTracker::align(dvo::RgbdImage &ref, dvo::RgbdImage &mov, Result& result, const Matrix4& init_trans /*= Eigen::Matrix4d::Identity()*/)
{
	std::string detectorType = cfg_.detector_type;
	std::string descriptorType = cfg_.descriptor_type;
	int maxKeyPointNum = cfg_.max_keypoints_num;

	// detect keypoint
	ref.detectKeyPoint(detectorType, descriptorType, maxKeyPointNum);
	mov.detectKeyPoint(detectorType, descriptorType, maxKeyPointNum);

	ref.createFlannIndex();
	std::vector<cv::DMatch> matches;

	// find matches
	if (findMatches(ref, mov, matches, init_trans) < cfg_.min_valid_matches)
	{
		std::cout << "Error: too few matches: " << matches.size() << std::endl;
		result.transformation = Matrix4::Identity();
		return false;
	}
	// estimate transformation use ransac method
	estimateTransformation(ref, mov, matches, result);
	if (cfg_.verbose)
	{
		cv::Mat refImage, movImage, matchImage;
		ref.intensity.convertTo(refImage, CV_8U);
		mov.intensity.convertTo(movImage, CV_8U);
		//cv::Mat img_keypoints;
		//cv::drawKeypoints(refImage, ref.keys, img_keypoints);
		//cv::imshow("key point", img_keypoints);
		cv::drawMatches(movImage, mov.keys, refImage, ref.keys, matches, matchImage);
		cv::imshow("matched Image", matchImage);
		cv::waitKey(0);
	}

	return true;
}

int dvo::FeatureTracker::findMatches(dvo::RgbdImage& ref, dvo::RgbdImage& mov, std::vector<cv::DMatch> &matches, const Matrix4& init_trans /*= Eigen::Matrix4d::Identity()*/)
{
	int knn = cfg_.knn;
	float max_descriptor_ratio = cfg_.max_descriptor_ratio;
	float max_keypoint_2dDist = cfg_.max_keypoint_2d_dist;
	float max_keypoint_pair3d_dev_dist = cfg_.max_keypoint_pair3d_dev_dist;
	float min_keypoint_pair3d_dist = cfg_.min_keypoint_pair3d_dist;
	int min_valid_matches = cfg_.min_valid_matches;
	float squaredKeypointDist = max_keypoint_2dDist*max_keypoint_2dDist;
	float squaredPairKpt3dDevDist = max_keypoint_pair3d_dev_dist * max_keypoint_pair3d_dev_dist;
	float squaredPairKpt3dDist = min_keypoint_pair3d_dist * min_keypoint_pair3d_dist;

	bool useInitPose = init_trans.isIdentity() ? false : true;
	Eigen::Matrix4f init_transf = init_trans.cast<float>();

	// find nearest k match
	std::vector<std::vector<cv::DMatch> > knnMatches;
	knnMatch(ref, mov, knnMatches, knn);

	// remove invalid knn matches 
	std::vector<std::vector<cv::DMatch> > validKnnMatches;
	for (size_t i = 0; i < knnMatches.size(); ++i)
	{
		cv::DMatch& iMatch = knnMatches[i][0];

		float featureDescDistThresh = iMatch.distance / max_descriptor_ratio;
		std::vector<cv::DMatch> newIMatches;
		for (size_t j = 0; j < knnMatches[i].size(); ++j)
		{
			cv::DMatch ijMatch = knnMatches[i][j];
			// key point descriptor distance threshold
			if (j == 0 || ijMatch.distance < featureDescDistThresh)
			{
				const cv::Point2f& refPoint = ref.keys[ijMatch.trainIdx].pt;
				cv::Point2f movPoint = mov.keys[ijMatch.queryIdx].pt;
				const PointT& movPoint3d = mov.kpoints->points[ijMatch.queryIdx];
				if (useInitPose)
				{
					PointT mov_pt_trans;
					mov_pt_trans.getVector4fMap() = init_transf * movPoint3d.getVector4fMap();
					movPoint = mov.point2UV(mov_pt_trans);
				}
				// key point 2d coordinate distance threshold
				if (dvo::squared2dEuclideanDistance(refPoint, movPoint) < squaredKeypointDist)
				{
					newIMatches.push_back(ijMatch);
				}
			}
		}
		if (!newIMatches.empty())
		{
			validKnnMatches.push_back(newIMatches);
		}
	}

	std::vector<int> numValidMatches(validKnnMatches.size(), 0);

	for (size_t i = 0; i < validKnnMatches.size() - 1; ++i)
	{
		const cv::DMatch& imatch = validKnnMatches[i][0];

		const PointT& refx = ref.kpoints->points[imatch.trainIdx];
		const PointT& movx = mov.kpoints->points[imatch.queryIdx];

		for (size_t j = i + 1; j < validKnnMatches.size(); ++j)
		{
			const cv::DMatch& jmatch = validKnnMatches[j][0];

			const PointT& refy = ref.kpoints->points[jmatch.trainIdx];
			const PointT& movy = mov.kpoints->points[jmatch.queryIdx];

			float refXYDist2 = pcl::squaredEuclideanDistance(refx, refy);
			float movXYDist2 = pcl::squaredEuclideanDistance(movx, movy);
			float xyPairDevDist2 = refXYDist2 + movXYDist2 - 2 * std::sqrt(refXYDist2*movXYDist2);
			if (refXYDist2 > squaredPairKpt3dDist && movXYDist2 > squaredPairKpt3dDist && xyPairDevDist2 < squaredPairKpt3dDevDist)
			{
				numValidMatches[i]++;
				numValidMatches[j]++;
			}
		}
	}
	// sort matches by valid pairs num
	std::vector<CoMatch> coMatches;
	for (size_t i = 0; i < validKnnMatches.size(); ++i)
	{
		if (numValidMatches[i] > min_valid_matches)
		{
			CoMatch comatch;
			comatch.match = validKnnMatches[i][0];
			comatch.valid_pair_num = numValidMatches[i];
			coMatches.push_back(comatch);
		}
	}

	std::sort(coMatches.begin(), coMatches.end(), [&](const CoMatch &m1, const CoMatch &m2)->bool
	{ return m1.valid_pair_num > m2.valid_pair_num; });

	// find best three matches
	std::vector<cv::DMatch> validMatches;

	for (size_t i = 0; i < coMatches.size() - 1; ++i)
	{
		CoMatch& refCoMatch = coMatches[i];
		validMatches.clear();
		validMatches.push_back(refCoMatch.match);
		const PointT& refx = ref.kpoints->points[refCoMatch.match.trainIdx];
		const PointT& movx = mov.kpoints->points[refCoMatch.match.queryIdx];
		for (size_t j = i + 1; j < coMatches.size(); ++j)
		{
			CoMatch& curCoMatch = coMatches[j];
			const PointT& refy = ref.kpoints->points[curCoMatch.match.trainIdx];
			const PointT& movy = mov.kpoints->points[curCoMatch.match.queryIdx];

			float refXY2 = pcl::squaredEuclideanDistance(refx, refy);
			float movXY2 = pcl::squaredEuclideanDistance(movx, movy);
			float xyPairDevDist2 = refXY2 + movXY2 - 2 * std::sqrt(refXY2*movXY2);
			if (refXY2 > squaredPairKpt3dDist && movXY2 > squaredPairKpt3dDist && xyPairDevDist2 < 0.25*squaredPairKpt3dDevDist)
			{
				validMatches.push_back(curCoMatch.match);
			}
			if (validMatches.size() > 3)
			{
				break;
			}
		}
		if (validMatches.size() > 3)
		{
			break;
		}
	}
	if (validMatches.size() < 3)
		return 0;

	cv::DMatch& m1 = validMatches[0];
	cv::DMatch& m2 = validMatches[1];
	cv::DMatch& m3 = validMatches[2];

	const PointT& refP1 = ref.kpoints->points[m1.trainIdx];
	const PointT& refP2 = ref.kpoints->points[m2.trainIdx];
	const PointT& refP3 = ref.kpoints->points[m3.trainIdx];
	const PointT& movP1 = mov.kpoints->points[m1.queryIdx];
	const PointT& movP2 = mov.kpoints->points[m2.queryIdx];
	const PointT& movP3 = mov.kpoints->points[m3.queryIdx];

	for (size_t i = 0; i < validKnnMatches.size(); ++i)
	{
		const std::vector<cv::DMatch>& iMatches = validKnnMatches[i];
		for (size_t j = 0; j < iMatches.size(); ++j)
		{
			const cv::DMatch& match = iMatches[j];
			const PointT& refx = ref.kpoints->points[match.trainIdx];
			const PointT& movx = mov.kpoints->points[match.queryIdx];

			if (validateSegment(refx, refP1, movx, movP1, squaredPairKpt3dDevDist) &&
				validateSegment(refx, refP2, movx, movP2, squaredPairKpt3dDevDist) &&
				validateSegment(refx, refP3, movx, movP3, squaredPairKpt3dDevDist))
			{
				matches.push_back(match);
				break;
			}
		}
	}
	return static_cast<int>(matches.size());

}
void dvo::FeatureTracker::knnMatch(dvo::RgbdImage& ref, dvo::RgbdImage& mov, std::vector<std::vector<cv::DMatch> >& matches, int knn)
{
	cv::Mat indices(mov.keys_desc.rows, knn, CV_32SC1);
	cv::Mat dists(mov.keys_desc.rows, knn, CV_32FC1);

	ref.flann_index->knnSearch(mov.keys_desc, indices, dists, knn, cv::flann::SearchParams());
	matches.clear();
	matches.resize(indices.rows);

	for (int i = 0; i < indices.rows; ++i)
	{
		for (int j = 0; j < indices.cols; ++j)
		{
			int idx = indices.at<int>(i, j);
			if (idx >= 0)
			{
				int train_idx = indices.at<int>(i, j);
				float dist = dists.at<float>(i, j);
				matches[i].push_back(cv::DMatch(i, train_idx, dist));
			}
		}
	}
}

bool dvo::FeatureTracker::validateSegment(const PointT& p0, const PointT& p1, const PointT& q0, const PointT& q1, float threshold)
{
	float length_p2 = pcl::squaredEuclideanDistance(p0, p1);
	float length_q2 = pcl::squaredEuclideanDistance(q0, q1);
	float diff2 = length_p2 + length_q2 - 2 * std::sqrt(length_p2*length_q2);
	if (diff2 < threshold)
		return true;
	return false;
}

void dvo::FeatureTracker::estimateTransformation(dvo::RgbdImage& ref, dvo::RgbdImage& mov, std::vector<cv::DMatch> &matches, Result& result)
{
	Matrix4 guess, final_transformation;
	std::vector<int> indice_src, indice_tgt;
	for (size_t i = 0; i < matches.size(); ++i)
	{
		indice_tgt.push_back(matches[i].trainIdx);
		indice_src.push_back(matches[i].queryIdx);
	}
	transformation_estimation_->estimateRigidTransformation(*mov.kpoints, indice_src, *ref.kpoints, indice_tgt, guess);
	final_transformation = guess;
	float error, lowest_error(0);
	float corr_dist = cfg_.max_corr_dist;
	lowest_error = ref.computeErrorMetric(mov, guess, corr_dist);

	std::vector<int> sample_indices_src, sample_indices_tgt;
	for (int iter = 0; iter < cfg_.max_iterations; ++iter)
	{
		// random select samples
		std::vector<int> sample_indices;
		while (sample_indices.size() < 4)
		{
			sample_indices.push_back(getRandomIndex(static_cast<int>(indice_src.size())));
		}
		// find corresponding feautres indices
		sample_indices_src.clear();
		sample_indices_tgt.clear();
		for (const auto& idx : sample_indices)
		{
			sample_indices_src.push_back(indice_src[idx]);
			sample_indices_tgt.push_back(indice_tgt[idx]);
		}
		// estimate the transform from samples
		transformation_estimation_->estimateRigidTransformation(*mov.kpoints, sample_indices_src, *ref.kpoints, sample_indices_tgt, guess);

		// compute the error
		error = ref.computeErrorMetric(mov, guess, corr_dist);

		// if the new error is lower, update the final transformation
		if (error < lowest_error)
		{
			lowest_error = error;
			final_transformation = guess;
//			std::cout << "error: " << lowest_error << std::endl;
		}
	}

	result.transformation = final_transformation;
	return;
}
}



