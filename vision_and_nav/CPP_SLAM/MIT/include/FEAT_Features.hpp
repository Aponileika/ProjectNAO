#ifndef __FEAT_FEATURES_HPP_
#define __FEAT_FEATURES_HPP_
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "../../third-party/ANMS-Codes/C++/include/anms.h"
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "LG_Logging.hpp"

typedef std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> PointPair2D;

struct OpenCVExtractAKAZE
{
    cv::Ptr<cv::AKAZE> akaze;
    cv::BFMatcher matcher;
    fp64 threshold;
    fp64 matchratio;
};

typedef std::pair<PointPair2D, std::pair<cv::Mat, cv::Mat>> CorrPReturn;

/*
 *************************************************************
 *************************************************************
 *************************************************************
 *                          AKAZE
 *************************************************************
 *************************************************************
 *************************************************************
 * */
struct OpenCVExtractAKAZE* AKAZE_InitAKAZE(fp64 threshold);
void AKAZE_destroy(struct OpenCVExtractAKAZE* akaze);
CorrPReturn AKAZE_GetMatches(void* extractor, cv::Mat img1, cv::Mat img2);

#endif // __FEAT_FEATURES_HPP_
