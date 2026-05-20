#ifndef __FEAT_FEATURES_HPP_
#define __FEAT_FEATURES_HPP_
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "../../third-party/ORBSLAM/include/ORBextractor.h"
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "LG_Logging.hpp"
#define NFEATURES 2000
#define MATCHRATIO 0.8f

typedef std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> PointPair2D;

struct OrbExtractor
{
    ORB_SLAM::ORBextractor orb;
    cv::BFMatcher matcher;
    fp64 matchratio;
};

struct OpenCVExtractAKAZE
{
    cv::Ptr<cv::AKAZE> akaze;
    cv::BFMatcher matcher;
    fp64 threshold;
    fp64 matchratio;
};

/*
 *************************************************************
 *************************************************************
 *************************************************************
 *                          ORB
 *************************************************************
 *************************************************************
 *************************************************************
 * */
struct OrbExtractor* ORB_InitORB();
void ORB_Destroy(struct OrbExtractor* orb);
PointPair2D ORB_GetMatches(void* extractor, cv::Mat img1, cv::Mat img2);

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
PointPair2D AKAZE_GetMatches(void* extractor, cv::Mat img1, cv::Mat img2);

#endif // __FEAT_FEATURES_HPP_
