#ifndef __EP_CORRESPONDING_POINTS_HPP_
#define __EP_CORRESPONDING_POINTS_HPP_
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "CArenaAlloc.h"
#define NFEATURES 1000
#define MATCHRATIO 0.75f

struct OrbExtractor
{
    cv::Ptr<cv::ORB> orb;
    cv::BFMatcher matcher;
    fp64 matchratio;
};

extern struct OrbExtractor orb;

typedef std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> PointPair2D;

void EP_InitCPointExtractor();
PointPair2D EP_CorrespExtract(cv::Mat img1, cv::Mat img2);
PointPair2D EP_FilterPointPairByMask(const PointPair2D& corrp, const cv::Mat& mask);
void EP_DrawCorrespondences(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Point2d>& pts1,
        const std::vector<cv::Point2d>& pts2);

#endif //__EP_CORRESPONDING_POINTS_HPP_
