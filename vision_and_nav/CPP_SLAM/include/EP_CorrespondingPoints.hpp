#ifndef __EP_CORRESPONDING_POINTS_HPP_
#define __EP_CORRESPONDING_POINTS_HPP_
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "CArenaAlloc.h"
#define NFEATURES 5000 
#define MATCHRATIO 0.75f
#define EpiPolarTreshhold 1.0f

struct OrbExtractor
{
    cv::Ptr<cv::ORB> orb;
    cv::BFMatcher matcher;
    fp64 matchratio;
};

extern struct OrbExtractor orb;

typedef std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> PointPair2D;

void EP_InitCPointExtractor();
//TODO, see ORB slam for grid based approach to get a more uniform
//distribution of points, boils down to basically forcing the detector
//to detect a more uniform distribution of FAST keypoints before descriptor
//matching, see if there are improvements to this?
PointPair2D EP_CorrespExtract(cv::Mat img1, cv::Mat img2);

cv::Mat EP_EFromRigid(cv::Mat R, cv::Mat t);
//Returns Set of matches found by distance < EpiPolarTreshhold to epipolar line
PointPair2D EP_FindCorrpEpipolar(const PointPair2D& corrp, const cv::Mat& E);
PointPair2D EP_FilterPointPairByMask(const PointPair2D& corrp, const cv::Mat& mask);
void EP_DrawCorrespondences(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Point2d>& pts1,
        const std::vector<cv::Point2d>& pts2);

#endif //__EP_CORRESPONDING_POINTS_HPP_
