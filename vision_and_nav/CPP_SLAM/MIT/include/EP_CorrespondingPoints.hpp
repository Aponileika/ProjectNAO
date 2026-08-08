#ifndef __EP_CORRESPONDING_POINTS_HPP_
#define __EP_CORRESPONDING_POINTS_HPP_
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include "CArenaAlloc.h"
#include "PROJ_ProjectiveUtils.hpp"
#include "Config.hpp"
#include "PANTO_Utils.hpp"
#include "OB_Observations.hpp"
#include "VW_Views.hpp"


struct MatchesRet 
{
    PointPair2D Matches;
    std::pair<cv::Mat, cv::Mat> Descriptors;
};

struct DescRet
{
    std::vector<cv::Point2d> Points;
    cv::Mat Descriptors;
};

void EP_InitCPointExtractor(void);
DescRet EP_GetDescriptors(cv::Mat Img);
MatchesRet EP_CorrespExtract(cv::Mat img1, cv::Mat img2);
MatchesRet EP_GetMatches(std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> Points, std::pair<std::vector<cv::Mat>, std::vector<cv::Mat>> Descriptors);

cv::Mat EP_EFromRigid(cv::Mat R, cv::Mat t);
//Returns Set of matches found by distance < EpiPolarTreshhold to epipolar line
std::pair<cv::Mat, cv::Mat> EP_GetR21t21(cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2);
PointPair2D EP_FindCorrpEpipolar(const PointPair2D& corrp, const cv::Mat& E);
PointPair2D EP_FilterPointPairByMask(const PointPair2D& corrp, const cv::Mat& mask);
void EP_DrawCorrespondences(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Point2d>& pts1,
        const std::vector<cv::Point2d>& pts2);
std::vector<std::vector<Eigen::Vector3d>> EP_PointPair2Eigen(PointPair2D pp);

#endif //__EP_CORRESPONDING_POINTS_HPP_
