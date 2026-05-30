#ifndef __EP_CORRESPONDING_POINTS_HPP_
#define __EP_CORRESPONDING_POINTS_HPP_
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "../../third-party/ORBSLAM/include/ORBextractor.h"
#include "CArenaAlloc.h"
#include "FEAT_Features.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#define EpiPolarTreshhold 4.0f

struct CorrespondenceExtractor
{
    void* extractor = nullptr;
    PointPair2D (*GetCorrp)(void* extractor, cv::Mat img1, cv::Mat img2);
};

void EP_InitCPointExtractor(void* extractor, PointPair2D (*GetCorrp)(void* extractor, cv::Mat img1, cv::Mat img2));
PointPair2D EP_CorrespExtract(cv::Mat img1, cv::Mat img2);

cv::Mat EP_EFromRigid(cv::Mat R, cv::Mat t);
//Returns Set of matches found by distance < EpiPolarTreshhold to epipolar line
std::pair<cv::Mat, cv::Mat> EP_GetR21t21(cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2);
PointPair2D EP_FindCorrpEpipolar(const PointPair2D& corrp, const cv::Mat& E);
PointPair2D EP_FilterPointPairByMask(const PointPair2D& corrp, const cv::Mat& mask);
void EP_DrawCorrespondences(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Point2d>& pts1,
        const std::vector<cv::Point2d>& pts2);

#endif //__EP_CORRESPONDING_POINTS_HPP_
