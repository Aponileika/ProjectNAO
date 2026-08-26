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
#include "PT_Types.hpp"
#include "PT_PantoImagePoint.hpp"
#include "PANTOVEC_PantoVector.hpp"


struct DescRet
{
    std::vector<cv::Point2d> Points;
    cv::Mat Descriptors;
};

void EP_InitCPointExtractor(void);
DescRet EP_GetDescriptors(const cv::Mat& Img);

cv::Mat EP_EFromRigid(cv::Mat R, cv::Mat t);
//Returns Set of matches found by distance < EpiPolarTreshhold to epipolar line
std::pair<cv::Mat, cv::Mat> EP_GetR21t21(cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2);

Eigen::Matrix3d EP_GetFundamentalMatrix21(const typeCameraPose& Pose1, const typeCameraPose& Pose2);
bool EP_CheckEpipolarConstraint(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2,
    const Eigen::Matrix3d& F21, const Eigen::Matrix3d& F12);

#endif //__EP_CORRESPONDING_POINTS_HPP_
