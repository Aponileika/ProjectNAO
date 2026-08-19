#ifndef __EP_CORRESPONDINGPOINTSPRIV_HPP
#define __EP_CORRESPONDINGPOINTSPRIV_HPP
#include "../../vendor/ANMS-Codes/C++/include/anms.h"
#include "../include/EP_CorrespondingPoints.hpp"
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "LG_Logging.hpp"
#include "PANTO_Utils.hpp"
#include <Eigen/Dense>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

struct AKAZEExtract {
  cv::Ptr<cv::AKAZE> akaze;
  cv::BFMatcher matcher;
  fp64 threshold;
  fp64 matchratio;
};

cv::Mat __EP_CrossProdMat(cv::Mat x);
void __EP_InitAkaze(void);
void __EP_destroy(void);
DescRet __EP_GetDesc(const cv::Mat& img);
static std::vector<cv::KeyPoint>
__EP_Anms(const cv::Ptr<cv::Feature2D> detector, cv::Mat img);
#endif // __EP_CORRESPONDINGPOINTSPRIV_HPP
