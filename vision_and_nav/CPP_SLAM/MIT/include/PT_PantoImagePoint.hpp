#ifndef __PT_PANTO_POINT_HPP_
#define __PT_PANTO_POINT_HPP_
#include "Config.hpp"
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <PT_Types.hpp>
#include <CM_Camera.hpp>
#include <PROJ_ProjectiveUtils.hpp>
#include <PANTO_Utils.hpp>
#include "PANTOVEC_PantoVector.hpp"
#include <algorithm>

typePantoKeypointFrame PT_CreatePantoImagePoints(const std::vector<cv::Point2d>& Points, const cv::Mat& Descriptors,
        std::vector<typePantoMapPoint>& CandidateMapPoints, const typeCamera& Pose);
typePantoKeypointFrame PT_CreatePantoImagePointsNoMatch(const std::vector<cv::Point2d>& Points, const cv::Mat& Descriptors);
#if defined(CONFIG_STEREO)
void PT_StereoMatch(typePantoKeypointFrame& KeyPointFrame, std::vector<cv::Point2d>& RightKeyPoints, const cv::Mat& RightDescriptors,
        const cv::Mat& GrayLeft, const cv::Mat& GrayRight);
void PT_LogStereoMatchData(void);
#endif // CONFIG_STEREO

#endif // __PT_PANTO_POINT_HPP_
