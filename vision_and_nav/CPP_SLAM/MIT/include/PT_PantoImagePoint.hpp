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

typePantoKeypointFrame PT_CreatePantoImagePoints(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors, std::vector<typePantoMapPoint>& CandidateMapPoints, const typeCamera& Pose,
        typePantoVector<typePantoMapPoint>& GlobalMapPoints);
typePantoKeypointFrame PT_CreatePantoImagePointsNoMatch(const std::vector<cv::Point2d>& Points, const cv::Mat& Descriptors);
u64 PT_MatchMapPointsToKeyFrame(typePantoKeypointFrame& KeyFrame, std::vector<typePantoMapPoint>& MapPoints, const typeCamera& Pose,
        typePantoVector<typePantoMapPoint>& GlobalMapPoints);

#endif // __PT_PANTO_POINT_HPP_
