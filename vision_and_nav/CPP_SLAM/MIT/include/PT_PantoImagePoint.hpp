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

typePantoKeypointFrame PT_CreatePantoImagePoints(std::vector<cv::Point2d> Points, 
        cv::Mat Descriptors, std::vector<typePantoMapPoint> CandidateMapPoints, typeCamera Pose);
u64 PT_MatchMapPointsToKeyFrame(typePantoKeypointFrame& KeyFrame, const std::vector<typePantoMapPoint>& MapPoints, const typeCamera& Pose);

#endif // __PT_PANTO_POINT_HPP_
