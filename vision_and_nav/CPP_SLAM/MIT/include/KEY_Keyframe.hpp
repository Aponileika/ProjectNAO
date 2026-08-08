#ifndef __KEY_KEYFRAME_HPP_
#define __KEY_KEYFRAME_HPP_
#include "CArenaAlloc.h"
#include <vector>
#include <Eigen/Dense>
#include <PT_PantoImagePoint.hpp>
#include <PT_PantoMapPoints.hpp>
#include <CM_Camera.hpp>

typedef struct
{
    fp64 VelocityChange;
    fp64 TrajectoryCurvature;
    fp64 FeatureDistributionQuality;
    fp64 FeatureMatchingRate;
    fp64 DynamicFeatureRatio;
}typeKeyFrameInformation;

typedef struct
{
    std::vector<typePantoImagePoint> Points;
    Camera Pose;
    std::vector<u64> MapPointIDs;
}typeKeyFrame;


typeKeyFrame KEY_GetKeyFrame(Camera PredictedPose, std::vector<typePantoMapPoint> MapPointIDs);
bool KEY_IsKeyFrame(typeKeyFrameInformation Information);

#endif //__KEY_KEYFRAME_HPP_
