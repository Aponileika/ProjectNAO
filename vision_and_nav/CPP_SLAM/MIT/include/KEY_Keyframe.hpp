#ifndef __KEY_KEYFRAME_HPP_
#define __KEY_KEYFRAME_HPP_
#include "CArenaAlloc.h"
#include "PT_Types.hpp"
#include "Config.hpp"
#include <PT_PantoMapPoints.hpp>
#include <DBoW3/DBoW3.h>
#include <DBoW3/BowVector.h>
#include <DBoW3/FeatureVector.h>
#include <DBoW3/Vocabulary.h>
#include <vector>
#include <Eigen/Dense>
#include <PT_PantoImagePoint.hpp>
#include <PT_PantoMapPoints.hpp>
#include <CM_Camera.hpp>
#include <queue>

typedef struct
{
    // Velocity change compared to last two frames
    fp64 VelocityChange;
    // Ratio of matches in local map
    fp64 LocalMapTrackingRatio;
    // fp64 TrajectoryCurvature;
    // fp64 FeatureDistributionQuality;
    fp64 AcumulatedDistanceTravelled;
    // fp64 FeatureMatchingRate;
    // fp64 DynamicFeatureRatio; // What is this?
}typeKeyFrameInformation;

typedef struct
{
    typePantoKeypointFrame Points;
    DBoW3::BowVector BowVector;
    DBoW3::FeatureVector FeatureVector;
    typeCamera Pose;
    u64 ID;
}typeKeyFrame;


typeKeyFrame KEY_GetKeyFrame(const typeCamera& PredictedPose, const std::vector<typePantoMapPoint>& LastFrameMapPoints);
//TODO
bool KEY_IsKeyFrame(const typeKeyFrameInformation& Information);
void KEY_SetAsKeyFrame(typeKeyFrame& KeyFrame, const u64& ID, const DBoW3::Vocabulary& Vocabulary);
void KEY_GetNewMapPoints(typeKeyFrame& KeyFrame1, typeKeyFrame& KeyFrame2, std::vector<typePantoMapPoint>& GlobalMapPoints);
void KEY_PopKeyFrame(void);

#endif //__KEY_KEYFRAME_HPP_
