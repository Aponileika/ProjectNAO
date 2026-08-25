#ifndef KEY_KEYFRAME_HPP_
#define KEY_KEYFRAME_HPP_
#include "CArenaAlloc.h"
#include "PT_Types.hpp"
#include "DBOW3_DeepBagofWords.hpp"
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
#include <cmath>
#include <limits>
#include <algorithm>

typedef struct
{
    // Velocity change compared to last two frames
    fp64 VelocityChange;
    // Ratio of matches in local map
    fp64 LocalMapTrackingRatio;
    // fp64 TrajectoryCurvature;
    // fp64 FeatureDistributionQuality;
    fp64 AcumulatedDistanceTravelled;
}typeKeyFrameInformation;

typedef struct
{
    typePantoKeypointFrame Points;
    DBoW3::BowVector BowVector;
    DBoW3::FeatureVector FeatureVector;
    typeCamera Pose;
    u64 ID;
    std::string ImagePath;
}typeKeyFrame;

typeKeyFrame KEY_GetThirdKeyFrame(typeKeyFrame& LastKeyFrame, std::vector<typePantoMapPoint>& GlobalMapPoints);
typeKeyFrame KEY_GetKeyFrame(typeCamera& PredictedPose, const std::vector<typePantoMapPoint>& LastFrameMapPoints);
bool KEY_IsKeyFrame(const typeKeyFrameInformation& Information);
void KEY_SetAsKeyFrame(typeKeyFrame& KeyFrame, std::vector<typePantoMapPoint>& GlobalMapPoints, 
        const std::vector<typeKeyFrame>& GlobalKeyFrames, const u64& ID, const DBoW3::Vocabulary* Vocabulary);
void KEY_InsertNewMapPoints(typeKeyFrame& KeyFrame1, typeKeyFrame& KeyFrame2, std::vector<typePantoMapPoint>& GlobalMapPoints);
void KEY_NonValidKeyFrame(void);
fp64 KEY_GetLocalMapMedianDepth(const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& LocalMapPoints);

#endif //__KEY_KEYFRAME_HPP_
