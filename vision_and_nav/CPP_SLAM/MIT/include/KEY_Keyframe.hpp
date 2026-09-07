#ifndef KEY_KEYFRAME_HPP_
#define KEY_KEYFRAME_HPP_
#include "CArenaAlloc.h"
#include "PT_Types.hpp"
#include "DBOW3_DeepBagofWords.hpp"
#include "Config.hpp"
#include "FR_Frames.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include <PT_PantoMapPoints.hpp>
#include <DBoW3/DBoW3.h>
#include <DBoW3/BowVector.h>
#include <DBoW3/FeatureVector.h>
#include <DBoW3/Vocabulary.h>
#include <vector>
#include <Eigen/Dense>
#include <PT_PantoImagePoint.hpp>
#include <PT_PantoMapPoints.hpp>
#include <IMU_IMUReader.hpp>
#include <IMU_PreIntegration.hpp>
#include <CM_Camera.hpp>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>

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

struct typeKeyFrame
{
    typePantoKeypointFrame Points;
    DBoW3::BowVector BowVector;
    DBoW3::FeatureVector FeatureVector;
    typeCamera Camera;
#if defined(CONFIG_IMU)
    typeNavigationState NavigationState;
    typePreIntegrationData PreIntegrationData;
    u64 PreviousKFID = PANTO_ID_NOT_SET;
    // Complete state of the inertial reference keyframe in the map snapshot
    // used by tracking. Before an asynchronously queued reference has a map
    // ID, TrackingReferenceMappingGeneration identifies it instead.
    typeCameraPose TrackingReferencePose;
    typeNavigationState TrackingReferenceNavigationState;
    // IMU integration from the tracking reference keyframe to this frame.
    // This is distinct from PreIntegrationData, which is frame-to-frame for
    // ordinary tracking frames and keyframe-to-keyframe in the mapper copy.
    typePreIntegrationData TrackingReferencePreIntegrationData;
    u64 TrackingReferenceMappingGeneration = PANTO_ID_NOT_SET;
    bool HasTrackingReferenceState = false;
#endif
    // Queue generation assigned when this tracking frame is submitted to
    // local mapping. It lets tracking recognize the optimized global copy
    // without blocking for the mapper.
    u64 MappingGeneration = PANTO_ID_NOT_SET;
    u64 ID;
    std::string ImagePath;
};

typeKeyFrame KEY_CreateKeyFrame(const typeNavigationState& NavState, const typePantoFrame& Frame,
        const u64 ID);
typeKeyFrame KEY_GetThirdKeyFrame(typeKeyFrame& LastKeyFrame, typePantoVector<typePantoMapPoint>& GlobalMapPoints);
#if !defined(CONFIG_IMU)
typeKeyFrame KEY_GetKeyFrame(typeCamera& PredictedPose,
        std::vector<typePantoMapPoint>& LastFrameMapPoints);
#else
typeKeyFrame KEY_GetKeyFrame(typeNavigationState& PredictedNavigationState,
        std::vector<typePantoMapPoint>& LastFrameMapPoints);
#endif
void KEY_LogGetKeyFrameTimingStatistics(void);
void KEY_LogIsKeyFrameStatistics(void);
void KEY_Reset(void);
bool KEY_IsKeyFrame(const typeKeyFrameInformation& Information);
void KEY_SetAsKeyFrame(typeKeyFrame& KeyFrame, typePantoVector<typePantoMapPoint>& GlobalMapPoints, 
        const typePantoVector<typeKeyFrame>& GlobalKeyFrames, const DBoW3::Vocabulary* Vocabulary);
std::vector<typePantoMapPoint> KEY_InsertNewMapPoints(typeKeyFrame& KeyFrame1, typeKeyFrame& KeyFrame2, const u64 MapAge);
void KEY_NonValidKeyFrame(void);
fp64 KEY_GetLocalMapMedianDepth(const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& LocalMapPoints);
#if defined(CONFIG_IMU)
void KEY_IntegrationStep(void);
typeNavigationState KEY_PredictPose(typeKeyFrame& PreviousKeyFrame);

// Assumes that the optimized camera pose is correct. Velocity and biases are
// intentionally left unchanged until visual-inertial optimization is added.
void KEY_UpdateNavState(typeKeyFrame* KeyFrame);
#endif

#endif //__KEY_KEYFRAME_HPP_
