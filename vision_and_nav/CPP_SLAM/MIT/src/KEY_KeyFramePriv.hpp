#ifndef __KEY_KEYFRAMEPRIV_HPP_
#define __KEY_KEYFRAMEPRIV_HPP_
#include <FR_Frames.hpp>
#include <EP_CorrespondingPoints.hpp>
#include "../include/KEY_Keyframe.hpp"

typedef struct
{
    // Velocity change compared to last two frames
    fp64 VelocityChange;
    // Ratio of matches in local map
    fp64 LocalMapTrackingRatio;
    // fp64 TrajectoryCurvature;
    // fp64 FeatureDistributionQuality;
    fp64 AcumulatedDistanceTravelled;
    u64 NumFrames;
    bool BootStrapDataSolved;
}typeKeyFrameBootStrapData;

#define FUZZY_LINEARMEMBERSHIP(Low, High, x) ((x > High) ? 1.0f : ((x < Low) ? 0.0f : ((x - Low) / (High - Low))))

typedef std::pair<fp64, fp64> typeLinearMemberShipParameters;

typedef struct
{
    typeLinearMemberShipParameters VelocityParamameters;
    typeLinearMemberShipParameters TrackingRatioParameters;
    typeLinearMemberShipParameters AccumulatedDistanceParameters;
    fp64 Threshold;
}typeFuzzyKeyFrameInference;

void KEYPriv_SolveBootStrapData(void);

#endif // _KEY_KEYFRAMEPRIV_HPP_
