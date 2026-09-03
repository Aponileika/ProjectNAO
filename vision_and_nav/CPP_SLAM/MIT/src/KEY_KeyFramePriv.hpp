#ifndef KEY_KEYFRAMEPRIV_HPP_
#define KEY_KEYFRAMEPRIV_HPP_
#include <FR_Frames.hpp>
#include <EP_CorrespondingPoints.hpp>
#include "../include/KEY_Keyframe.hpp"
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
    u64 NumFrames;
    bool BootStrapDataSolved;
}typeKeyFrameBootStrapData;

inline fp64 FUZZY_LINEAR_INCREASING_MEMBERSHIP(fp64 Low, fp64 High, fp64 x) 
{
    assert(Low < High);
    return (x > High) ? 1.0 : ((x < Low) ? 0.0 : ((x - Low) / (High - Low)));
}

#define FUZZY_LINEAR_DECREASING_MEMBERSHIP(Low, High, x) (1.0 - FUZZY_LINEAR_INCREASING_MEMBERSHIP(Low, High, x))
#define FUZZY_UNIONRULEINFERENCE(...) (std::max({__VA_ARGS__}))
#define FUZZY_INTERSECTIONRULEINFERENCE(...) (std::min({__VA_ARGS__}))

typedef std::pair<fp64, fp64> typeLinearMemberShipParameters;

typedef struct
{
    typeLinearMemberShipParameters VelocityParamameters;
    typeLinearMemberShipParameters TrackingRatioParameters;
    typeLinearMemberShipParameters AccumulatedDistanceParameters;
    fp64 MaxRuleThreshold;
    fp64 SpatialTrackingThreshold;
}typeFuzzyKeyFrameInference;

void KEYPriv_SolveBootStrapData(void);
bool KEYPriv_IsKeyFrame(const typeKeyFrameInformation& KeyFrameInformation);

#endif // _KEY_KEYFRAMEPRIV_HPP_
