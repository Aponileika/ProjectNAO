#ifndef __SL__SLAM_HPP_
#define __SL__SLAM_HPP_
#include "CM_Camera.hpp"
#include "Config.hpp"
#include "DBOW3_DeepBagofWords.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "FR_Frames.hpp"
#include "GRAPH_PantoGraph.hpp"
#include "GT_ReadGroundTruth.hpp"
#include "IMU_PreIntegration.hpp"
#include "INIT_InitializeSLAM.hpp"
#include "KEY_Keyframe.hpp"
#include "LG_Logging.hpp"
#include "MAP_Mapping.hpp"
#include "OP_BA.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include "PANTO_Utils.hpp"
#include "PT_PantoImagePoint.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"
#include "VIZ_Visualization.hpp"
#include <DBoW3/DBoW3.h>
#include <DBoW3/Vocabulary.h>
#include <Eigen/Dense>
#include <condition_variable>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unordered_set>
#include <utility>

typedef struct
{
    typeKeyFrame PreviousFrame;
    std::vector<typePantoMapPoint> PreviousFrameMapPoints;
    typeKeyFrame PreviousPreviousFrame;
} typePreviousFrameData;

typedef struct
{
#if !defined(CONFIG_IMU)
    typeCamera Pose;
#else
    typeNavigationState Pose;
#endif // CONFIG_IMU
} typePosePrediction;

typedef struct
{
    typePreviousFrameData PreviousFrameData;
    typeLocalMapTracking TrackingMap;
    typeKeyFrame NewFrame;
    fp64 AccumulatedDistance;
    typePosePrediction PosePrediction;

    typeKeyFrameQueue *KeyFrameQueue;
    typeGlobalMap *GlobalMap;
    typeCovisibilityGraph *CovisibilityGraph;
} typeTrackingData;

struct typeTimingStatistics
{
    u64 Count = 0;
    fp64 Sum = 0.0;
    fp64 SumSquared = 0.0;
};

typedef struct
{
    typeLocalMap LocalMap;
    std::unordered_set<u64> RecentMapPointIndexes;

    typeKeyFrameQueue *KeyFrameQueue;
    typeGlobalMap *GlobalMap;
    typeCovisibilityGraph *CovisibilityGraph;
    typeTimingStatistics *VisualizationUpdateTiming;
} typeLocalMapData;

typedef struct {
    typeGlobalMap *GlobalMap;
    typeCovisibilityGraph *CovisibilityGraph;
    std::unordered_set<u64> *RecentMapPointIndexes;
    const DBoW3::Vocabulary *Vocabulary;
    typeKeyFrameQueue *KeyFrameQueue;
    typePreviousFrameData PreviousFrameData;
    typePosePrediction NextFramePosePrediction;

    std::vector<Eigen::Vector3d> TrackingTrajectory;
    std::vector<fp64> TrackingTrajectoryTimeStamps;
    std::mutex TrackingTrajectoryMutex;
} typeSLAM;

void SL_InitSlam();
void SL_PantoSLAM(i32 num_loops);
void SL_AddTimingSample(typeTimingStatistics &Statistics, const fp64 &Time);
void SL_LogTimingStatistics(const char *Name,
        const typeTimingStatistics &Statistics);

#endif //__SL__SLAM_HPP_
