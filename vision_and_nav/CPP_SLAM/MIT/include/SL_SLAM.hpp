#ifndef __SL__SLAM_HPP_
#define __SL__SLAM_HPP_
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <utility>
#include <DBoW3/DBoW3.h>
#include <DBoW3/Vocabulary.h>
#include <stdio.h>
#include "DBOW3_DeepBagofWords.hpp"
#include "LG_Logging.hpp"
#include "OP_BA.hpp"
#include "VIZ_Visualization.hpp"
#include "PANTO_Utils.hpp"
#include "MAP_Mapping.hpp"
#include "KEY_KeyFrame.hpp"
#include "PT_PantoImagePoint.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"
#include "CM_Camera.hpp"
#include "FR_Frames.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "Config.hpp"
#include "GRAPH_PantoGraph.hpp"
#include "INIT_InitializeSLAM.hpp"
#include "PANTOVEC_PantoVector.hpp"

typedef struct
{
    typeCamera PreviousFramePose;
    std::vector<typePantoMapPoint> PreviousFrameMapPoints;
    typeCamera PreviousPreviousFramePose;
}typePreviousFrameData;

typedef struct 
{
    typeGlobalMap GlobalMap;
    typeLocalMap LocalMap;
    typeCovisibilityGraph CovisibilityGraph;

    typeLocalMapTracking LocalMapTracking;
    u64 CurrentFrameID;

    typeCamera NextFramePosePrediction;
    typePreviousFrameData PreviousFrameData;

    fp64 AccumulatedDistance;
    DBoW3::Vocabulary* Vocabulary;
    typePantoVector<u64> RecentMapPointIndexes;
}typeSLAM;

struct typeTimingStatistics
{
    u64 Count = 0;
    fp64 Sum = 0.0;
    fp64 SumSquared = 0.0;
};

void SL_InitSlam();
void SL_PantoSLAM(i32 num_loops);
void SL_AddTimingSample(typeTimingStatistics& Statistics, const fp64& Time);
void SL_LogTimingStatistics(const char* Name, const typeTimingStatistics& Statistics);

#endif //__SL__SLAM_HPP_
