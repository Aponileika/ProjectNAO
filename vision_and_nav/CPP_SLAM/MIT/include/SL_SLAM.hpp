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
#include "KEY_Keyframe.hpp"
#include "PT_PantoImagePoint.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"
#include "CM_Camera.hpp"
#include "FR_Frames.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "Config.hpp"
#include "GRAPH_PantoGraph.hpp"

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
    typeCamera NextFramePosePrediction;
    typePreviousFrameData PreviousFrameData;
    DBoW3::Vocabulary Vocabulary;
    fp64 AccumulatedDistance;
}typeSLAM;

void SL_InitSlam();
void SL_SlamLoop(i32 num_loops);

#endif //__SL__SLAM_HPP_
