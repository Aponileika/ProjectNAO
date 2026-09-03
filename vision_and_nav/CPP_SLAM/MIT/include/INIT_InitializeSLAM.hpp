/*
 * Multi-frame monocular initialization based on:
 *
 * H. Dou, B. Liu, Y. Jia, and C. Wang,
 * "Monocular Initialization for Real-Time Feature-Based SLAM
 *  in Dynamic Environments with Multiple Frames,"
 * Sensors, vol. 25, no. 8, 2404, 2025.
 * DOI: 10.3390/s25082404
 *
 * This is an independent implementation based on the
 * algorithm described in the paper.
 */
#ifndef INIT_INITIALIZESLAM_HPP_
#define INIT_INITIALIZESLAM_HPP_
#include <vector>
#include <ranges>
#include <thread>
#include <functional>
#include <memory>
#include <DBoW3/DBoW3.h>
#include <DBoW3/BowVector.h>
#include <DBoW3/FeatureVector.h>
#include "MAP_Mapping.hpp"
#include "CM_Camera.hpp"
#include "CArenaAlloc.h"
#include "Config.hpp"
#include "PT_PantoImagePoint.hpp"
#include "FR_Frames.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "DBOW3_DeepBagofWords.hpp"
#include "PANTO_Utils.hpp"
#include "KEY_Keyframe.hpp"
#include "PT_Types.hpp"
#include "PT_PantoImagePoint.hpp"
#include "PANTOVEC_PantoVector.hpp"

typedef struct
{
    Eigen::Vector4d Point4D;
    std::pair<u64, u64> InitImagePointID;
}typeInitMapPoint;

typedef struct
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    u64 NumPointsInFront;
    std::vector<typeInitMapPoint> MapPoints;
    std::pair<u64, u64> ChosenInitFrameID;
    bool Valid;
}typeInitReconstruction;

typedef struct
{
    std::vector<u64> FeatureTrack;
    u64 InlierCount;
    u64 OutlierCount;
}typeFeatureTrack;

typedef struct
{
    Eigen::Vector2d Point;
    typeDescriptor Descriptor;
    u64 ID;
    u64 FeatureTrackID;
}typeInitImagePoint;

typedef struct
{
    std::vector<typeInitImagePoint> ImagePoints;
    DBoW3::BowVector BoWVector;
    DBoW3::FeatureVector FeatureVector;
    u64 ID;
    fp64 TimeStamp;
    std::string ImagePath;
}typeInitFrame;

typedef struct
{
    std::vector<typeInitFrame> InitFrames;
    std::vector<typeFeatureTrack> FeatureTracks;
    bool EnoughStationaryPointsForInit;
}typePantoInitData;

void INIT_CreateInitData(void);
typeInitReconstruction INIT_ProcessNewFrame(void);
void INIT_DestroyInitData(void);
typeGlobalMap INIT_ConstructInitialMap(typeInitReconstruction Reconstruction);

#endif // INIT_INITIALIZESLAM_HPP_
