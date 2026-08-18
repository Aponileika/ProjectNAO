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
#include <DBoW3.h>
#include <ranges>
#include "CArenaAlloc.h"
#include "Config.hpp"
#include "PT_PantoImagePoint.hpp"
#include "PT_Types.hpp"
#include "FR_Frames.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "DBOW3_DeepBagofWords.hpp"
#include "PANTO_Utils.hpp"

// set to PANTO_FEATURE_TRACK_NOT_OBSERVED if no track exists
typedef struct
{
    std::vector<i64> FeatureTrack;
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
    DBoW3::FeatureVector FeatureVector;
    u64 ID;
    fp64 TimeStamp;
}typeInitFrame;

typedef struct
{
    std::vector<typeInitFrame> InitFrames;
    std::vector<typeFeatureTrack> FeatureTracks;
    bool EnoughStationaryPointsForInit;
}typePantoInitData;

void INIT_CreateInitStruct(void);
void INIT_ProcessNewFrame(void);
std::pair<typeInitFrame, typeInitFrame> INIT_Initialize(void);
void INIT_DestroyInitStruct(void);

#endif // INIT_INITIALIZESLAM_HPP_
