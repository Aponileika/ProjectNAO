#include "../include/SL_SLAM.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "FR_Frames.hpp"
#include "OB_Observations.hpp"
#include "OP_BA.hpp"
#include "VIZ_Visualization.hpp"
#include "VW_Views.hpp"
#include <chrono>
#include <utility>

typeSLAM PantoSLAM;

static cv::Mat __SL_GetNextFrame(u64 num_views);
void __SL_PrintSlam();
static void __SL_SlamStart();
void __SL_SlamLoopBundle();
PointPair2D __SL_SlamLoopPnP();
typeKeyFrameInformation SLPriv_GetKeyFrameInformation(const typePreviousFrameData& PreviousFrameDataCopy, const typeKeyFrame& NewKeyFrame,
        fp64 TrackedRatio);

void SL_InitSlam()
{
    PantoSLAM.GlobalMap = {};
    PantoSLAM.LocalMap = {};
    EP_InitCPointExtractor();
    FR_InitFrameGetter();
}

void SL_SlamLoop(i32 num_loops)
{
    for(i32 i = 0; i < num_loops; i++)
    {
        typeKeyFrame NewKeyFrame = KEY_GetKeyFrame(PantoSLAM.NextFramePosePrediction, PantoSLAM.PreviousFrameData.PreviousFrameMapPoints);
        MAP_InsertKeyFrame(PantoSLAM.GlobalMap, NewKeyFrame);
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});
        
        PantoSLAM.LocalMap = MAP_CreateLocalMap(PantoSLAM.GlobalMap, NewKeyFrame);
        const fp64 TrackedRatio = MAP_SearchLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMap, NewKeyFrame);
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});

        const typePreviousFrameData PreviousFrameDataCopy = PantoSLAM.PreviousFrameData;

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, NewKeyFrame);
        PantoSLAM.PreviousFrameData.PreviousPreviousFramePose = PantoSLAM.PreviousFrameData.PreviousFramePose;
        PantoSLAM.PreviousFrameData.PreviousFramePose = NewKeyFrame.Pose;
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFramePose.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFramePose.Pose);

        typeKeyFrameInformation KeyFrameInfo = SLPriv_GetKeyFrameInformation(PreviousFrameDataCopy, NewKeyFrame, TrackedRatio);
        if(KEY_IsKeyFrame(KeyFrameInfo))
        {
            KEY_SetAsKeyFrame(NewKeyFrame, static_cast<u64>(PantoSLAM.GlobalMap.KeyFrames.size()) - 1);
            MAP_CreateNewMapPoints(PantoSLAM.GlobalMap, PantoSLAM.LocalMap);
            OP_BundleAdjust(PantoSLAM.GlobalMap, typeLocal, PantoSLAM.LocalMap);
            KEY_CullLocalMap(PantoSLAM.LocalMap);
        }
    }
}

typeKeyFrameInformation SLPriv_GetKeyFrameInformation(const typePreviousFrameData& PreviousFrameDataCopy, const typeKeyFrame& NewKeyFrame,
        fp64 TrackedRatio)
{
    typeKeyFrameInformation KeyFrameInfo = 
    {
        .VelocityChange = {},
        .LocalMapTrackingRatio = TrackedRatio,
        .DistanceTravelled = {}
    };
    // Order oldest to newest 0->1->2
    // Calculate velocity change
    const typeCamera PreviousPreviousFramePose = PreviousFrameDataCopy.PreviousPreviousFramePose;
    const typeCamera PreviousFramePose = PreviousFrameDataCopy.PreviousFramePose;

    const fp64 Delta01 = PreviousPreviousFramePose.TimeStamp - PreviousFramePose.TimeStamp;
    const fp64 Delta12 = PreviousFramePose.TimeStamp - NewKeyFrame.Pose.TimeStamp;

    const Eigen::Vector3d C0 = CM_GetCameraCenter(PreviousPreviousFramePose);
    const Eigen::Vector3d C1 = CM_GetCameraCenter(PreviousFramePose);
    const Eigen::Vector3d C2 = CM_GetCameraCenter(NewKeyFrame.Pose);

    const Eigen::Vector3d V0 = (C1 - C0) / Delta01;
    const Eigen::Vector3d V1 = (C2 - C1) / Delta12;

    KeyFrameInfo.VelocityChange = (V1 - V0).norm();

    PantoSLAM.AccumulatedDistance += abs((C2 - C1).norm());

    KeyFrameInfo.DistanceTravelled = PantoSLAM.AccumulatedDistance;

    return KeyFrameInfo;
}
