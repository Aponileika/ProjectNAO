#include "../include/SL_SLAM.hpp"

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
    PantoSLAM.CovisibilityGraph = {};
    PantoSLAM.NextFramePosePrediction = {};
    PantoSLAM.PreviousFrameData = {};
    PantoSLAM.Vocabulary.load(PANTO_VocabFilePath);
    PantoSLAM.AccumulatedDistance = {};

    EP_InitCPointExtractor();
    FR_InitFrameGetter();
}

void SL_SlamLoop(i32 num_loops)
{
    for(i32 i = 0; i < num_loops; i++)
    {
        typeKeyFrame NewKeyFrame = KEY_GetKeyFrame(PantoSLAM.NextFramePosePrediction, PantoSLAM.PreviousFrameData.PreviousFrameMapPoints);
        MAP_InsertPreliminaryKeyFrame(PantoSLAM.GlobalMap, NewKeyFrame);
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});
        
        PantoSLAM.LocalMap = MAP_CreateLocalMap(PantoSLAM.GlobalMap, NewKeyFrame);
        const fp64 TrackedRatio = MAP_MatchMapPointLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMap, NewKeyFrame);
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});

        const typePreviousFrameData PreviousFrameDataCopy = PantoSLAM.PreviousFrameData;

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, NewKeyFrame);
        PantoSLAM.PreviousFrameData.PreviousPreviousFramePose = PantoSLAM.PreviousFrameData.PreviousFramePose;
        PantoSLAM.PreviousFrameData.PreviousFramePose = NewKeyFrame.Pose;
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFramePose.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFramePose.Pose);

        typeKeyFrameInformation KeyFrameInfo = SLPriv_GetKeyFrameInformation(PreviousFrameDataCopy, NewKeyFrame, TrackedRatio);
        if(KEY_IsKeyFrame(KeyFrameInfo))
        {
            KEY_SetAsKeyFrame(PantoSLAM.GlobalMap.KeyFrames.back(), static_cast<u64>(PantoSLAM.GlobalMap.KeyFrames.size()) - 1, PantoSLAM.Vocabulary);
            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints);
            MAP_CreateNewMapPoints(PantoSLAM.GlobalMap, PantoSLAM.LocalMap, PantoSLAM.GlobalMap.KeyFrames.back());
            //Not implemented yet
            GRAPH_UpdateCovisibility(PantoSLAM.CovisibilityGraph, PantoSLAM.LocalMap, PantoSLAM.GlobalMap.MapPoints);
            OP_BundleAdjust(PantoSLAM.GlobalMap, typeLocal, PantoSLAM.LocalMap);
            MAP_CullLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMap);
        }
        else
        {
            MAP_RemovePreliminaryKeyFrame(PantoSLAM.GlobalMap);
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
