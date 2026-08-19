#include "../include/SL_SLAM.hpp"

typeSLAM PantoSLAM;

static cv::Mat __SL_GetNextFrame(u64 num_views);
void __SL_PrintSlam();
static void __SL_SlamStart();
void __SL_SlamLoopBundle();
PointPair2D __SL_SlamLoopPnP();
typeKeyFrameInformation SLPriv_GetKeyFrameInformation(const typePreviousFrameData& PreviousFrameDataCopy, const typeKeyFrame& NewKeyFrame,
        const typeLocalMapInfo& LocalMapInfo);

void SL_InitSlam()
{
    PantoSLAM.GlobalMap = {};
    PantoSLAM.LocalMap = {};
    PantoSLAM.CovisibilityGraph = {};
    PantoSLAM.NextFramePosePrediction = {};
    PantoSLAM.PreviousFrameData = {};
    PantoSLAM.Vocabulary = DBOW3_GetVocabulary();
    PantoSLAM.AccumulatedDistance = {};

    EP_InitCPointExtractor();
    FR_InitFrameGetter();
}

void SL_PantoSLAM(i32 num_loops)
{
    bool Initialized = false;
    INIT_CreateInitData();
    while(!Initialized)
    {
        typeInitReconstruction Reconstruction = INIT_ProcessNewFrame();
        if(Reconstruction.Valid)
        {
            Initialized = true;
        }
    }

    for(i32 i = 0; i < num_loops; i++)
    {
        typeKeyFrame NewKeyFrame = KEY_GetKeyFrame(PantoSLAM.NextFramePosePrediction, PantoSLAM.PreviousFrameData.PreviousFrameMapPoints);
        MAP_InsertPreliminaryKeyFrame(PantoSLAM.GlobalMap, NewKeyFrame);
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});
        
        PantoSLAM.LocalMap = MAP_CreateLocalMap(PantoSLAM.GlobalMap, NewKeyFrame);

        const typeLocalMapInfo LocalMapInfo  = MAP_MatchMapPointLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMap, NewKeyFrame);

        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});

        const typePreviousFrameData PreviousFrameDataCopy = PantoSLAM.PreviousFrameData;

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, NewKeyFrame);
        PantoSLAM.PreviousFrameData.PreviousPreviousFramePose = PantoSLAM.PreviousFrameData.PreviousFramePose;
        PantoSLAM.PreviousFrameData.PreviousFramePose = NewKeyFrame.Pose;
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFramePose.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFramePose.Pose);

        typeKeyFrameInformation KeyFrameInfo = SLPriv_GetKeyFrameInformation(PreviousFrameDataCopy, NewKeyFrame, LocalMapInfo);

        if(KEY_IsKeyFrame(KeyFrameInfo))
        {
            PantoSLAM.AccumulatedDistance = 0.0f;

            KEY_SetAsKeyFrame(PantoSLAM.GlobalMap.KeyFrames.back(), static_cast<u64>(PantoSLAM.GlobalMap.KeyFrames.size()) - 1, PantoSLAM.Vocabulary);
            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints);
            const u64 NumNewPoints = MAP_CreateNewMapPoints(PantoSLAM.GlobalMap, PantoSLAM.LocalMap, PantoSLAM.GlobalMap.KeyFrames.back());
            GRAPH_UpdateCovisibility(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.MapPoints, NumNewPoints);
            OP_BundleAdjust(PantoSLAM.GlobalMap, typeLocal, PantoSLAM.LocalMap);
            MAP_CullLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMap);
            // Loop closure + Full bundle adjust
        }
        else
        {
            MAP_RemovePreliminaryKeyFrame(PantoSLAM.GlobalMap);
        }
    }
}

typeKeyFrameInformation SLPriv_GetKeyFrameInformation(const typePreviousFrameData& PreviousFrameDataCopy, const typeKeyFrame& NewKeyFrame,
        const typeLocalMapInfo& LocalMapInfo)
{
    // Order oldest to newest 0->1->2
    typeKeyFrameInformation KeyFrameInfo = 
    {
        .VelocityChange = {},
        .LocalMapTrackingRatio = LocalMapInfo.TrackedRatio,
        .AcumulatedDistanceTravelled = {}
    };

    const fp64 LocalMapDepth = LocalMapInfo.MedianDepth;

    const typeCamera PreviousPreviousFramePose = PreviousFrameDataCopy.PreviousPreviousFramePose;
    const typeCamera PreviousFramePose = PreviousFrameDataCopy.PreviousFramePose;

    const fp64 Delta01 = PreviousFramePose.TimeStamp - PreviousPreviousFramePose.TimeStamp;
    const fp64 Delta12 = NewKeyFrame.Pose.TimeStamp - PreviousFramePose.TimeStamp;

    const Eigen::Vector3d C0 = CM_GetCameraCenter(PreviousPreviousFramePose);
    const Eigen::Vector3d C1 = CM_GetCameraCenter(PreviousFramePose);
    const Eigen::Vector3d C2 = CM_GetCameraCenter(NewKeyFrame.Pose);

    const Eigen::Vector3d V0 = (C1 - C0) / Delta01;
    const Eigen::Vector3d V1 = (C2 - C1) / Delta12;

    KeyFrameInfo.VelocityChange = (V1 - V0).norm() / LocalMapDepth;

    PantoSLAM.AccumulatedDistance += (C2 - C1).norm() / LocalMapDepth;

    KeyFrameInfo.AcumulatedDistanceTravelled = PantoSLAM.AccumulatedDistance;

    return KeyFrameInfo;
}
