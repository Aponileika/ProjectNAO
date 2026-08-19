#include "../include/SL_SLAM.hpp"

typeSLAM PantoSLAM;

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
    typeInitReconstruction Reconstruction{};

    while(!Initialized)
    {
        Reconstruction = INIT_ProcessNewFrame();
        if(Reconstruction.Valid)
        {
            Initialized = true;
        }
    }

    PantoSLAM.GlobalMap = INIT_ConstructInitialMap(Reconstruction);

    INIT_DestroyInitData();

    typeKeyFrame ThirdKeyFrame = KEY_GetThirdKeyFrame(PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints);
    MAP_AppendKeyFrame(PantoSLAM.GlobalMap, ThirdKeyFrame);

    OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});

    PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, PantoSLAM.GlobalMap.KeyFrames.back());
    PantoSLAM.PreviousFrameData.PreviousPreviousFramePose = PantoSLAM.GlobalMap.KeyFrames[1].Pose;
    PantoSLAM.PreviousFrameData.PreviousFramePose = PantoSLAM.GlobalMap.KeyFrames.back().Pose;
    PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFramePose.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFramePose.Pose);

    for(i32 i = 0; i < num_loops; i++)
    {
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Starting loop %d\n", i);

        typeKeyFrame NewKeyFrame = KEY_GetKeyFrame(PantoSLAM.NextFramePosePrediction, PantoSLAM.PreviousFrameData.PreviousFrameMapPoints);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Inserting preliminary keyframe\n");
        MAP_InsertPreliminaryKeyFrame(PantoSLAM.GlobalMap, NewKeyFrame);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Running first tracking optimization\n");
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});
        
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Creating local map\n");
        PantoSLAM.LocalMap = MAP_CreateLocalMap(PantoSLAM.GlobalMap, NewKeyFrame);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Matching local map points\n");
        const typeLocalMapInfo LocalMapInfo  = MAP_MatchMapPointLocalMap(PantoSLAM.LocalMap, NewKeyFrame);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Running second tracking optimization\n");
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, {});

        const typePreviousFrameData PreviousFrameDataCopy = PantoSLAM.PreviousFrameData;

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, PantoSLAM.GlobalMap.KeyFrames.back());
        PantoSLAM.PreviousFrameData.PreviousPreviousFramePose = PantoSLAM.PreviousFrameData.PreviousFramePose;
        PantoSLAM.PreviousFrameData.PreviousFramePose = PantoSLAM.GlobalMap.KeyFrames.back().Pose;
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFramePose.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFramePose.Pose);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Evaluating keyframe insertion\n");
        typeKeyFrameInformation KeyFrameInfo = SLPriv_GetKeyFrameInformation(PreviousFrameDataCopy, NewKeyFrame, LocalMapInfo);

        if(KEY_IsKeyFrame(KeyFrameInfo))
        {
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Current frame selected as keyframe\n");

            PantoSLAM.AccumulatedDistance = 0.0f;

            KEY_SetAsKeyFrame(PantoSLAM.GlobalMap.KeyFrames.back(), static_cast<u64>(PantoSLAM.GlobalMap.KeyFrames.size()) - 1, PantoSLAM.Vocabulary);
            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints);

            const u64 NumNewPoints = MAP_CreateNewMapPoints(PantoSLAM.GlobalMap, PantoSLAM.LocalMap, PantoSLAM.GlobalMap.KeyFrames.back());

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Created %llu new map points\n", static_cast<unsigned long long>(NumNewPoints));

            GRAPH_UpdateCovisibility(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.MapPoints, NumNewPoints, PantoSLAM.GlobalMap.KeyFrames.back().ID);

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Running local bundle adjustment\n");
            OP_BundleAdjust(PantoSLAM.GlobalMap, typeLocal, PantoSLAM.LocalMap);

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Culling local map\n");
            MAP_CullLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMap);

            // Loop closure + Full bundle adjust
        }
        else
        {
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Removing preliminary keyframe\n");
            MAP_RemovePreliminaryKeyFrame(PantoSLAM.GlobalMap);
        }

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Finished loop %d\n", i);
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
