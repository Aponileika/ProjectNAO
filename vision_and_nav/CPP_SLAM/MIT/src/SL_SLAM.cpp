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
    const PantoClock::time_point SLAMStartTime = PantoClock::now();

    typeTimingStatistics KeyFrameTiming{};
    typeTimingStatistics FirstTrackingTiming{};
    typeTimingStatistics LocalMapCreationTiming{};
    typeTimingStatistics LocalMapMatchingTiming{};
    typeTimingStatistics SecondTrackingTiming{};
    typeTimingStatistics MapPointCreationTiming{};
    typeTimingStatistics LocalBATiming{};
    typeTimingStatistics LocalMapCullingTiming{};
    typeTimingStatistics KeyFrameDecisionTiming{};
    typeTimingStatistics LoopTiming{};

    bool Initialized = false;
    fp64 NumAcceptedKeyFrames = 0.0;
    fp64 NumTestedKeyFrames = 0.0;
    INIT_CreateInitData();
    typeInitReconstruction Reconstruction{};

    const PantoClock::time_point InitializationStartTime = PantoClock::now();

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

    const PantoClock::time_point InitializationEndTime = PantoClock::now();

    LG_Log(LogSeverity::DATA, "[SLAMTiming] Initialization = %.6f s\n",
            std::chrono::duration<fp64>(InitializationEndTime - InitializationStartTime).count());

    const PantoClock::time_point ThirdKeyFrameStartTime = PantoClock::now();

    typeKeyFrame ThirdKeyFrame = KEY_GetThirdKeyFrame(PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints);
    MAP_AppendKeyFrame(PantoSLAM.GlobalMap, ThirdKeyFrame);

    const u64 NumKeyFrames = static_cast<u64>(PantoSLAM.GlobalMap.KeyFrames.size());

    for(const typeKeyFrame& KeyFrame : PantoSLAM.GlobalMap.KeyFrames)
    {
        GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, KeyFrame, PantoSLAM.GlobalMap.MapPoints, NumKeyFrames);
    }

    LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error before tracking\n");
    MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);
    OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, nullptr);
    LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error after tracking\n");
    MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);

    const PantoClock::time_point ThirdKeyFrameEndTime = PantoClock::now();

    LG_Log(LogSeverity::DATA, "[SLAMTiming] Third keyframe setup = %.6f s\n",
            std::chrono::duration<fp64>(ThirdKeyFrameEndTime - ThirdKeyFrameStartTime).count());

    PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, PantoSLAM.GlobalMap.KeyFrames.back());
    PantoSLAM.PreviousFrameData.PreviousPreviousFramePose = PantoSLAM.GlobalMap.KeyFrames[1].Pose;
    PantoSLAM.PreviousFrameData.PreviousFramePose = PantoSLAM.GlobalMap.KeyFrames.back().Pose;
    PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFramePose.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFramePose.Pose);

    for(i32 i = 0; i < num_loops; i++)
    {
        const PantoClock::time_point LoopStartTime = PantoClock::now();

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Starting loop %d\n", i);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Logging all poses\n");
        MAP_LogGlobalMapPoses(PantoSLAM.GlobalMap);

        const PantoClock::time_point FrameStartTime = PantoClock::now();

        typeKeyFrame NewKeyFrame = KEY_GetKeyFrame(PantoSLAM.NextFramePosePrediction, PantoSLAM.PreviousFrameData.PreviousFrameMapPoints);

        const PantoClock::time_point FrameEndTime = PantoClock::now();

        const fp64 FrameTime =
            std::chrono::duration<fp64>(FrameEndTime - FrameStartTime).count();

        SL_AddTimingSample(KeyFrameTiming, FrameTime);

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d KEY_GetKeyFrame = %.6f s\n",
                i,
                FrameTime);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Inserting preliminary keyframe\n");
        MAP_InsertPreliminaryKeyFrame(PantoSLAM.GlobalMap, NewKeyFrame);

        const PantoClock::time_point FirstTrackingStartTime = PantoClock::now();

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Running first tracking optimization\n");
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, nullptr);

        const PantoClock::time_point FirstTrackingEndTime = PantoClock::now();

        const fp64 FirstTrackingTime =
            std::chrono::duration<fp64>(FirstTrackingEndTime - FirstTrackingStartTime).count();

        SL_AddTimingSample(FirstTrackingTiming, FirstTrackingTime);

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d first tracking = %.6f s\n", i,
                FirstTrackingTime);
        
        const PantoClock::time_point LocalMapCreationStartTime = PantoClock::now();

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Creating local map\n");

        PantoSLAM.LocalMap = MAP_CreateLocalMap(PantoSLAM.GlobalMap, NewKeyFrame);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Local Map size = %zu\n",PantoSLAM.LocalMap.KeyFrames.size()); 

        const PantoClock::time_point LocalMapCreationEndTime = PantoClock::now();

        const fp64 LocalMapCreationTime =
            std::chrono::duration<fp64>(LocalMapCreationEndTime - LocalMapCreationStartTime).count();

        SL_AddTimingSample(LocalMapCreationTiming, LocalMapCreationTime);

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d local map creation = %.6f s\n",
                i,
                LocalMapCreationTime);

        const PantoClock::time_point LocalMapMatchingStartTime = PantoClock::now();

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Matching local map points\n");
        const typeLocalMapInfo LocalMapInfo  = MAP_MatchMapPointLocalMap(PantoSLAM.LocalMap, NewKeyFrame);

        const PantoClock::time_point LocalMapMatchingEndTime = PantoClock::now();

        const fp64 LocalMapMatchingTime =
            std::chrono::duration<fp64>(LocalMapMatchingEndTime - LocalMapMatchingStartTime).count();

        SL_AddTimingSample(LocalMapMatchingTiming, LocalMapMatchingTime);

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d local map matching = %.6f s\n",
                i,
                LocalMapMatchingTime);

        const PantoClock::time_point SecondTrackingStartTime = PantoClock::now();

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Running second tracking optimization\n");
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error before tracking\n");
        MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);
        OP_BundleAdjust(PantoSLAM.GlobalMap, typeTracking, nullptr);
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error after tracking\n");
        MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);

        const PantoClock::time_point SecondTrackingEndTime = PantoClock::now();

        const fp64 SecondTrackingTime =
            std::chrono::duration<fp64>(SecondTrackingEndTime - SecondTrackingStartTime).count();

        SL_AddTimingSample(SecondTrackingTiming, SecondTrackingTime);

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d second tracking = %.6f s\n",
                i,
                SecondTrackingTime);

        const typePreviousFrameData PreviousFrameDataCopy = PantoSLAM.PreviousFrameData;

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, PantoSLAM.GlobalMap.KeyFrames.back());
        PantoSLAM.PreviousFrameData.PreviousPreviousFramePose = PantoSLAM.PreviousFrameData.PreviousFramePose;
        PantoSLAM.PreviousFrameData.PreviousFramePose = PantoSLAM.GlobalMap.KeyFrames.back().Pose;
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFramePose.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFramePose.Pose);

        const PantoClock::time_point KeyFrameEvaluationStartTime = PantoClock::now();

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Evaluating keyframe insertion\n");
        typeKeyFrameInformation KeyFrameInfo = SLPriv_GetKeyFrameInformation(PreviousFrameDataCopy, NewKeyFrame, LocalMapInfo);
        if(i >= PANTO_NUM_BOOTSTRAP_FRAMES)
        {
            NumTestedKeyFrames+=1.0;
        }

        if(KEY_IsKeyFrame(KeyFrameInfo))
        {
            if(i >= PANTO_NUM_BOOTSTRAP_FRAMES)
            {
                NumAcceptedKeyFrames+=1.0;
            }
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Current frame selected as keyframe\n");

            PantoSLAM.AccumulatedDistance = 0.0f;

            KEY_SetAsKeyFrame(PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints, PantoSLAM.GlobalMap.KeyFrames,
                    static_cast<u64>(PantoSLAM.GlobalMap.KeyFrames.size()) - 1, PantoSLAM.Vocabulary);

            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints, PantoSLAM.GlobalMap.KeyFrames.size());

            const PantoClock::time_point MapPointCreationStartTime = PantoClock::now();

            const u64 NumNewPoints = MAP_CreateNewMapPoints(PantoSLAM.GlobalMap, PantoSLAM.LocalMap, PantoSLAM.GlobalMap.KeyFrames.back());

            PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(
                    PantoSLAM.GlobalMap, PantoSLAM.GlobalMap.KeyFrames.back());

            const PantoClock::time_point MapPointCreationEndTime = PantoClock::now();

            const fp64 MapPointCreationTime =
                std::chrono::duration<fp64>(MapPointCreationEndTime - MapPointCreationStartTime).count();

            SL_AddTimingSample(MapPointCreationTiming, MapPointCreationTime);

            LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d map point creation = %.6f s\n",
                    i,
                    MapPointCreationTime);

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Created %llu new map points\n", static_cast<unsigned long long>(NumNewPoints));

            GRAPH_UpdateCovisibility(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.MapPoints, NumNewPoints, PantoSLAM.GlobalMap.KeyFrames.back().ID);

            const PantoClock::time_point LocalBAStartTime = PantoClock::now();

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Running local bundle adjustment\n");
            OP_BundleAdjust(PantoSLAM.GlobalMap, typePoseAndPoints, &PantoSLAM.LocalMap);

            const PantoClock::time_point LocalBAEndTime = PantoClock::now();

            const fp64 LocalBATime =
                std::chrono::duration<fp64>(LocalBAEndTime - LocalBAStartTime).count();

            SL_AddTimingSample(LocalBATiming, LocalBATime);

            LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d local bundle adjustment = %.6f s\n",
                    i,
                    LocalBATime);

            const PantoClock::time_point CullingStartTime = PantoClock::now();

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Culling local map\n");
            MAP_CullLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMap);

            const PantoClock::time_point CullingEndTime = PantoClock::now();

            const fp64 CullingTime =
                std::chrono::duration<fp64>(CullingEndTime - CullingStartTime).count();

            SL_AddTimingSample(LocalMapCullingTiming, CullingTime);

            LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d local map culling = %.6f s\n",
                    i,
                    CullingTime);

        }
        else
        {
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Removing preliminary keyframe\n");
            MAP_RemovePreliminaryKeyFrame(PantoSLAM.GlobalMap);
        }

        const PantoClock::time_point KeyFrameEvaluationEndTime = PantoClock::now();

        const fp64 KeyFrameDecisionTime =
            std::chrono::duration<fp64>(KeyFrameEvaluationEndTime - KeyFrameEvaluationStartTime).count();

        SL_AddTimingSample(KeyFrameDecisionTiming, KeyFrameDecisionTime);

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d keyframe decision/mapping = %.6f s\n",
                i,
                KeyFrameDecisionTime);

        const PantoClock::time_point LoopEndTime = PantoClock::now();

        const fp64 LoopTime =
            std::chrono::duration<fp64>(LoopEndTime - LoopStartTime).count();

        SL_AddTimingSample(LoopTiming, LoopTime);

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Loop %d total = %.6f s\n",
                i,
                LoopTime);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Finished loop %d\n", i);

        if((i % 5) == 0)
        {
            VIZ_WriteColmap(PantoSLAM.GlobalMap);
        }
    }

    const PantoClock::time_point SLAMEndTime = PantoClock::now();

    LG_Log(LogSeverity::DATA, "[SLAMTiming] Total SLAM time = %.6f s\n",
            std::chrono::duration<fp64>(SLAMEndTime - SLAMStartTime).count());

    LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] ========================================\n");
    LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] Per-loop timing statistics\n");

    SL_LogTimingStatistics("KEY_GetKeyFrame", KeyFrameTiming);
    SL_LogTimingStatistics("First tracking", FirstTrackingTiming);
    SL_LogTimingStatistics("Local map creation", LocalMapCreationTiming);
    SL_LogTimingStatistics("Local map matching", LocalMapMatchingTiming);
    SL_LogTimingStatistics("Second tracking", SecondTrackingTiming);
    SL_LogTimingStatistics("Map point creation", MapPointCreationTiming);
    SL_LogTimingStatistics("Local bundle adjustment", LocalBATiming);
    SL_LogTimingStatistics("Local map culling", LocalMapCullingTiming);
    SL_LogTimingStatistics("Keyframe decision/mapping", KeyFrameDecisionTiming);
    SL_LogTimingStatistics("Total loop", LoopTiming);

    LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] ========================================\n");

    LG_Log(LogSeverity::DBG, "[SLAMLoop] Num accepted / tested keyframes %lf \n", NumAcceptedKeyFrames / NumTestedKeyFrames);
    LG_Log(LogSeverity::DBG, "[SLAMLoop] Num tested %lf \n", NumTestedKeyFrames);
    LG_Log(LogSeverity::DBG, "[SLAMLoop] Num accepted %lf \n", NumAcceptedKeyFrames);

    VIZ_DestroyVisualization();
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

    LG_Log(
        LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] TimeStamps: PreviousPrevious = %f, Previous = %f, Current = %f\n",
        PreviousPreviousFramePose.TimeStamp,
        PreviousFramePose.TimeStamp,
        NewKeyFrame.Pose.TimeStamp);

    LG_Log(
        LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] Delta01 = %f, Delta12 = %f, LocalMapDepth = %f, LocalMapTrackedRatio = %f\n",
        Delta01,
        Delta12,
        LocalMapDepth,
        LocalMapInfo.TrackedRatio);

    const Eigen::Vector3d C0 = CM_GetCameraCenter(PreviousPreviousFramePose);
    const Eigen::Vector3d C1 = CM_GetCameraCenter(PreviousFramePose);
    const Eigen::Vector3d C2 = CM_GetCameraCenter(NewKeyFrame.Pose);

    LG_Log(
        LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] C0 = (%f, %f, %f), C1 = (%f, %f, %f), C2 = (%f, %f, %f)\n",
        C0[0], C0[1], C0[2],
        C1[0], C1[1], C1[2],
        C2[0], C2[1], C2[2]);

    const Eigen::Vector3d V0 = (C1 - C0) / Delta01;
    const Eigen::Vector3d V1 = (C2 - C1) / Delta12;

    LG_Log(
        LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] V0 = (%f, %f, %f), V1 = (%f, %f, %f)\n",
        V0[0], V0[1], V0[2],
        V1[0], V1[1], V1[2]);

    KeyFrameInfo.VelocityChange = (V1 - V0).norm() / LocalMapDepth;

    PantoSLAM.AccumulatedDistance += (C2 - C1).norm() / LocalMapDepth;

    KeyFrameInfo.AcumulatedDistanceTravelled = PantoSLAM.AccumulatedDistance;

    LG_Log( LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] Result: VelocityChange = %f, LocalMapTrackingRatio = %f, AcumulatedDistanceTravelled = %f\n",
        KeyFrameInfo.VelocityChange,
        KeyFrameInfo.LocalMapTrackingRatio,
        KeyFrameInfo.AcumulatedDistanceTravelled);

    return KeyFrameInfo;
}


void SL_AddTimingSample(typeTimingStatistics& Statistics, const fp64& Time)
{
    Statistics.Count++;
    Statistics.Sum += Time;
    Statistics.SumSquared += Time * Time;
}

void SL_LogTimingStatistics(const char* Name, const typeTimingStatistics& Statistics)
{
    if(Statistics.Count == 0)
    {
        LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] %s: no samples\n", Name);
        return;
    }

    const fp64 Mean = Statistics.Sum / static_cast<fp64>(Statistics.Count);
    const fp64 Variance = std::max<fp64>(
            0.0,
            Statistics.SumSquared / static_cast<fp64>(Statistics.Count) - Mean * Mean);
    const fp64 StandardDeviation = std::sqrt(Variance);

    LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] %s: mean = %.6f s, std dev = %.6f s, samples = %llu\n",
            Name,
            Mean,
            StandardDeviation,
            static_cast<unsigned long long>(Statistics.Count));
}
