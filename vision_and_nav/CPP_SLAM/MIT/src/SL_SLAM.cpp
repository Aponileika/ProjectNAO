#include "../include/SL_SLAM.hpp"
#include "Config.hpp"
#include "OP_BA.hpp"
#include <cmath>

typeSLAM PantoSLAM;

typeKeyFrameInformation SLPriv_GetKeyFrameInformation(const typePreviousFrameData& PreviousFrameDataCopy, const typeKeyFrame& NewKeyFrame,
        const typeLocalMapInfo& LocalMapInfo);
void SLPriv_ResetMapAndTracking(void);
void SLPriv_InitializeMap(void);
static std::vector<typeGroundTruth> GroundTruth;

#if defined(CONFIG_IMU)
static bool SLPriv_IntegrateIMUUntil(const fp64 TimeStamp,
        typeIMUMeasurement* LastMeasurement = nullptr,
        typePreIntegration* KeyFramePreIntegration = nullptr)
{
    static typeIMUMeasurement BufferedMeasurement{};
    static typeIMUMeasurement LastIntegratedMeasurement{};
    static bool HasBufferedMeasurement = false;
    static bool HasLastIntegratedMeasurement = false;

    if(TimeStamp < 0.0)
    {
        return false;
    }

    if(HasLastIntegratedMeasurement &&
       LastIntegratedMeasurement.TimeStamp >= TimeStamp)
    {
        if(LastMeasurement != nullptr)
        {
            *LastMeasurement = LastIntegratedMeasurement;
        }
        return true;
    }

    while(true)
    {
        typeIMUMeasurement Measurement{};
        if(HasBufferedMeasurement)
        {
            Measurement = BufferedMeasurement;
            HasBufferedMeasurement = false;
        }
        else
        {
            Measurement = IMU_GetMeasurement();
        }

        // IMU_GetMeasurement returns a value-initialized measurement at EOF.
        if(Measurement.TimeStamp <= 0.0)
        {
            LG_Log(LogSeverity::ERROR, "[SLPriv_IntegrateIMUUntil] IMU data ended before timestamp %.9f\n",
                    TimeStamp);
            return false;
        }

        if(HasLastIntegratedMeasurement &&
           LastIntegratedMeasurement.TimeStamp < TimeStamp &&
           Measurement.TimeStamp > TimeStamp)
        {
            const fp64 Interval = Measurement.TimeStamp -
                LastIntegratedMeasurement.TimeStamp;

            if(Interval <= 0.0)
            {
                LG_Log(LogSeverity::ERROR,
                        "[SLPriv_IntegrateIMUUntil] Non-increasing IMU timestamp %.9f after %.9f\n",
                        Measurement.TimeStamp,
                        LastIntegratedMeasurement.TimeStamp);
                return false;
            }

            const fp64 Alpha =
                (TimeStamp - LastIntegratedMeasurement.TimeStamp) / Interval;
            typeIMUMeasurement InterpolatedMeasurement
            {
                .TimeStamp = TimeStamp,
                .AngularVelocity =
                    (1.0 - Alpha) * LastIntegratedMeasurement.AngularVelocity +
                    Alpha * Measurement.AngularVelocity,
                .Acceleration =
                    (1.0 - Alpha) * LastIntegratedMeasurement.Acceleration +
                    Alpha * Measurement.Acceleration
            };

            IMU_IngegrationStep(InterpolatedMeasurement);
            if(KeyFramePreIntegration != nullptr)
            {
                IMU_IngegrationStep(
                        InterpolatedMeasurement,
                        *KeyFramePreIntegration);
            }

            BufferedMeasurement = Measurement;
            HasBufferedMeasurement = true;
            LastIntegratedMeasurement = InterpolatedMeasurement;

            if(LastMeasurement != nullptr)
            {
                *LastMeasurement = InterpolatedMeasurement;
            }
            return true;
        }

        IMU_IngegrationStep(Measurement);
        if(KeyFramePreIntegration != nullptr)
        {
            IMU_IngegrationStep(Measurement, *KeyFramePreIntegration);
        }

        LastIntegratedMeasurement = Measurement;
        HasLastIntegratedMeasurement = true;

        if(Measurement.TimeStamp >= TimeStamp)
        {
            if(LastMeasurement != nullptr)
            {
                *LastMeasurement = Measurement;
            }

            return true;
        }
    }
}

static Eigen::Vector3d SLPriv_GetGroundTruthAcceleration(
        const std::size_t GroundTruthIndex)
{
    if(GroundTruth.size() < 2)
    {
        return Eigen::Vector3d::Zero();
    }

    const std::size_t FirstIndex =
        GroundTruthIndex == 0 ? 0 : GroundTruthIndex - 1;
    const std::size_t SecondIndex =
        GroundTruthIndex + 1 < GroundTruth.size() ?
        GroundTruthIndex + 1 : GroundTruthIndex;

    const fp64 DeltaTime = GroundTruth[SecondIndex].TimeStamp -
        GroundTruth[FirstIndex].TimeStamp;

    if(DeltaTime <= 0.0)
    {
        return Eigen::Vector3d::Zero();
    }

    return (GroundTruth[SecondIndex].Velocity -
            GroundTruth[FirstIndex].Velocity) / DeltaTime;
}
#endif

static bool SLPriv_GetNextGroundTruthFrame(typePantoFrame& Frame, typeGroundTruth& Measurement, std::size_t& GroundTruthIndex,
        const fp64 MinimumTimeStamp, const bool IntegrateIMU)
{
    constexpr fp64 TimeStampTolerance = 1e-6;

    while(GroundTruthIndex < GroundTruth.size() && GroundTruth[GroundTruthIndex].TimeStamp < MinimumTimeStamp)
    {
        GroundTruthIndex++;
    }

    while(GroundTruthIndex < GroundTruth.size())
    {
#if defined(CONFIG_IMU)
        if(IntegrateIMU)
        {
            const fp64 NextFrameTimeStamp = FR_PeekNextFrameTimeStamp();

            if(!SLPriv_IntegrateIMUUntil(NextFrameTimeStamp))
            {
                return false;
            }
        }
#else
        (void)IntegrateIMU;
#endif

        Frame = FR_GetFrame();

        if(Frame.TimeStamp < 0.0)
        {
            return false;
        }

        while(GroundTruthIndex < GroundTruth.size() &&
              GroundTruth[GroundTruthIndex].TimeStamp < Frame.TimeStamp - TimeStampTolerance)
        {
            GroundTruthIndex++;
        }

        if(GroundTruthIndex == GroundTruth.size())
        {
            return false;
        }

        if(std::abs(GroundTruth[GroundTruthIndex].TimeStamp - Frame.TimeStamp) <= TimeStampTolerance)
        {
            Measurement = GroundTruth[GroundTruthIndex];
            GroundTruthIndex++;
            return true;
        }
    }
    return false;
}

void SL_InitSlam()
{
    PantoSLAM.Vocabulary = DBOW3_GetVocabulary();
    SLPriv_ResetMapAndTracking();

    EP_InitCPointExtractor();
    FR_InitFrameGetter();
}

void SLPriv_ResetMapAndTracking(void)
{
    INIT_DestroyInitData();
    KEY_Reset();

    PantoSLAM.GlobalMap = typeGlobalMap{};
    PantoSLAM.LocalMap = typeLocalMap{};
    PantoSLAM.CovisibilityGraph = typeCovisibilityGraph{};
    PantoSLAM.LocalMapTracking = typeLocalMapTracking{};
    PantoSLAM.CurrentFrameID = PANTO_ID_NOT_SET;

#if !defined(CONFIG_IMU)
    PantoSLAM.NextFramePosePrediction = typeCamera{};
#else
    PantoSLAM.NextFramePosePrediction = typeNavigationState{};
#endif

    PantoSLAM.PreviousFrameData = typePreviousFrameData{};
    PantoSLAM.AccumulatedDistance = fp64{};
    PantoSLAM.RecentMapPointIndexes = typePantoVector<u64>{};
    PantoSLAM.TrackingTrajectory = std::vector<Eigen::Vector3d>{};
}

void SLPriv_InitializeMap(void)
{
    if(PANTO_GROUNDTRUTH_INIT)
    {
        if(GroundTruth.empty())
        {
            GroundTruth = GT_GetAllMeasurements();
        }

        if(GroundTruth.empty())
        {
            LG_Log(LogSeverity::ERROR,
                    "[SLPriv_InitializeMap] No ground-truth measurements available\n");
            return;
        }

        std::size_t GroundTruthIndex = 0;
        typePantoFrame FirstFrame{};
        typeGroundTruth FirstGT{};

        if(!SLPriv_GetNextGroundTruthFrame(
                    FirstFrame,
                    FirstGT,
                    GroundTruthIndex,
                    GroundTruth.front().TimeStamp,
                    false))
        {
            LG_Log(LogSeverity::ERROR,
                    "[SLPriv_InitializeMap] Could not find a first frame with a matching ground-truth timestamp\n");
            return;
        }

        typeNavigationState First(FirstGT.Orientation, FirstGT.Velocity,
                FirstGT.Position, FirstGT.GyroBias, FirstGT.AccelBias);

        bool InitialMapFound = false;
        typePantoFrame SecondFrame{};
        typeGroundTruth SecondGT{};

#if defined(CONFIG_IMU)
        // Advance the IMU reader to the first image and then make the first
        // ground-truth state the actual start of preintegration.
        IMU_NewNavigationStateArrival(First);

        typeIMUMeasurement FirstIMUMeasurement{};

        if(!SLPriv_IntegrateIMUUntil(
                    FirstFrame.TimeStamp, &FirstIMUMeasurement))
        {
            LG_Log(LogSeverity::ERROR,
                    "[SLPriv_InitializeMap] Could not synchronize IMU data with the first ground-truth frame\n");
            return;
        }

        const std::size_t FirstGroundTruthIndex = GroundTruthIndex - 1;
        const Eigen::Vector3d FirstWorldAcceleration =
            SLPriv_GetGroundTruthAcceleration(FirstGroundTruthIndex);

        if(!IMU_InitializeGravity(
                    First,
                    FirstIMUMeasurement,
                    FirstWorldAcceleration))
        {
            LG_Log(LogSeverity::ERROR,
                    "[SLPriv_InitializeMap] Could not initialize gravity from the first ground-truth frame\n");
            return;
        }

        IMU_NewNavigationStateArrival(First);
#endif

        while(SLPriv_GetNextGroundTruthFrame(
                    SecondFrame, SecondGT, GroundTruthIndex,
                    FirstFrame.TimeStamp,
                    true))
        {
            typeNavigationState Second(SecondGT.Orientation, SecondGT.Velocity,
                    SecondGT.Position, SecondGT.GyroBias, SecondGT.AccelBias);

            const fp64 Baseline =
                (SecondGT.Position - FirstGT.Position).norm();

            if(!std::isfinite(Baseline) ||
               Baseline < PANTO_MIN_INITIALIZATION_BASELINE_METERS)
            {
                LG_Log(LogSeverity::DATA,
                        "[SLAMGTInitialization] Candidate timestamp = %.9f; interval = %.6f s; baseline = %.6f/%.6f m; rejected before triangulation\n",
                        SecondFrame.TimeStamp,
                        SecondFrame.TimeStamp - FirstFrame.TimeStamp,
                        Baseline,
                        PANTO_MIN_INITIALIZATION_BASELINE_METERS);
                continue;
            }

#if defined(CONFIG_IMU)
            const typePreIntegrationData FirstToSecondPreIntegration =
                IMU_GetLatestPreIntegrationData();
#endif

            typeGlobalMap CandidateMap = MAP_InitializeFromGT(First, Second, FirstFrame, SecondFrame);

#if defined(CONFIG_IMU)
            CandidateMap.KeyFrames[0].PreviousKFID = PANTO_ID_NOT_SET;
            CandidateMap.KeyFrames[1].PreviousKFID = 0;
            CandidateMap.KeyFrames[1].PreIntegrationData =
                FirstToSecondPreIntegration;
#endif

            const std::size_t NumTriangulatedMapPoints = CandidateMap.MapPoints.active_size();

            LG_Log(LogSeverity::DATA,
                    "[SLAMGTInitialization] Candidate timestamp = %.9f; interval = %.6f s; baseline = %.6f/%.6f m; triangulated map points = %zu/%d\n",
                    SecondFrame.TimeStamp, SecondFrame.TimeStamp - FirstFrame.TimeStamp,
                    Baseline, PANTO_MIN_INITIALIZATION_BASELINE_METERS,
                    NumTriangulatedMapPoints, PANTO_MIN_NUMBER_INITIAL_MAP_POINTS);

            if(NumTriangulatedMapPoints < static_cast<std::size_t>(PANTO_MIN_NUMBER_INITIAL_MAP_POINTS))
            {
                continue;
            }

            PantoSLAM.GlobalMap = std::move(CandidateMap);
#if defined(CONFIG_IMU)
            // Only the accepted candidate ends the first keyframe interval.
            // Rejected candidates remain part of the same first-to-candidate
            // preintegration interval.
            IMU_NewNavigationStateArrival(Second);
#endif
            InitialMapFound = true;
            break;
        }

        if(!InitialMapFound)
        {
            LG_Log(LogSeverity::ERROR,
                    "[SLPriv_InitializeMap] No subsequent synchronized frame satisfied the %.3f m baseline and %d triangulated-map-point requirements\n",
                    PANTO_MIN_INITIALIZATION_BASELINE_METERS,
                    PANTO_MIN_NUMBER_INITIAL_MAP_POINTS);
            return;
        }

        PantoSLAM.RecentMapPointIndexes.reserve(PantoSLAM.GlobalMap.MapPoints.size());
        for(const typePantoMapPoint& MapPoint : PantoSLAM.GlobalMap.MapPoints)
        {
            PantoSLAM.RecentMapPointIndexes.push_back(MapPoint.ID);
        }

        OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypePoseAndPoints, {}, nullptr);

        for(const typeKeyFrame& KeyFrame : PantoSLAM.GlobalMap.KeyFrames)
        {
            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, KeyFrame, PantoSLAM.GlobalMap.MapPoints, KeyFrame.ID);
            PantoSLAM.TrackingTrajectory.push_back(CM_GetCameraCenter(KeyFrame.Camera));
        }

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, PantoSLAM.GlobalMap.KeyFrames.back());
        PantoSLAM.PreviousFrameData.PreviousPreviousFrame = PantoSLAM.GlobalMap.KeyFrames[0];
        PantoSLAM.PreviousFrameData.PreviousFrame = PantoSLAM.GlobalMap.KeyFrames.back();

#if defined(CONFIG_IMU)
        PantoSLAM.NextFramePosePrediction =
            PantoSLAM.PreviousFrameData.PreviousFrame.NavigationState;
#else
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFrame.Camera.Pose,
                PantoSLAM.PreviousFrameData.PreviousPreviousFrame.Camera.Pose);
#endif

    }
    else
    {
        INIT_CreateInitData();

        typeInitReconstruction Reconstruction{};

        while(!Reconstruction.Valid)
        {
            Reconstruction = INIT_ProcessNewFrame();
        }

        PantoSLAM.GlobalMap = INIT_ConstructInitialMap(Reconstruction);

        INIT_DestroyInitData();

        PantoSLAM.RecentMapPointIndexes.reserve(PantoSLAM.GlobalMap.MapPoints.size());
        for(const typePantoMapPoint& MapPoint : PantoSLAM.GlobalMap.MapPoints)
        {
            PantoSLAM.RecentMapPointIndexes.push_back(MapPoint.ID);
        }

        OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypePoseAndPoints, {}, nullptr);

        typeKeyFrame ThirdKeyFrame = KEY_GetThirdKeyFrame(PantoSLAM.GlobalMap.KeyFrames.back(), PantoSLAM.GlobalMap.MapPoints);
        MAP_AppendKeyFrame(PantoSLAM.GlobalMap, ThirdKeyFrame);

        for(const typeKeyFrame& KeyFrame : PantoSLAM.GlobalMap.KeyFrames)
        {
            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, KeyFrame, PantoSLAM.GlobalMap.MapPoints, KeyFrame.ID);
            PantoSLAM.TrackingTrajectory.push_back(CM_GetCameraCenter(KeyFrame.Camera));
        }

#if defined(DEBUG)
        MAP_LogGlobalMap(PantoSLAM.GlobalMap);
        MAP_LogGraphConsistency(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph);
        GRAPH_Log(PantoSLAM.CovisibilityGraph);
#endif

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error before tracking\n");
        MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);
        OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypeTracking, {},
                &PantoSLAM.GlobalMap.KeyFrames.back(),
                &PantoSLAM.GlobalMap.KeyFrames[1]);
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error after tracking\n");
        MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);
        OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypePoseAndPoints, {}, nullptr);

#if !defined(DEBUG)
        VIZ_WriteColmap(PantoSLAM.GlobalMap, PantoSLAM.TrackingTrajectory);
#endif

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, PantoSLAM.GlobalMap.KeyFrames.back());
        PantoSLAM.PreviousFrameData.PreviousPreviousFrame = PantoSLAM.GlobalMap.KeyFrames[1];
        PantoSLAM.PreviousFrameData.PreviousFrame = PantoSLAM.GlobalMap.KeyFrames.back();

#if !defined(CONFIG_IMU)
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFrame.Camera.Pose, PantoSLAM.PreviousFrameData.PreviousPreviousFrame.Camera.Pose);
#endif
    }

}

void SL_PantoSLAM(i32 num_loops)
{
    const PantoClock::time_point SLAMStartTime = PantoClock::now();

#if !defined(DEBUG)
    if(PANTO_USE_DATASET && !std::string(panto_gt_path).empty())
    {
        GroundTruth = GT_GetAllMeasurements();

        std::vector<Eigen::Vector3d> GroundTruthTrajectory;
        GroundTruthTrajectory.reserve(GroundTruth.size());
        std::vector<fp64> GroundTruthTimeStamps;
        GroundTruthTimeStamps.reserve(GroundTruth.size());

        for(const typeGroundTruth& Measurement : GroundTruth)
        {
            GroundTruthTrajectory.push_back(Measurement.Position);
            GroundTruthTimeStamps.push_back(Measurement.TimeStamp);
        }

        VIZ_SetGroundTruth(
                GroundTruthTrajectory,
                GroundTruthTimeStamps);

        LG_Log(LogSeverity::DATA,
                "[SLAMGroundTruthVisualization] Published %zu ground-truth positions\n",
                GroundTruthTrajectory.size());
    }
#endif

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

    fp64 NumAcceptedKeyFrames = 0.0;
    fp64 NumTestedKeyFrames = 0.0;

    const PantoClock::time_point InitializationStartTime = PantoClock::now();

    SLPriv_InitializeMap();

    const PantoClock::time_point InitializationEndTime = PantoClock::now();

    LG_Log(LogSeverity::DATA, "[SLAMTiming] Initialization = %.6f s\n",
            std::chrono::duration<fp64>(InitializationEndTime - InitializationStartTime).count());

#if defined(CONFIG_IMU)
    typePreIntegration PreIntegrationBetweenKF{};
    IMU_InitializePreIntegration(
            PreIntegrationBetweenKF,
            PantoSLAM.GlobalMap.KeyFrames.back().NavigationState);
#endif

    for(i32 i = 0; i < num_loops; i++)
    {
        const PantoClock::time_point LoopStartTime = PantoClock::now();

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Starting loop %d\n", i);

#if defined(CONFIG_IMU)
        IMU_NewNavigationStateArrival( PantoSLAM.PreviousFrameData.PreviousFrame.NavigationState);

        const fp64 NextFrameTimeStamp = FR_PeekNextFrameTimeStamp();
        if(!SLPriv_IntegrateIMUUntil(
                    NextFrameTimeStamp,
                    nullptr,
                    &PreIntegrationBetweenKF))
        {
            break;
        }

        PantoSLAM.NextFramePosePrediction = KEY_PredictPose(PantoSLAM.PreviousFrameData.PreviousFrame);
#endif

#if defined(DEBUG)
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Logging all poses\n");
        MAP_LogGlobalMapPoses(PantoSLAM.GlobalMap);
        for(const typePantoMapPoint& MapPoint :
                PantoSLAM.PreviousFrameData.PreviousFrameMapPoints)
        {
            if(!PantoSLAM.GlobalMap.MapPoints.contains(MapPoint.ID))
            {
                LG_Log( LogSeverity::ERROR,
                        "[SLAMLoop] PreviousFrameMapPoints contains removed MP %llu\n",
                        MapPoint.ID);

                assert(false);
            }
        }
#endif 

        const PantoClock::time_point FrameStartTime = PantoClock::now();

        typeKeyFrame CurrentFrame = KEY_GetKeyFrame(PantoSLAM.NextFramePosePrediction, PantoSLAM.PreviousFrameData.PreviousFrameMapPoints, PantoSLAM.GlobalMap.MapPoints);
#if defined(CONFIG_IMU)
        CurrentFrame.PreIntegrationData = IMU_GetLatestPreIntegrationData();
        CurrentFrame.PreviousKFID = PantoSLAM.GlobalMap.KeyFrames.back().ID;
#endif

        if(CurrentFrame.Camera.TimeStamp < 0.0f)
        {
            // Invalid timestamp means failure to read image
            break;
        }

        const PantoClock::time_point FrameEndTime = PantoClock::now();

        const fp64 FrameTime =
            std::chrono::duration<fp64>(FrameEndTime - FrameStartTime).count();

        SL_AddTimingSample(KeyFrameTiming, FrameTime);

        u64 NumMatchedMapPoints = 0;

        for(const typePantoImagePoint& ImagePoint : CurrentFrame.Points.ImagePoints)
        {
            if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
            {
                NumMatchedMapPoints++;
            }
        }

#if !defined(CONFIG_IMU)
        if(NumMatchedMapPoints <= PANTO_TRACKING_MIN_MATCHED_MAP_POINTS)
        {
            LG_Log(
                LogSeverity::ERROR,
                "[SLAMLoop] Tracking lost with %llu matched map points, resetting SLAM and restarting initialization\n",
                static_cast<unsigned long long>(NumMatchedMapPoints));

            SLPriv_ResetMapAndTracking();

            const PantoClock::time_point ReinitializationStartTime = PantoClock::now();
            SLPriv_InitializeMap();

#if defined(CONFIG_IMU)
            IMU_InitializePreIntegration(
                    PreIntegrationBetweenKF,
                    PantoSLAM.GlobalMap.KeyFrames.back().NavigationState);
#endif
            const PantoClock::time_point ReinitializationEndTime = PantoClock::now();

            LG_Log(LogSeverity::DATA, "[SLAMTiming] Reinitialization = %.6f s\n",
                std::chrono::duration<fp64>(
                    ReinitializationEndTime - ReinitializationStartTime).count());

            continue;
        }
#endif

        if(NumMatchedMapPoints > PANTO_TRACKING_MIN_MATCHED_MAP_POINTS)
        {
            const PantoClock::time_point FirstTrackingStartTime = PantoClock::now();
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Running first tracking optimization\n");
            const Eigen::Matrix3d RBefore = CurrentFrame.Camera.Pose.R;
            const Eigen::Vector3d tBefore = CurrentFrame.Camera.Pose.t;

            OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypeTracking, {},
                    &CurrentFrame,
                    &PantoSLAM.PreviousFrameData.PreviousFrame);

            /*
             * Compare optimized pose against predicted/input pose.
             */
            const Eigen::Matrix3d& RAfter = CurrentFrame.Camera.Pose.R;
            const Eigen::Vector3d& tAfter = CurrentFrame.Camera.Pose.t;
            /*
             * Relative rotation:
             *
             * R_delta = R_after * R_before^T
             */
            const Eigen::Matrix3d RDelta = RAfter * RBefore.transpose();

            const fp64 CosAngle = std::clamp( (RDelta.trace() - 1.0) * 0.5, -1.0, 1.0);

            const fp64 RotationChangeRadians = std::acos(CosAngle);

            const fp64 RotationChangeDegrees = RotationChangeRadians * 180.0 / M_PI;

            const Eigen::Vector3d TranslationDelta = tAfter - tBefore;

            LG_Log(
                    LogSeverity::DBG,
                    "[SLAMLoop] First tracking BA pose change: "
                    "R = %.6f deg, "
                    "dt = (%.6f, %.6f, %.6f), "
                    "|dt| = %.6f\n",
                    RotationChangeDegrees,
                    TranslationDelta.x(),
                    TranslationDelta.y(),
                    TranslationDelta.z(),
                    TranslationDelta.norm());
            const PantoClock::time_point FirstTrackingEndTime = PantoClock::now();


            const fp64 FirstTrackingTime =
                std::chrono::duration<fp64>(FirstTrackingEndTime - FirstTrackingStartTime).count();
            SL_AddTimingSample(FirstTrackingTiming, FirstTrackingTime);
        }

        const PantoClock::time_point LocalMapCreationStartTime = PantoClock::now();
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Creating local map\n");
        PantoSLAM.LocalMapTracking = MAP_CreateLocalMapTracking(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph, CurrentFrame);
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Local Map size = %zu\n",PantoSLAM.LocalMap.KeyFrameIDs.size());
        const PantoClock::time_point LocalMapCreationEndTime = PantoClock::now();

        const fp64 LocalMapCreationTime =
            std::chrono::duration<fp64>(LocalMapCreationEndTime - LocalMapCreationStartTime).count();

        SL_AddTimingSample(LocalMapCreationTiming, LocalMapCreationTime);

        const PantoClock::time_point LocalMapMatchingStartTime = PantoClock::now();
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Matching local map points\n");
        const typeLocalMapInfo LocalMapInfo  = MAP_MatchMapPointLocalMap(PantoSLAM.GlobalMap, PantoSLAM.LocalMapTracking, CurrentFrame);
        const PantoClock::time_point LocalMapMatchingEndTime = PantoClock::now();
        const fp64 LocalMapMatchingTime =
            std::chrono::duration<fp64>(LocalMapMatchingEndTime - LocalMapMatchingStartTime).count();
        SL_AddTimingSample(LocalMapMatchingTiming, LocalMapMatchingTime);

        MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);

        const PantoClock::time_point SecondTrackingStartTime = PantoClock::now();

        OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypeTracking, {},
                &CurrentFrame,
                &PantoSLAM.PreviousFrameData.PreviousFrame);

        MAP_LogGlobalMapProjectionErrors(PantoSLAM.GlobalMap);
        const PantoClock::time_point SecondTrackingEndTime = PantoClock::now();
        const fp64 SecondTrackingTime =
            std::chrono::duration<fp64>(SecondTrackingEndTime - SecondTrackingStartTime).count();
        SL_AddTimingSample(SecondTrackingTiming, SecondTrackingTime);

        const typePreviousFrameData PreviousFrameDataCopy = PantoSLAM.PreviousFrameData;
        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(PantoSLAM.GlobalMap, CurrentFrame);
        PantoSLAM.PreviousFrameData.PreviousPreviousFrame =
            PantoSLAM.PreviousFrameData.PreviousFrame;
        PantoSLAM.PreviousFrameData.PreviousFrame = CurrentFrame;

#if !defined(CONFIG_IMU)
        PantoSLAM.NextFramePosePrediction = CM_PredictPose(
                PantoSLAM.PreviousFrameData.PreviousFrame.Camera.Pose,
                PantoSLAM.PreviousFrameData.PreviousPreviousFrame.Camera.Pose);
#endif

        PantoSLAM.TrackingTrajectory.push_back(CM_GetCameraCenter(CurrentFrame.Camera));

        const PantoClock::time_point KeyFrameEvaluationStartTime = PantoClock::now();
        typeKeyFrameInformation KeyFrameInfo = SLPriv_GetKeyFrameInformation(PreviousFrameDataCopy, CurrentFrame, LocalMapInfo);

        if(i >= PANTO_NUM_BOOTSTRAP_FRAMES)
        {
            NumTestedKeyFrames+=1.0;
        }

        if(KEY_IsKeyFrame(KeyFrameInfo) || (i % 20 == 0))
        {
            if(i >= PANTO_NUM_BOOTSTRAP_FRAMES)
            {
                NumAcceptedKeyFrames+=1.0;
            }
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Current frame selected as keyframe\n");


            PantoSLAM.AccumulatedDistance = 0.0f;

#if defined(CONFIG_IMU)
            CurrentFrame.PreIntegrationData = PreIntegrationBetweenKF;
            CurrentFrame.PreviousKFID = PantoSLAM.GlobalMap.KeyFrames.back().ID;
#endif

#if defined(DEBUG)
            MAP_AssertMapPointObservations(PantoSLAM.GlobalMap);
#endif

            PantoSLAM.CurrentFrameID = MAP_AppendKeyFrame(PantoSLAM.GlobalMap, CurrentFrame);

            typeKeyFrame& CurrentKeyFrame = PantoSLAM.GlobalMap.KeyFrames[PantoSLAM.CurrentFrameID];

            KEY_SetAsKeyFrame(CurrentKeyFrame, PantoSLAM.GlobalMap.MapPoints, PantoSLAM.GlobalMap.KeyFrames, PantoSLAM.Vocabulary);

            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, CurrentKeyFrame, PantoSLAM.GlobalMap.MapPoints, PantoSLAM.CurrentFrameID);

#if defined(DEBUG)
            MAP_LogGlobalMap(PantoSLAM.GlobalMap);
            MAP_LogGraphConsistency(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph);
            GRAPH_Log(PantoSLAM.CovisibilityGraph);
#endif

            MAP_CullRecentMapPoints(PantoSLAM.RecentMapPointIndexes, PantoSLAM.GlobalMap);

#if defined(DEBUG)
            MAP_LogGlobalMap(PantoSLAM.GlobalMap);
            MAP_LogGraphConsistency(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph);
            GRAPH_Log(PantoSLAM.CovisibilityGraph);
#endif

            const PantoClock::time_point MapPointCreationStartTime = PantoClock::now();

            std::vector<u64> NewPointIndexes = MAP_CreateNewMapPoints(PantoSLAM.GlobalMap, CurrentKeyFrame, PantoSLAM.CovisibilityGraph, PantoSLAM.CurrentFrameID);

#if defined(DEBUG)
            MAP_LogGlobalMap(PantoSLAM.GlobalMap);
            GRAPH_Log(PantoSLAM.CovisibilityGraph);
#endif
            PantoSLAM.LocalMap = MAP_CreateLocalMap(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph, PantoSLAM.CurrentFrameID);

            for(const u64& MapPointID : NewPointIndexes)
            {
                PantoSLAM.RecentMapPointIndexes.push_back(MapPointID);
            }


            const PantoClock::time_point MapPointCreationEndTime = PantoClock::now();

            const fp64 MapPointCreationTime = std::chrono::duration<fp64>(MapPointCreationEndTime - MapPointCreationStartTime).count();

            SL_AddTimingSample(MapPointCreationTiming, MapPointCreationTime);

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Created %llu new map points\n", static_cast<u64>(NewPointIndexes.size()));

            GRAPH_UpdateCovisibility(PantoSLAM.CovisibilityGraph, PantoSLAM.GlobalMap.MapPoints, PantoSLAM.CurrentFrameID, NewPointIndexes);

#if defined(DEBUG)
            MAP_LogGlobalMap(PantoSLAM.GlobalMap);
            MAP_LogGraphConsistency(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph);
            GRAPH_Log(PantoSLAM.CovisibilityGraph);
#endif

            const PantoClock::time_point LocalBAStartTime = PantoClock::now();

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Running local bundle adjustment\n");

            OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypeLocal, PantoSLAM.LocalMap, nullptr);

            const PantoClock::time_point LocalBAEndTime = PantoClock::now();

            const fp64 LocalBATime =
                std::chrono::duration<fp64>(LocalBAEndTime - LocalBAStartTime).count();

            SL_AddTimingSample(LocalBATiming, LocalBATime);

            const PantoClock::time_point CullingStartTime = PantoClock::now();

            LG_Log(LogSeverity::DBG, "[SLAMLoop] Culling local map\n");

#if defined(DEBUG)
            MAP_AssertGraphEqual(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph);
#endif
            MAP_CullObservationEdges(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph);
#if !defined(CONFIG_IMU)
            // This causes problems for the preintegration steps, will be added back when SE2(3) is implemented,
            // it is possible to do it now, but better to wait until SE2(3) then it should be seamless.
            MAP_CullLocalMap(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph, PantoSLAM.CurrentFrameID);
#endif

#if defined(DEBUG)
            MAP_AssertGraphEqual(PantoSLAM.GlobalMap, PantoSLAM.CovisibilityGraph);
#endif

            const PantoClock::time_point CullingEndTime = PantoClock::now();

            const fp64 CullingTime =
                std::chrono::duration<fp64>(CullingEndTime - CullingStartTime).count();

            SL_AddTimingSample(LocalMapCullingTiming, CullingTime);

            PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(
                    PantoSLAM.GlobalMap, CurrentKeyFrame);

#if !defined(DEBUG)
            VIZ_WriteColmap(PantoSLAM.GlobalMap, PantoSLAM.TrackingTrajectory);
#endif

#if defined(CONFIG_IMU)
            // Local BA may have changed the accepted frame's state. Keep the
            // frame-to-frame starting state consistent with the optimized
            // keyframe before integrating the next frame.
            PantoSLAM.PreviousFrameData.PreviousFrame = CurrentKeyFrame;

            IMU_InitializePreIntegration(
                    PreIntegrationBetweenKF,
                    CurrentKeyFrame.NavigationState);
#endif
        }
        else
        {
            KEY_NonValidKeyFrame();
        }

        const PantoClock::time_point KeyFrameEvaluationEndTime = PantoClock::now();

        const fp64 KeyFrameDecisionTime =
            std::chrono::duration<fp64>(KeyFrameEvaluationEndTime - KeyFrameEvaluationStartTime).count();

        SL_AddTimingSample(KeyFrameDecisionTiming, KeyFrameDecisionTime);

        const PantoClock::time_point LoopEndTime = PantoClock::now();

        const fp64 LoopTime =
            std::chrono::duration<fp64>(LoopEndTime - LoopStartTime).count();

        SL_AddTimingSample(LoopTiming, LoopTime);

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Finished loop %d\n", i);
        if(i % 100 == 0)
        {
            OP_BundleAdjust(PantoSLAM.GlobalMap, OptimizationTypePose, {}, nullptr);
        }
    }


    const PantoClock::time_point SLAMEndTime = PantoClock::now();

    LG_Log(LogSeverity::DATA, "[SLAMTiming] Total SLAM time = %.6f s\n",
            std::chrono::duration<fp64>(SLAMEndTime - SLAMStartTime).count());

    MAP_LogMappingData();

    LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] ========================================\n");
    LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] Per-loop timing statistics\n");

    SL_LogTimingStatistics("KEY_GetKeyFrame", KeyFrameTiming);
    KEY_LogGetKeyFrameTimingStatistics();
    EP_LogGetDescriptorTimingStatistics();
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
    
    LG_Log(LogSeverity::DATA, "[SLAMKeyFrameSummary] ========================================\n");

    LG_Log(LogSeverity::DATA, "Num accepted / tested keyframes %lf \n", NumAcceptedKeyFrames / NumTestedKeyFrames);
    LG_Log(LogSeverity::DATA, "Num tested %lf \n", NumTestedKeyFrames);
    LG_Log(LogSeverity::DATA, "Num accepted %lf \n", NumAcceptedKeyFrames);

    KEY_LogIsKeyFrameStatistics();

    LG_Log(LogSeverity::DATA, "[SLAMKeyFrameSummary] ========================================\n");

#if !defined(DEBUG)
    while(true)
    {
        std::cout << "Visualization complete; press Enter to close it.\n";
        std::cin.get();
    }
    VIZ_StopViewer();
#endif
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

    const typeCamera PreviousPreviousFramePose =
        PreviousFrameDataCopy.PreviousPreviousFrame.Camera;
    const typeCamera PreviousFramePose =
        PreviousFrameDataCopy.PreviousFrame.Camera;

    const fp64 Delta01 = PreviousFramePose.TimeStamp - PreviousPreviousFramePose.TimeStamp;
    const fp64 Delta12 = NewKeyFrame.Camera.TimeStamp - PreviousFramePose.TimeStamp;

    LG_Log(
        LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] TimeStamps: PreviousPrevious = %f, Previous = %f, Current = %f\n",
        PreviousPreviousFramePose.TimeStamp,
        PreviousFramePose.TimeStamp,
        NewKeyFrame.Camera.TimeStamp);

    LG_Log(
        LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] Delta01 = %f, Delta12 = %f, LocalMapDepth = %f, LocalMapTrackedRatio = %f\n",
        Delta01,
        Delta12,
        LocalMapDepth,
        LocalMapInfo.TrackedRatio);

    const Eigen::Vector3d C0 = CM_GetCameraCenter(PreviousPreviousFramePose);
    const Eigen::Vector3d C1 = CM_GetCameraCenter(PreviousFramePose);
    const Eigen::Vector3d C2 = CM_GetCameraCenter(NewKeyFrame.Camera);

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
