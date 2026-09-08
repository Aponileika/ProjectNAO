#include "../include/SL_SLAM.hpp"
#include "Config.hpp"
#include "GRAPH_PantoGraph.hpp"
#include "IMU_PreIntegration.hpp"
#include "KEY_Keyframe.hpp"
#include "MAP_Mapping.hpp"
#include "OP_BA.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <thread>

typeSLAM PantoSLAM;

typeKeyFrameInformation SLPriv_GetKeyFrameInformation(const typePreviousFrameData& PreviousFrameDataCopy, const typeKeyFrame& NewKeyFrame,
        const typeLocalMapInfo& LocalMapInfo, fp64& AccumulatedDistance);
void SLPriv_ResetMapAndTracking(void);
void SLPriv_InitializeMap(void);
enum class typeTrackingTimingStage : std::size_t
{
    FrameQueuePeek,
    RealtimePacingSleep,
    SkipFrame,
    IterationTotal,
    MapSnapshotTransaction,
    CreateTrackingMap,
    GetPreviousFrameMapPoints,
    UpdateCorrectedTrajectory,
    IMUStateArrival,
    IntegrateIMU,
    PredictPose,
    GetKeyFrame,
    CountMatchedMapPoints,
    FirstTrackingBA,
    MatchLocalMap,
    CommitTrackingStatistics,
    SecondTrackingBA,
    UpdatePreviousFrameMapPoints,
    GetKeyFrameInformation,
    IsKeyFrame,
    EnqueueKeyFrame,
    WaitForLocalMapping,
    InitializeKeyFramePreintegration,
    RejectKeyFrame,
    AppendTrackingTrajectory,
    UpdateVisualization,
    ShutdownMappingQueue,
    Count
};

static constexpr std::array<const char*,
    static_cast<std::size_t>(typeTrackingTimingStage::Count)>
TrackingTimingNames =
{
    "tracking/frame queue peek",
    "tracking/realtime pacing sleep",
    "tracking/skip late frame",
    "tracking/complete processed-frame iteration",
    "tracking/map snapshot critical section",
    "tracking/create tracking map",
    "tracking/get previous-frame map points",
    "tracking/update corrected trajectory",
    "tracking/IMU state arrival",
    "tracking/integrate IMU",
    "tracking/predict pose",
    "tracking/get keyframe and descriptors",
    "tracking/count matched map points",
    "tracking/first pose optimization",
    "tracking/match local map",
    "tracking/commit tracking statistics",
    "tracking/second pose optimization",
    "tracking/update previous-frame map points",
    "tracking/get keyframe information",
    "tracking/keyframe decision",
    "tracking/enqueue keyframe",
    "tracking/wait for local mapping",
    "tracking/initialize keyframe preintegration",
    "tracking/reject keyframe",
    "tracking/append trajectory",
    "tracking/update visualization",
    "tracking/shutdown mapping queue"
};

class typeTrackingScopedTimer
{
    public:
        explicit typeTrackingScopedTimer(typeTimingStatistics& Statistics)
            : Statistics(Statistics), Start(PantoClock::now())
        {}

        ~typeTrackingScopedTimer()
        {
            SL_AddTimingSample(
                    Statistics,
                    std::chrono::duration<fp64>(
                        PantoClock::now() - Start).count());
        }

    private:
        typeTimingStatistics& Statistics;
        PantoClock::time_point Start;
};

void SLPriv_TrackingThread(typeTrackingData& TrackingData, const i32 num_loops,
        typePreIntegration& PreIntegrationBetweenKF,
        bool& TrackingLost, i32& NumProcessedLoops);
void SLPriv_LocalMappingThread(typeLocalMapData& LocalMap);
static std::vector<typeGroundTruth> GroundTruth;

static void SLPriv_RemoveMissingMapPointAssociations(
        typeKeyFrame& Frame,
        const typeGlobalMap& GlobalMap)
{
    for(typePantoImagePoint& ImagePoint : Frame.Points.ImagePoints)
    {
        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET &&
           !GlobalMap.MapPoints.contains(ImagePoint.MapPointID))
        {
            ImagePoint.MapPointID = PANTO_ID_NOT_SET;
        }
    }
}

static void SLPriv_UpdateTrackingTrajectoryPose(
        const fp64 TimeStamp,
        const typeCamera& Camera)
{
    std::lock_guard<std::mutex> Lock(PantoSLAM.TrackingTrajectoryMutex);
    for(std::size_t i = PantoSLAM.TrackingTrajectoryTimeStamps.size();
        i > 0; i--)
    {
        const std::size_t Index = i - 1;
        if(std::abs(PantoSLAM.TrackingTrajectoryTimeStamps[Index] -
                    TimeStamp) < 1e-6)
        {
            PantoSLAM.TrackingTrajectory[Index] =
                CM_GetCameraCenter(Camera);
            return;
        }
    }
}

static Eigen::Vector3d SLPriv_GetGroundTruthCameraCenter(
        const typeGroundTruth& Measurement)
{
    const Eigen::Vector3d CameraInBody =
        CM_GetIntrinsics()->T_BS.block<3,1>(0,3);
    return Measurement.Position +
        Measurement.Orientation.toRotationMatrix() * CameraInBody;
}

static bool SLPriv_GetInitialMapParallaxStatistics(
        const typeGlobalMap& GlobalMap,
        fp64& MinimumDegrees,
        fp64& MedianDegrees,
        fp64& MaximumDegrees)
{
    std::vector<fp64> Parallaxes;
    Parallaxes.reserve(GlobalMap.MapPoints.active_size());

    const Eigen::Matrix3d& K = CM_GetIntrinsics()->K;
    const fp64 fx = K(0,0);
    const fp64 fy = K(1,1);
    const fp64 cx = K(0,2);
    const fp64 cy = K(1,2);

    for(const typePantoMapPoint& MapPoint : GlobalMap.MapPoints)
    {
        if(MapPoint.KeyFrameIDs.active_size() < 2 ||
           MapPoint.ImagePointIDs.active_size() < 2)
        {
            continue;
        }

        std::array<Eigen::Vector3d, 2> WorldRays{};
        std::size_t NumRays = 0;
        for(std::size_t i = 0;
            i < MapPoint.KeyFrameIDs.size() && NumRays < 2;
            i++)
        {
            if(!MapPoint.KeyFrameIDs.contains(i) ||
               !MapPoint.ImagePointIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID = MapPoint.KeyFrameIDs[i];
            const u64 ImagePointID = MapPoint.ImagePointIDs[i];
            if(!GlobalMap.KeyFrames.contains(KeyFrameID) ||
               !GlobalMap.KeyFrames[KeyFrameID].Points.ImagePoints.contains(
                   ImagePointID))
            {
                continue;
            }

            const typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameID];
            const Eigen::Vector2d& Point =
                KeyFrame.Points.ImagePoints[ImagePointID].Point;
            const Eigen::Vector3d CameraRay(
                    (Point.x() - cx) / fx,
                    (Point.y() - cy) / fy,
                    1.0);
            WorldRays[NumRays++] =
                KeyFrame.Camera.Pose.R.transpose() * CameraRay.normalized();
        }

        if(NumRays != 2)
        {
            continue;
        }

        const fp64 CosParallax = std::clamp(
                WorldRays[0].normalized().dot(WorldRays[1].normalized()),
                -1.0, 1.0);
        const fp64 ParallaxDegrees =
            std::acos(CosParallax) * 180.0 / M_PI;
        if(std::isfinite(ParallaxDegrees))
        {
            Parallaxes.push_back(ParallaxDegrees);
        }
    }

    if(Parallaxes.empty())
    {
        return false;
    }

    std::sort(Parallaxes.begin(), Parallaxes.end());
    MinimumDegrees = Parallaxes.front();
    MedianDegrees = Parallaxes[Parallaxes.size() / 2];
    MaximumDegrees = Parallaxes.back();
    return true;
}

#if !defined(DEBUG)
static std::vector<Eigen::Vector3d> GroundTruthVisualizationTrajectory;
static std::vector<fp64> GroundTruthVisualizationTimeStamps;

static Eigen::Vector3d SLPriv_GetGroundTruthCameraPosition(
        const typeGroundTruth& Measurement)
{
    const Eigen::Vector3d CameraInBody =
        CM_GetIntrinsics()->T_BS.block<3,1>(0,3);

    return Measurement.Position +
        Measurement.Orientation.toRotationMatrix() * CameraInBody;
}
#endif

#if !defined(DEBUG)
static void SLPriv_UpdateVisualization(typeGlobalMap* GlobalMap, typeTimingStatistics& VisualizationUpdateTiming)
{
    std::scoped_lock Lock(
            GlobalMap->Mutex, PantoSLAM.TrackingTrajectoryMutex);

    const PantoClock::time_point StartTime = PantoClock::now();
    VIZ_WriteColmap(*GlobalMap, PantoSLAM.TrackingTrajectory);
    const fp64 UpdateTime = std::chrono::duration<fp64>(
            PantoClock::now() - StartTime).count();

    SL_AddTimingSample(VisualizationUpdateTiming, UpdateTime);
    LG_Log(LogSeverity::DATA,
            "[SLAMVisualizationTiming] Update = %.6f s\n",
            UpdateTime);
}
#endif

#if defined(CONFIG_IMU)
static void SLPriv_UpdateCameraFromNavigationState(typeKeyFrame& Frame)
{
    const Eigen::Matrix4d& TBS = Frame.Camera.Intrinsics->T_BS;
    const Eigen::Matrix3d Rbs = TBS.block<3,3>(0,0);
    const Eigen::Vector3d Tbs = TBS.block<3,1>(0,3);
    const Eigen::Matrix3d Rbw = Frame.NavigationState.Rwb.transpose();
    const Eigen::Vector3d Tbw =
        -Rbw * Frame.NavigationState.Position;

    Frame.Camera.Pose.SetPose(
            Rbs.transpose() * Rbw,
            Rbs.transpose() * (Tbw - Tbs));
}

static void SLPriv_ReconstructFrameFromReferenceKF(typeKeyFrame& Frame,
        const typeKeyFrame& CurrentReference,
        const typePreIntegrationData& ReferenceToFramePreIntegration)
{
    if(!Frame.HasTrackingReferenceState)
    {
        return;
    }

    Frame.NavigationState = IMU_PredictNavigationState(
            CurrentReference.NavigationState,
            ReferenceToFramePreIntegration);
    SLPriv_UpdateCameraFromNavigationState(Frame);
    Frame.TrackingReferencePose = CurrentReference.Camera.Pose;
    Frame.TrackingReferenceNavigationState =
        CurrentReference.NavigationState;
    Frame.HasTrackingReferenceState = true;
}

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

static Eigen::Vector3d SLPriv_GetGroundTruthAcceleration( const std::size_t GroundTruthIndex)
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

    delete PantoSLAM.GlobalMap;
    delete PantoSLAM.CovisibilityGraph;
    delete PantoSLAM.RecentMapPointIndexes;
    delete PantoSLAM.KeyFrameQueue;

    PantoSLAM.GlobalMap = new typeGlobalMap();
    PantoSLAM.CovisibilityGraph = new typeCovisibilityGraph();
    PantoSLAM.RecentMapPointIndexes = new typePantoVector<u64>;
    PantoSLAM.KeyFrameQueue = new typeKeyFrameQueue();

    PantoSLAM.TrackingTrajectory = std::vector<Eigen::Vector3d>{};
    PantoSLAM.TrackingTrajectoryTimeStamps= std::vector<fp64>{};

#if !defined(DEBUG)
    VIZ_ResetMapVisualization();
#endif

#if !defined(CONFIG_IMU) && !defined(DEBUG)
    if(!GroundTruthVisualizationTrajectory.empty())
    {
        VIZ_SetGroundTruth(
                GroundTruthVisualizationTrajectory,
                GroundTruthVisualizationTimeStamps);
    }
#endif
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
        const Eigen::Vector3d FirstWorldAcceleration = SLPriv_GetGroundTruthAcceleration(FirstGroundTruthIndex);

        IMU_ResetGravityInitialization();

        if(!IMU_AddGravityInitializationMeasurement(
                    First,
                    FirstIMUMeasurement,
                    FirstWorldAcceleration))
        {
            LG_Log(LogSeverity::DATA,
                    "[SLPriv_InitializeMap] Ignored invalid first gravity initialization sample\n");
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

#if defined(CONFIG_IMU)
            typeIMUMeasurement SecondIMUMeasurement{};
            if(!SLPriv_IntegrateIMUUntil(
                        SecondFrame.TimeStamp,
                        &SecondIMUMeasurement))
            {
                LG_Log(LogSeverity::ERROR,
                        "[SLPriv_InitializeMap] Could not retrieve the IMU measurement for GT candidate %.9f\n",
                        SecondFrame.TimeStamp);
                return;
            }

            const std::size_t SecondGroundTruthIndex = GroundTruthIndex - 1;
            const Eigen::Vector3d SecondWorldAcceleration = SLPriv_GetGroundTruthAcceleration(SecondGroundTruthIndex);

            if(!IMU_AddGravityInitializationMeasurement(
                        Second, SecondIMUMeasurement, SecondWorldAcceleration))
            {
                LG_Log(LogSeverity::DATA,
                        "[SLPriv_InitializeMap] Ignored invalid gravity initialization sample at timestamp %.9f\n",
                        SecondFrame.TimeStamp);
            }
#endif

            const fp64 BodyBaseline =
                (SecondGT.Position - FirstGT.Position).norm();
            const fp64 Baseline =
                (SLPriv_GetGroundTruthCameraCenter(SecondGT) -
                 SLPriv_GetGroundTruthCameraCenter(FirstGT)).norm();

            if(!std::isfinite(Baseline) || Baseline < PANTO_MIN_INITIALIZATION_BASELINE_METERS)
            {
                LG_Log(LogSeverity::DATA,
                        "[SLAMGTInitialization] Candidate timestamp = %.9f; interval = %.6f s; camera baseline = %.6f/%.6f m; body baseline = %.6f m; rejected before triangulation\n",
                        SecondFrame.TimeStamp,
                        SecondFrame.TimeStamp - FirstFrame.TimeStamp,
                        Baseline,
                        PANTO_MIN_INITIALIZATION_BASELINE_METERS,
                        BodyBaseline);
                continue;
            }

#if defined(CONFIG_IMU)
            const typePreIntegrationData FirstToSecondPreIntegration =
                IMU_GetLatestPreIntegrationData();
#endif

            MAP_InitializeFromGT(First, Second, FirstFrame, SecondFrame, PantoSLAM.GlobalMap);

#if defined(CONFIG_IMU)
            PantoSLAM.GlobalMap->KeyFrames[0].PreviousKFID = PANTO_ID_NOT_SET;
            PantoSLAM.GlobalMap->KeyFrames[1].PreviousKFID = 0;
            PantoSLAM.GlobalMap->KeyFrames[1].PreIntegrationData =
                FirstToSecondPreIntegration;
#endif

            const std::size_t NumTriangulatedMapPoints = PantoSLAM.GlobalMap->MapPoints.active_size();

            fp64 MinimumParallaxDegrees = 0.0;
            fp64 MedianParallaxDegrees = 0.0;
            fp64 MaximumParallaxDegrees = 0.0;
            const bool HasParallaxStatistics =
                SLPriv_GetInitialMapParallaxStatistics(
                        *PantoSLAM.GlobalMap,
                        MinimumParallaxDegrees,
                        MedianParallaxDegrees,
                        MaximumParallaxDegrees);

            LG_Log(LogSeverity::DATA,
                    "[SLAMGTInitialization] Candidate timestamp = %.9f; interval = %.6f s; camera baseline = %.6f/%.6f m; body baseline = %.6f m; triangulated map points = %zu/%d; accepted parallax min/median/max = %.6f/%.6f/%.6f deg\n",
                    SecondFrame.TimeStamp, SecondFrame.TimeStamp - FirstFrame.TimeStamp,
                    Baseline, PANTO_MIN_INITIALIZATION_BASELINE_METERS,
                    BodyBaseline,
                    NumTriangulatedMapPoints, PANTO_MIN_NUMBER_INITIAL_MAP_POINTS,
                    MinimumParallaxDegrees,
                    MedianParallaxDegrees,
                    MaximumParallaxDegrees);

            if(NumTriangulatedMapPoints < static_cast<std::size_t>(PANTO_MIN_NUMBER_INITIAL_MAP_POINTS) ||
               !HasParallaxStatistics ||
               MinimumParallaxDegrees + 1e-9 < PANTO_INIT_MIN_PARALLAX_DEGREES)
            {
                delete(PantoSLAM.GlobalMap);
                PantoSLAM.GlobalMap = new typeGlobalMap();
                continue;
            }

#if defined(CONFIG_IMU)
            // Only the accepted candidate ends the first keyframe interval.
            // Rejected candidates remain part of the same first-to-candidate
            // preintegration interval.
            if(!IMU_FinalizeGravityInitialization())
            {
                LG_Log(LogSeverity::ERROR,
                        "[SLPriv_InitializeMap] Could not finalize gravity from the initialization interval\n");
                return;
            }

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

        // The GT-initialized points have already passed initialization's
        // geometric checks. They are not newly-created local-mapping points.
        // Keeping them in RecentMapPointIndexes until the first asynchronous
        // mapper pass makes their found ratio accumulate for many frames and
        // can delete nearly the entire initial map in one cull.

        const fp64 RefinedCameraBaseline =
            (CM_GetCameraCenter(PantoSLAM.GlobalMap->KeyFrames[1].Camera) -
             CM_GetCameraCenter(PantoSLAM.GlobalMap->KeyFrames[0].Camera)).norm();
        LG_Log(LogSeverity::DATA,
                "[SLAMGTInitialization] Camera baseline after points-only refinement = %.6f m\n",
                RefinedCameraBaseline);

        for(const typeKeyFrame& KeyFrame : PantoSLAM.GlobalMap->KeyFrames)
        {
            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, KeyFrame, PantoSLAM.GlobalMap->MapPoints, KeyFrame.ID);
            PantoSLAM.TrackingTrajectory.push_back(CM_GetCameraCenter(KeyFrame.Camera));
            PantoSLAM.TrackingTrajectoryTimeStamps.push_back(
                    KeyFrame.Camera.TimeStamp);
        }

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(*PantoSLAM.GlobalMap, PantoSLAM.GlobalMap->KeyFrames.back());
        PantoSLAM.PreviousFrameData.PreviousPreviousFrame = PantoSLAM.GlobalMap->KeyFrames[0];
        PantoSLAM.PreviousFrameData.PreviousFrame = PantoSLAM.GlobalMap->KeyFrames.back();
#if defined(CONFIG_IMU)
        PantoSLAM.PreviousFrameData.PreviousFrame.TrackingReferencePose =
            PantoSLAM.GlobalMap->KeyFrames[0].Camera.Pose;
        PantoSLAM.PreviousFrameData.PreviousFrame.
            TrackingReferenceNavigationState =
                PantoSLAM.GlobalMap->KeyFrames[0].NavigationState;
        PantoSLAM.PreviousFrameData.PreviousFrame.HasTrackingReferenceState = true;
#endif

#if defined(CONFIG_IMU)
        PantoSLAM.NextFramePosePrediction.Pose = IMU_PredictNavigationState(PantoSLAM.PreviousFrameData.PreviousFrame.NavigationState, PantoSLAM.PreviousFrameData.PreviousFrame.PreIntegrationData);
#else
        PantoSLAM.NextFramePosePrediction.Pose = CM_PredictPose(PantoSLAM.PreviousFrameData.PreviousFrame.Camera.Pose,
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

         INIT_ConstructInitialMap(Reconstruction,PantoSLAM.GlobalMap );

        INIT_DestroyInitData();

        PantoSLAM.RecentMapPointIndexes->reserve(PantoSLAM.GlobalMap->MapPoints.size());
        for(const typePantoMapPoint& MapPoint : PantoSLAM.GlobalMap->MapPoints)
        {
            PantoSLAM.RecentMapPointIndexes->push_back(MapPoint.ID);
        }

        typeKeyFrame ThirdKeyFrame = KEY_GetThirdKeyFrame(PantoSLAM.GlobalMap->KeyFrames.back(), PantoSLAM.GlobalMap->MapPoints);
        MAP_AppendKeyFrame(PantoSLAM.GlobalMap, ThirdKeyFrame);

        for(const typeKeyFrame& KeyFrame : PantoSLAM.GlobalMap->KeyFrames)
        {
            GRAPH_AddKeyFrame(PantoSLAM.CovisibilityGraph, KeyFrame, PantoSLAM.GlobalMap->MapPoints, KeyFrame.ID);
            PantoSLAM.TrackingTrajectory.push_back(CM_GetCameraCenter(KeyFrame.Camera));
            PantoSLAM.TrackingTrajectoryTimeStamps.push_back(
                    KeyFrame.Camera.TimeStamp);
        }

#if defined(DEBUG)
        MAP_LogGlobalMap(*PantoSLAM.GlobalMap);
        MAP_LogGraphConsistency(
                *PantoSLAM.GlobalMap,
                *PantoSLAM.CovisibilityGraph);
        GRAPH_Log(*PantoSLAM.CovisibilityGraph);
#endif

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error before tracking\n");
        MAP_LogGlobalMapProjectionErrors(*PantoSLAM.GlobalMap);
        OP_BundleAdjust(*PantoSLAM.GlobalMap, OptimizationTypeTracking, {},
                &PantoSLAM.GlobalMap->KeyFrames.back(),
                &PantoSLAM.GlobalMap->KeyFrames[1]);
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Global map reprojection error after tracking\n");
        MAP_LogGlobalMapProjectionErrors(*PantoSLAM.GlobalMap);
        OP_BundleAdjust(*PantoSLAM.GlobalMap,
                OptimizationTypePoseAndPoints, {}, nullptr);

        PantoSLAM.PreviousFrameData.PreviousFrameMapPoints = MAP_GetLastFrameMapPoints(*PantoSLAM.GlobalMap, PantoSLAM.GlobalMap->KeyFrames.back());
        PantoSLAM.PreviousFrameData.PreviousPreviousFrame = PantoSLAM.GlobalMap->KeyFrames[1];
        PantoSLAM.PreviousFrameData.PreviousFrame = PantoSLAM.GlobalMap->KeyFrames.back();

#if !defined(CONFIG_IMU)
        PantoSLAM.NextFramePosePrediction.Pose = CM_PredictPose(
                PantoSLAM.PreviousFrameData.PreviousFrame.Camera.Pose,
                PantoSLAM.PreviousFrameData.PreviousPreviousFrame.Camera.Pose);
#endif
    }

}

void SL_PantoSLAM(i32 num_loops)
{
    typeTimingStatistics VisualizationUpdateTiming{};

#if !defined(DEBUG)
    if(PANTO_USE_DATASET && !std::string(panto_gt_path).empty())
    {
        GroundTruth = GT_GetAllMeasurements();

        GroundTruthVisualizationTrajectory.clear();
        GroundTruthVisualizationTrajectory.reserve(GroundTruth.size());
        GroundTruthVisualizationTimeStamps.clear();
        GroundTruthVisualizationTimeStamps.reserve(GroundTruth.size());

        for(const typeGroundTruth& Measurement : GroundTruth)
        {
            GroundTruthVisualizationTrajectory.push_back(
                    SLPriv_GetGroundTruthCameraPosition(Measurement));
            GroundTruthVisualizationTimeStamps.push_back(
                    Measurement.TimeStamp);
        }

        VIZ_SetGroundTruth(
                GroundTruthVisualizationTrajectory,
                GroundTruthVisualizationTimeStamps);

        LG_Log(LogSeverity::DATA,
                "[SLAMGroundTruthVisualization] Published %zu ground-truth positions\n",
                GroundTruthVisualizationTrajectory.size());
    }
#endif

    const PantoClock::time_point InitializationStartTime = PantoClock::now();

    SLPriv_InitializeMap();

    const PantoClock::time_point InitializationEndTime = PantoClock::now();

    LG_Log(LogSeverity::DATA, "[SLAMTiming] Initialization = %.6f s\n",
            std::chrono::duration<fp64>(InitializationEndTime - InitializationStartTime).count());

    if(PantoSLAM.GlobalMap->KeyFrames.active_size() == 0)
    {
        LG_Log(LogSeverity::ERROR,
                "[SL_PantoSLAM] Initialization produced no keyframes\n");
#if !defined(DEBUG)
        std::cout << "SLAM initialization failed; press Enter to close the visualization.\n";
        std::cin.get();
        VIZ_StopViewer();
#endif
        return;
    }

#if !defined(DEBUG)
    SLPriv_UpdateVisualization(
            PantoSLAM.GlobalMap,
            VisualizationUpdateTiming);
#endif

    i32 RemainingLoops = num_loops;
    while(RemainingLoops > 0)
    {
        typePreIntegration PreIntegrationBetweenKF{};
#if defined(CONFIG_IMU)
        IMU_InitializePreIntegration(PreIntegrationBetweenKF,
                PantoSLAM.GlobalMap->KeyFrames.back().NavigationState);
#endif

        typeKeyFrameQueue KeyFrameQueue{};

        typeTrackingData TrackingData
        {
            .PreviousFrameData = PantoSLAM.PreviousFrameData,
            .TrackingMap = typeLocalMapTracking{},
            .NewFrame = typeKeyFrame{},
            .AccumulatedDistance = fp64{},
            .PosePrediction = PantoSLAM.NextFramePosePrediction,

            .KeyFrameQueue = &KeyFrameQueue,
            .GlobalMap = PantoSLAM.GlobalMap,
            .CovisibilityGraph = PantoSLAM.CovisibilityGraph
        };

        bool TrackingLost = false;
        i32 NumProcessedLoops = 0;
        std::thread TrackingThread(
                SLPriv_TrackingThread,
                std::ref(TrackingData),
                RemainingLoops,
                std::ref(PreIntegrationBetweenKF),
                std::ref(TrackingLost),
                std::ref(NumProcessedLoops));

        typeLocalMapData LocalMappingData
        {
            .LocalMap = typeLocalMap{},
            .RecentMapPointIndexes = *PantoSLAM.RecentMapPointIndexes,

            .KeyFrameQueue = &KeyFrameQueue,
            .GlobalMap = PantoSLAM.GlobalMap,
            .CovisibilityGraph = PantoSLAM.CovisibilityGraph,
            .VisualizationUpdateTiming = &VisualizationUpdateTiming
        };
        std::thread LocalMappingThread(
                SLPriv_LocalMappingThread,
                std::ref(LocalMappingData));

        TrackingThread.join();
        LocalMappingThread.join();

        RemainingLoops -= std::min(RemainingLoops, NumProcessedLoops);
        if(!TrackingLost)
        {
            break;
        }

        LG_Log(LogSeverity::DATA,
                "[SLAMLoop] Workers stopped after tracking loss; resetting and reinitializing SLAM\n");

        SLPriv_ResetMapAndTracking();

        const PantoClock::time_point ReinitializationStartTime = PantoClock::now();
        SLPriv_InitializeMap();
        const PantoClock::time_point ReinitializationEndTime = PantoClock::now();

        LG_Log(LogSeverity::DATA, "[SLAMTiming] Reinitialization = %.6f s\n",
                std::chrono::duration<fp64>(
                    ReinitializationEndTime - ReinitializationStartTime).count());

        if(PantoSLAM.GlobalMap->KeyFrames.active_size() == 0)
        {
            LG_Log(LogSeverity::ERROR,
                    "[SL_PantoSLAM] Reinitialization produced no keyframes\n");
            break;
        }

#if !defined(DEBUG)
        SLPriv_UpdateVisualization(
                PantoSLAM.GlobalMap,
                VisualizationUpdateTiming);
#endif
    }

#if !defined(DEBUG)
    if(PantoSLAM.GlobalMap->KeyFrames.active_size() > 0)
    {
        // Publish once after both workers have stopped so the viewer receives
        // the final committed map even if no final keyframe caused an update.
        SLPriv_UpdateVisualization(
                PantoSLAM.GlobalMap,
                VisualizationUpdateTiming);
    }

    SL_LogTimingStatistics(
            "Visualization update",
            VisualizationUpdateTiming);

    std::cout << "SLAM complete; press Enter to close the visualization.\n";
    std::cin.get();
    VIZ_StopViewer();
#endif
}

void SLPriv_TrackingThread(typeTrackingData& TrackingData, const i32 num_loops,
        typePreIntegration& PreIntegrationBetweenKF,
        bool& TrackingLost, i32& NumProcessedLoops)
{
    std::array<typeTimingStatistics,
        static_cast<std::size_t>(typeTrackingTimingStage::Count)>
        Timing{};
    const auto Statistics = [&Timing](const typeTrackingTimingStage Stage)
        -> typeTimingStatistics&
    {
        return Timing[static_cast<std::size_t>(Stage)];
    };

    TrackingLost = false;
    NumProcessedLoops = 0;
#if !defined(CONFIG_IMU)
    (void)PreIntegrationBetweenKF;
#endif

    LG_Log(LogSeverity::DATA,
            "[SLAMDatasetMode] %s\n",
            PANTO_DATASET_REALTIME_MODE
                ? "real-time timestamps; late frames may be skipped and local BA may be interrupted"
                : "all frames; tracking waits for each queued local-mapping iteration and local BA runs to completion");

    // Replay timestamped datasets at their recorded rate. An absolute clock
    // anchor prevents processing and sleep errors from accumulating as drift.
    const fp64 ReplayStartTimeStamp = TrackingData.PreviousFrameData.PreviousFrame.Camera.TimeStamp;
    const PantoClock::time_point ReplayStartWallTime = PantoClock::now();

#if defined(CONFIG_IMU)
    // The preintegration accumulator is reset whenever tracking selects a new
    // keyframe. Keep that (possibly not-yet-committed) keyframe as the inertial
    // reference for every following ordinary frame.
    typeKeyFrame InertialReferenceKeyFrame =
        TrackingData.PreviousFrameData.PreviousFrame;
    std::vector<typeKeyFrame> InertialReferenceChain{
        InertialReferenceKeyFrame};
    u64 InertialReferenceKFID = InertialReferenceKeyFrame.ID;
    u64 InertialReferenceMappingGeneration =
        InertialReferenceKeyFrame.MappingGeneration;
    u64 LastSeenMapRevision = 0;
    {
        std::lock_guard<std::mutex> Lock(TrackingData.GlobalMap->Mutex);
        LastSeenMapRevision = TrackingData.GlobalMap->Revision;
    }
#endif

    while(NumProcessedLoops < num_loops)
    {
        fp64 NextFrameTimeStamp = PANTO_TIMESTAMP_NOT_SET;

        if(PANTO_USE_DATASET)
        {
            {
                typeTrackingScopedTimer Timer(
                        Statistics(typeTrackingTimingStage::FrameQueuePeek));
                NextFrameTimeStamp = FR_PeekNextFrameTimeStamp();
            }
            if(NextFrameTimeStamp < 0.0)
            {
                break;
            }

            if(PANTO_DATASET_REALTIME_MODE)
            {
                const PantoClock::time_point TargetWallTime =
                    ReplayStartWallTime + std::chrono::duration_cast<PantoClock::duration>(
                            std::chrono::duration<fp64>(NextFrameTimeStamp - ReplayStartTimeStamp));
                const PantoClock::time_point CurrentWallTime = PantoClock::now();

                if(CurrentWallTime > TargetWallTime)
                {
                    const fp64 Lateness = std::chrono::duration<fp64>(
                            CurrentWallTime - TargetWallTime).count();
                    fp64 SkippedTimeStamp = PANTO_TIMESTAMP_NOT_SET;
                    {
                        typeTrackingScopedTimer Timer(
                                Statistics(typeTrackingTimingStage::
                                    SkipFrame));
                        SkippedTimeStamp = FR_SkipNextFrame();
                    }
                    if(SkippedTimeStamp < 0.0)
                    {
                        break;
                    }

                    NumProcessedLoops++;
                    LG_Log(LogSeverity::DATA,
                            "[SLAMFramePacing] Skipped frame at %.9f s; late by %.6f s\n",
                            SkippedTimeStamp, Lateness);
                    continue;
                }

                {
                    typeTrackingScopedTimer Timer(
                            Statistics(typeTrackingTimingStage::
                                RealtimePacingSleep));
                    std::this_thread::sleep_until(TargetWallTime);
                }
            }
        }

        NumProcessedLoops++;
        typeTrackingScopedTimer IterationTimer(
                Statistics(typeTrackingTimingStage::IterationTotal));
#if defined(CONFIG_IMU)
        bool UpdatePreviousTrajectory = false;
        typeCamera PreviousTrajectoryCamera{};
#endif

        {
            typeTrackingScopedTimer TransactionTimer(
                    Statistics(typeTrackingTimingStage::
                        MapSnapshotTransaction));
#if defined(CONFIG_IMU)
            // Take one coherent, bounded map/graph snapshot for the whole
            // tracking iteration. Building a second snapshot after feature
            // matching can mix point positions from one local-BA commit with
            // keyframe poses/covisibility from another.
            std::scoped_lock Lock(TrackingData.GlobalMap->Mutex,
                    TrackingData.CovisibilityGraph->Mutex);
            typeKeyFrame& PreviousFrame = TrackingData.PreviousFrameData.PreviousFrame;
            const bool MapChangedSincePreviousFrame =
                TrackingData.GlobalMap->Revision != LastSeenMapRevision;
            bool AdoptedCommittedKeyFrame = false;
            if(PreviousFrame.MappingGeneration != PANTO_ID_NOT_SET)
            {
                for(const typeKeyFrame& GlobalKeyFrame :
                        TrackingData.GlobalMap->KeyFrames)
                {
                    if(GlobalKeyFrame.MappingGeneration !=
                            PreviousFrame.MappingGeneration)
                    {
                        continue;
                    }

                    PreviousFrame = GlobalKeyFrame;
                    if(InertialReferenceMappingGeneration ==
                           GlobalKeyFrame.MappingGeneration)
                    {
                        InertialReferenceKeyFrame = GlobalKeyFrame;
                        InertialReferenceKFID = GlobalKeyFrame.ID;
                    }
                    if(PreviousFrame.PreviousKFID != PANTO_ID_NOT_SET &&
                       TrackingData.GlobalMap->KeyFrames.contains(
                           PreviousFrame.PreviousKFID))
                    {
                        PreviousFrame.TrackingReferencePose =
                            TrackingData.GlobalMap->KeyFrames[
                                PreviousFrame.PreviousKFID].Camera.Pose;
                        PreviousFrame.TrackingReferenceNavigationState =
                            TrackingData.GlobalMap->KeyFrames[
                                PreviousFrame.PreviousKFID].NavigationState;
                        PreviousFrame.HasTrackingReferenceState = true;
                    }
                    PreviousTrajectoryCamera = PreviousFrame.Camera;
                    UpdatePreviousTrajectory = true;
                    AdoptedCommittedKeyFrame = true;
                    LG_Log(LogSeverity::DBG,
                            "[SLAMAsyncCorrection] Adopted optimized KF %llu for mapping generation %llu\n",
                            GlobalKeyFrame.ID,
                            GlobalKeyFrame.MappingGeneration);
                    break;
                }
            }

            // Refresh the committed prefix, then propagate that revision
            // through any keyframes still waiting in the mapper queue. This
            // keeps the active reference current even when mapping falls more
            // than one keyframe behind tracking.
            bool InertialReferenceResolvedFromMap = false;
            std::size_t LastCommittedReferenceIndex = 0;
            bool PreviousReferenceIsCurrent = false;
            for(std::size_t ReferenceIndex = 0;
                ReferenceIndex < InertialReferenceChain.size();
                ReferenceIndex++)
            {
                typeKeyFrame& Reference =
                    InertialReferenceChain[ReferenceIndex];
                const typeKeyFrame* CommittedReference = nullptr;
                if(Reference.MappingGeneration != PANTO_ID_NOT_SET)
                {
                    for(const typeKeyFrame& GlobalKeyFrame :
                            TrackingData.GlobalMap->KeyFrames)
                    {
                        if(GlobalKeyFrame.MappingGeneration ==
                                Reference.MappingGeneration)
                        {
                            CommittedReference = &GlobalKeyFrame;
                            break;
                        }
                    }
                }
                else if(TrackingData.GlobalMap->KeyFrames.contains(
                            Reference.ID))
                {
                    CommittedReference =
                        &TrackingData.GlobalMap->KeyFrames[Reference.ID];
                }

                if(CommittedReference != nullptr)
                {
                    Reference = *CommittedReference;
                    LastCommittedReferenceIndex = ReferenceIndex;
                    PreviousReferenceIsCurrent = true;
                }
                else if(ReferenceIndex > 0 && PreviousReferenceIsCurrent)
                {
                    if(MapChangedSincePreviousFrame)
                    {
                        SLPriv_ReconstructFrameFromReferenceKF(
                                Reference,
                                InertialReferenceChain[ReferenceIndex - 1],
                                Reference.PreIntegrationData);
                    }
                    PreviousReferenceIsCurrent = true;
                }
                else
                {
                    PreviousReferenceIsCurrent = false;
                }
            }

            InertialReferenceResolvedFromMap = PreviousReferenceIsCurrent;
            const bool ActiveReferenceIsCommitted =
                LastCommittedReferenceIndex + 1 ==
                    InertialReferenceChain.size();
            if(LastCommittedReferenceIndex > 0)
            {
                InertialReferenceChain.erase(
                        InertialReferenceChain.begin(),
                        InertialReferenceChain.begin() +
                            LastCommittedReferenceIndex);
            }
            InertialReferenceKeyFrame = InertialReferenceChain.back();
            InertialReferenceKFID = ActiveReferenceIsCommitted
                ? InertialReferenceKeyFrame.ID
                : PANTO_ID_NOT_SET;
            InertialReferenceMappingGeneration =
                InertialReferenceKeyFrame.MappingGeneration;

            const bool PreviousFrameIsInertialReference =
                std::abs(PreviousFrame.Camera.TimeStamp -
                         InertialReferenceKeyFrame.Camera.TimeStamp) < 1e-9;
            if(MapChangedSincePreviousFrame &&
               !AdoptedCommittedKeyFrame &&
               InertialReferenceResolvedFromMap &&
               PreviousFrame.HasTrackingReferenceState &&
               !PreviousFrameIsInertialReference)
            {
                SLPriv_ReconstructFrameFromReferenceKF(
                        PreviousFrame,
                        InertialReferenceKeyFrame,
                        PreviousFrame.
                            TrackingReferencePreIntegrationData);
                LG_Log(LogSeverity::DBG,
                        "[SLAMAsyncCorrection] Repropagated frame %.9f from optimized KF %llu at map revision %llu\n",
                        PreviousFrame.Camera.TimeStamp,
                        static_cast<unsigned long long>(
                            InertialReferenceKeyFrame.ID),
                        static_cast<unsigned long long>(
                            TrackingData.GlobalMap->Revision));
                PreviousTrajectoryCamera = PreviousFrame.Camera;
                UpdatePreviousTrajectory = true;
            }
            LastSeenMapRevision = TrackingData.GlobalMap->Revision;

            typeKeyFrame& PreviousPreviousFrame =
                TrackingData.PreviousFrameData.PreviousPreviousFrame;
            if(PreviousPreviousFrame.MappingGeneration != PANTO_ID_NOT_SET)
            {
                for(const typeKeyFrame& GlobalKeyFrame :
                        TrackingData.GlobalMap->KeyFrames)
                {
                    if(GlobalKeyFrame.MappingGeneration ==
                            PreviousPreviousFrame.MappingGeneration)
                    {
                        PreviousPreviousFrame = GlobalKeyFrame;
                        break;
                    }
                }
            }
            {
                typeTrackingScopedTimer Timer(
                        Statistics(typeTrackingTimingStage::
                            CreateTrackingMap));
                TrackingData.TrackingMap = MAP_CreateLocalMapTracking(
                        *TrackingData.GlobalMap,
                        *TrackingData.CovisibilityGraph,
                        PreviousFrame);
            }
            {
                typeTrackingScopedTimer Timer(
                        Statistics(typeTrackingTimingStage::
                            GetPreviousFrameMapPoints));
                TrackingData.PreviousFrameData.PreviousFrameMapPoints =
                    MAP_GetLastFrameMapPoints(
                            TrackingData.TrackingMap.MapPoints,
                            PreviousFrame);
            }
#else
            // Local mapping may have culled or optimized points since the
            // previous frame was processed. Match against a fresh snapshot.
            std::lock_guard<std::mutex> Lock(TrackingData.GlobalMap->Mutex);
            {
                typeTrackingScopedTimer Timer(
                        Statistics(typeTrackingTimingStage::
                            GetPreviousFrameMapPoints));
                TrackingData.PreviousFrameData.PreviousFrameMapPoints =
                    MAP_GetLastFrameMapPoints(
                            *TrackingData.GlobalMap,
                            TrackingData.PreviousFrameData.PreviousFrame);
            }
#endif
        }

#if defined(CONFIG_IMU)
        if(UpdatePreviousTrajectory)
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::
                        UpdateCorrectedTrajectory));
            SLPriv_UpdateTrackingTrajectoryPose(
                    PreviousTrajectoryCamera.TimeStamp,
                    PreviousTrajectoryCamera);
        }
#endif

#if defined(CONFIG_IMU)
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::IMUStateArrival));
            IMU_NewNavigationStateArrival(
                    TrackingData.PreviousFrameData.PreviousFrame.
                        NavigationState);
        }

        bool IMUIntegrated = false;
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::IntegrateIMU));
            IMUIntegrated = SLPriv_IntegrateIMUUntil(
                    NextFrameTimeStamp,
                    nullptr,
                    &PreIntegrationBetweenKF);
        }
        if(!IMUIntegrated)
        {
            break;
        }

        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::PredictPose));
            TrackingData.PosePrediction.Pose =
                KEY_PredictPose(
                        TrackingData.PreviousFrameData.PreviousFrame);
        }
#endif

        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::GetKeyFrame));
            TrackingData.NewFrame = KEY_GetKeyFrame(
                    TrackingData.PosePrediction.Pose,
                    TrackingData.PreviousFrameData.PreviousFrameMapPoints);
        }

        if(TrackingData.NewFrame.Camera.TimeStamp < 0.0f)
        {
            // Invalid timestamp means failure to read image.
            break;
        }

#if defined(CONFIG_IMU)
        TrackingData.NewFrame.PreIntegrationData = IMU_GetLatestPreIntegrationData();
#endif

        {
#if defined(CONFIG_IMU)
            // TrackingMap and its reference pose were captured atomically
            // above; do not combine them with a newer global-map revision.
            TrackingData.NewFrame.PreviousKFID = InertialReferenceKFID;
            TrackingData.NewFrame.TrackingReferencePose =
                InertialReferenceKeyFrame.Camera.Pose;
            TrackingData.NewFrame.TrackingReferenceNavigationState =
                InertialReferenceKeyFrame.NavigationState;
            TrackingData.NewFrame.TrackingReferencePreIntegrationData =
                PreIntegrationBetweenKF;
            TrackingData.NewFrame.TrackingReferenceMappingGeneration =
                InertialReferenceMappingGeneration;
            TrackingData.NewFrame.HasTrackingReferenceState = true;
#else
            std::scoped_lock Lock( TrackingData.GlobalMap->Mutex,
                    TrackingData.CovisibilityGraph->Mutex);
            // A point can disappear after the previous-frame snapshot was
            // taken but before feature extraction finishes.
            SLPriv_RemoveMissingMapPointAssociations(
                    TrackingData.NewFrame,
                    *TrackingData.GlobalMap);
            TrackingData.TrackingMap = MAP_CreateLocalMapTracking(
                    *TrackingData.GlobalMap,
                    *TrackingData.CovisibilityGraph,
                    TrackingData.NewFrame);
#endif
        }

#if defined(DEBUG)
        {
            std::lock_guard<std::mutex> Lock(TrackingData.GlobalMap->Mutex);
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Logging all poses\n");
            MAP_LogGlobalMapPoses(*TrackingData.GlobalMap);
        }
#endif

        const auto CountMatchedMapPoints = [&TrackingData]()
        {
            u64 Count = 0;
            for(const typePantoImagePoint& ImagePoint :
                    TrackingData.NewFrame.Points.ImagePoints)
            {
                if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
                {
                    Count++;
                }
            }
            return Count;
        };

        u64 NumMatchedMapPoints = 0;
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::
                        CountMatchedMapPoints));
            NumMatchedMapPoints = CountMatchedMapPoints();
        }
        typeLocalMapInfo LocalMapInfo{};
        bool LocalMapAlreadyMatched = false;

#if !defined(CONFIG_IMU)
        if(NumMatchedMapPoints <= PANTO_TRACKING_MIN_MATCHED_MAP_POINTS)
        {
            LG_Log(LogSeverity::DATA,
                    "[SLAMLoop] Previous-frame tracking found %llu map points; attempting local-map recovery\n",
                    static_cast<unsigned long long>(NumMatchedMapPoints));

            // A tracking map built from the failed current-frame matches can
            // be empty. Rebuild it around the last successfully tracked frame
            // so recovery searches its full covisible local neighborhood.
            {
                std::scoped_lock Lock(
                        TrackingData.GlobalMap->Mutex,
                        TrackingData.CovisibilityGraph->Mutex);
                TrackingData.TrackingMap = MAP_CreateLocalMapTracking(
                        *TrackingData.GlobalMap,
                        *TrackingData.CovisibilityGraph,
                        TrackingData.PreviousFrameData.PreviousFrame);
            }

            LocalMapInfo = MAP_MatchMapPointLocalMap(
                    TrackingData.TrackingMap,
                    TrackingData.NewFrame);
            LocalMapAlreadyMatched = true;
            NumMatchedMapPoints = CountMatchedMapPoints();

            if(NumMatchedMapPoints > PANTO_TRACKING_MIN_MATCHED_MAP_POINTS)
            {
                LG_Log(LogSeverity::DATA,
                        "[SLAMLoop] Local-map recovery succeeded with %llu matched map points\n",
                        static_cast<unsigned long long>(NumMatchedMapPoints));
            }
            else
            {
                LG_Log(
                    LogSeverity::ERROR,
                    "[SLAMLoop] Tracking lost after local-map recovery with %llu matched map points\n",
                    static_cast<unsigned long long>(NumMatchedMapPoints));

                TrackingLost = true;
                break;
            }
        }
#endif

        if(!LocalMapAlreadyMatched &&
           NumMatchedMapPoints > PANTO_TRACKING_MIN_MATCHED_MAP_POINTS)
        {
            LG_Log(LogSeverity::DBG, "[SLAMLoop] Running first tracking optimization\n");
            const Eigen::Matrix3d RBefore = TrackingData.NewFrame.Camera.Pose.R;
            const Eigen::Vector3d tBefore = TrackingData.NewFrame.Camera.Pose.t;

            {
                typeTrackingScopedTimer Timer(
                        Statistics(typeTrackingTimingStage::FirstTrackingBA));
                OP_BundleAdjustTracking(
                        TrackingData.TrackingMap,
                        &TrackingData.NewFrame,
                        &TrackingData.PreviousFrameData.PreviousFrame);
            }

            /*
             * Compare optimized pose against predicted/input pose.
             */
            const Eigen::Matrix3d& RAfter = TrackingData.NewFrame.Camera.Pose.R;
            const Eigen::Vector3d& tAfter = TrackingData.NewFrame.Camera.Pose.t;
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

        }

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Creating local map\n");
        LG_Log(LogSeverity::DBG, "[SLAMLoop] Local Map size = %zu\n",
                TrackingData.TrackingMap.KeyFrames.size());

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Matching local map points\n");
        if(!LocalMapAlreadyMatched)
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::MatchLocalMap));
            LocalMapInfo = MAP_MatchMapPointLocalMap(
                    TrackingData.TrackingMap, TrackingData.NewFrame);
        }
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::
                        CommitTrackingStatistics));
            MAP_CommitTrackingStatistics(
                    TrackingData.GlobalMap,
                    LocalMapInfo);
        }

        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::SecondTrackingBA));
            OP_BundleAdjustTracking(
                    TrackingData.TrackingMap,
                    &TrackingData.NewFrame,
                    &TrackingData.PreviousFrameData.PreviousFrame);
        }

        const typePreviousFrameData PreviousFrameDataCopy =
            TrackingData.PreviousFrameData;
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::
                        UpdatePreviousFrameMapPoints));
            TrackingData.PreviousFrameData.PreviousFrameMapPoints =
                MAP_GetLastFrameMapPoints(
                        TrackingData.TrackingMap.MapPoints,
                        TrackingData.NewFrame);
        }
        TrackingData.PreviousFrameData.PreviousPreviousFrame =
            TrackingData.PreviousFrameData.PreviousFrame;
        TrackingData.PreviousFrameData.PreviousFrame = TrackingData.NewFrame;

#if !defined(CONFIG_IMU)
        TrackingData.PosePrediction.Pose = CM_PredictPose(
                TrackingData.PreviousFrameData.PreviousFrame.Camera.Pose,
                TrackingData.PreviousFrameData.PreviousPreviousFrame.Camera.Pose);
#endif

        typeKeyFrameInformation KeyFrameInfo{};
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::
                        GetKeyFrameInformation));
            KeyFrameInfo = SLPriv_GetKeyFrameInformation(
                    PreviousFrameDataCopy,
                    TrackingData.NewFrame,
                    LocalMapInfo,
                    TrackingData.AccumulatedDistance);
        }
        bool IsKeyFrame = false;
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::IsKeyFrame));
            IsKeyFrame = KEY_IsKeyFrame(KeyFrameInfo);
        }
        // if(IsKeyFrame)
        if(((NumProcessedLoops % 5) == 0))
        {
#if defined(CONFIG_IMU)
            // Let the local mapping thread get the KF <-> KF preintegration data.
            TrackingData.NewFrame.PreIntegrationData = PreIntegrationBetweenKF;
#endif
            u64 QueuedGeneration = PANTO_ID_NOT_SET;
            {
                typeTrackingScopedTimer Timer(
                        Statistics(typeTrackingTimingStage::EnqueueKeyFrame));
                QueuedGeneration =
                    TrackingData.KeyFrameQueue->enque(
                            TrackingData.NewFrame);
            }
            TrackingData.NewFrame.MappingGeneration = QueuedGeneration;
            TrackingData.PreviousFrameData.PreviousFrame.MappingGeneration =
                QueuedGeneration;
#if defined(CONFIG_IMU)
            InertialReferenceKeyFrame = TrackingData.NewFrame;
            InertialReferenceMappingGeneration = QueuedGeneration;
            InertialReferenceKFID = PANTO_ID_NOT_SET;
            InertialReferenceChain.push_back(InertialReferenceKeyFrame);
            // Keep frame <-> frame preintegration data for tracking.
            TrackingData.NewFrame.PreIntegrationData = IMU_GetLatestPreIntegrationData();
#endif
            TrackingData.AccumulatedDistance = 0.0;

            if(!PANTO_DATASET_REALTIME_MODE)
            {
                {
                    typeTrackingScopedTimer Timer(
                            Statistics(typeTrackingTimingStage::
                                WaitForLocalMapping));
                    TrackingData.KeyFrameQueue->WaitUntilProcessed(
                            QueuedGeneration);
                }

                // The mapper owns a copy of the queued frame and local BA may
                // change both its pose and navigation state. In serialized
                // mode, hand that committed state back to tracking before the
                // next IMU prediction and before recording the trajectory.
                std::lock_guard<std::mutex> Lock(
                        TrackingData.GlobalMap->Mutex);
                const typeKeyFrame& CommittedKeyFrame =
                    TrackingData.GlobalMap->KeyFrames.back();
                assert(std::abs(CommittedKeyFrame.Camera.TimeStamp -
                            TrackingData.NewFrame.Camera.TimeStamp) < 1e-6);

                TrackingData.NewFrame = CommittedKeyFrame;
#if defined(CONFIG_IMU)
                if(TrackingData.NewFrame.PreviousKFID != PANTO_ID_NOT_SET &&
                   TrackingData.GlobalMap->KeyFrames.contains(
                       TrackingData.NewFrame.PreviousKFID))
                {
                    TrackingData.NewFrame.TrackingReferencePose =
                        TrackingData.GlobalMap->KeyFrames[
                            TrackingData.NewFrame.PreviousKFID].Camera.Pose;
                    TrackingData.NewFrame.TrackingReferenceNavigationState =
                        TrackingData.GlobalMap->KeyFrames[
                            TrackingData.NewFrame.PreviousKFID].NavigationState;
                    TrackingData.NewFrame.HasTrackingReferenceState = true;
                }
#endif
                TrackingData.PreviousFrameData.PreviousFrame =
                    TrackingData.NewFrame;
                InertialReferenceKeyFrame = CommittedKeyFrame;
                InertialReferenceKFID = CommittedKeyFrame.ID;
                InertialReferenceMappingGeneration =
                    CommittedKeyFrame.MappingGeneration;
                InertialReferenceChain.clear();
                InertialReferenceChain.push_back(CommittedKeyFrame);
                TrackingData.PreviousFrameData.PreviousFrameMapPoints =
                    MAP_GetLastFrameMapPoints(
                            *TrackingData.GlobalMap,
                            TrackingData.NewFrame);
            }

#if defined(CONFIG_IMU)
            {
                typeTrackingScopedTimer Timer(
                        Statistics(typeTrackingTimingStage::
                            InitializeKeyFramePreintegration));
                IMU_InitializePreIntegration(
                        PreIntegrationBetweenKF,
                        TrackingData.NewFrame.NavigationState);
            }
#endif
        }
        else
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::RejectKeyFrame));
            KEY_NonValidKeyFrame();
        }

        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::
                        AppendTrackingTrajectory));
            std::lock_guard<std::mutex> Lock(
                    PantoSLAM.TrackingTrajectoryMutex);
            PantoSLAM.TrackingTrajectory.push_back(
                    CM_GetCameraCenter(TrackingData.NewFrame.Camera));
            PantoSLAM.TrackingTrajectoryTimeStamps.push_back(
                    TrackingData.NewFrame.Camera.TimeStamp);
        }
#if !defined(DEBUG)
        typeTimingStatistics Stats;
        {
            typeTrackingScopedTimer Timer(
                    Statistics(typeTrackingTimingStage::
                        UpdateVisualization));
            SLPriv_UpdateVisualization(TrackingData.GlobalMap, Stats);
        }
#endif
    }
    {
        typeTrackingScopedTimer Timer(
                Statistics(typeTrackingTimingStage::ShutdownMappingQueue));
        TrackingData.KeyFrameQueue->ShutDown();
    }

    LG_EnableDataSummaryLoggingForCurrentThread(true);
    for(std::size_t StageIndex = 0; StageIndex < Timing.size(); StageIndex++)
    {
        if(Timing[StageIndex].Count == 0)
        {
            continue;
        }
        SL_LogTimingStatistics(
                TrackingTimingNames[StageIndex],
                Timing[StageIndex]);
    }
    LG_EnableDataSummaryLoggingForCurrentThread(false);
}

typeKeyFrameInformation SLPriv_GetKeyFrameInformation(const typePreviousFrameData& PreviousFrameDataCopy, const typeKeyFrame& NewKeyFrame,
        const typeLocalMapInfo& LocalMapInfo, fp64& AccumulatedDistance)
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

    AccumulatedDistance += (C2 - C1).norm() / LocalMapDepth;

    KeyFrameInfo.AcumulatedDistanceTravelled = AccumulatedDistance;

    LG_Log( LogSeverity::DBG,
        "[SLPriv_GetKeyFrameInformation] Result: VelocityChange = %f, LocalMapTrackingRatio = %f, AcumulatedDistanceTravelled = %f\n",
        KeyFrameInfo.VelocityChange,
        KeyFrameInfo.LocalMapTrackingRatio,
        KeyFrameInfo.AcumulatedDistanceTravelled);

    return KeyFrameInfo;
}

enum class typeLocalMappingTimingStage : std::size_t
{
    QueueWait,
    IterationTotal,
    TopologyTransaction,
    RemoveMissingAssociations,
    ReconstructInertialState,
    AppendKeyFrame,
    SetAsKeyFrame,
    AddCovisibilityKeyFrame,
    CullRecentMapPoints,
    CreateNewMapPoints,
    CreateLocalMap,
    BundleAdjustLocal,
    CommitLocalMap,
    UpdateTrackingTrajectory,
    CullingTransaction,
    CullObservationEdges,
    CullLocalMap,
    MarkProcessed,
    UpdateVisualization,
    Count
};

static constexpr std::array<const char*,
    static_cast<std::size_t>(typeLocalMappingTimingStage::Count)>
LocalMappingTimingNames =
{
    "queue wait/dequeue",
    "complete iteration",
    "topology critical section",
    "remove missing associations",
    "reconstruct inertial state",
    "append keyframe",
    "set as keyframe",
    "add covisibility keyframe",
    "cull recent map points",
    "create new map points",
    "create local map snapshot",
    "local bundle adjustment",
    "commit local map",
    "update tracking trajectory",
    "culling critical section",
    "cull observation edges",
    "cull local map",
    "mark queue generation processed",
    "update visualization"
};

class typeLocalMappingScopedTimer
{
    public:
        explicit typeLocalMappingScopedTimer(typeTimingStatistics& Statistics)
            : Statistics(Statistics), Start(PantoClock::now())
        {}

        ~typeLocalMappingScopedTimer()
        {
            SL_AddTimingSample(
                    Statistics,
                    std::chrono::duration<fp64>(
                        PantoClock::now() - Start).count());
        }

    private:
        typeTimingStatistics& Statistics;
        PantoClock::time_point Start;
};

void SLPriv_LocalMappingThread(typeLocalMapData& LocalMap)
{
    std::array<typeTimingStatistics,
        static_cast<std::size_t>(typeLocalMappingTimingStage::Count)>
        Timing{};
    const auto Statistics = [&Timing](const typeLocalMappingTimingStage Stage)
        -> typeTimingStatistics&
    {
        return Timing[static_cast<std::size_t>(Stage)];
    };

    typeKeyFrame NewKeyFrame{};
    while(true)
    {
        bool HasKeyFrame = false;
        {
            typeLocalMappingScopedTimer Timer(
                    Statistics(typeLocalMappingTimingStage::QueueWait));
            HasKeyFrame = LocalMap.KeyFrameQueue->deque(NewKeyFrame);
        }
        if(!HasKeyFrame)
        {
            break;
        }

        typeLocalMappingScopedTimer IterationTimer(
                Statistics(typeLocalMappingTimingStage::IterationTotal));
        u64 ID = PANTO_ID_NOT_SET;
        std::vector<u64> NewPointIndexes;
        {
            typeLocalMappingScopedTimer TransactionTimer(
                    Statistics(
                        typeLocalMappingTimingStage::TopologyTransaction));
            // All topology changes are one map/graph transaction. The local
            // optimization snapshot is copied before releasing these locks.
            std::scoped_lock Lock(LocalMap.GlobalMap->Mutex, LocalMap.CovisibilityGraph->Mutex);

#if defined(DEBUG)
            MAP_AssertMapPointObservations(*LocalMap.GlobalMap);
#endif

            // The frame waited in the queue while local mapping was free to
            // cull map points referenced by its tracking snapshot.
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            RemoveMissingAssociations));
                SLPriv_RemoveMissingMapPointAssociations(
                        NewKeyFrame, *LocalMap.GlobalMap);
            }

#if defined(CONFIG_IMU)
            const u64 PreviousKFID = LocalMap.GlobalMap->KeyFrames.back().ID;

            // Repropagate the complete queued state from its actual temporal
            // parent before its pose is used for triangulation.
            const typeKeyFrame& CurrentReference =
                LocalMap.GlobalMap->KeyFrames[PreviousKFID];
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            ReconstructInertialState));
                SLPriv_ReconstructFrameFromReferenceKF(
                        NewKeyFrame,
                        CurrentReference,
                        NewKeyFrame.PreIntegrationData);
            }
#endif
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            AppendKeyFrame));
                ID = MAP_AppendKeyFrame(LocalMap.GlobalMap, NewKeyFrame);
            }

            typeKeyFrame& CurrentKeyFrame =
                LocalMap.GlobalMap->KeyFrames[ID];
#if defined(CONFIG_IMU)
            // Keyframe insertion is serialized by the map lock, so the
            // last keyframe observed before insertion is the temporal parent.
            CurrentKeyFrame.PreviousKFID = PreviousKFID;
            CurrentKeyFrame.TrackingReferencePose =
                LocalMap.GlobalMap->KeyFrames[PreviousKFID].Camera.Pose;
            CurrentKeyFrame.TrackingReferenceNavigationState =
                LocalMap.GlobalMap->KeyFrames[PreviousKFID].NavigationState;
            CurrentKeyFrame.HasTrackingReferenceState = true;
#endif

            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            SetAsKeyFrame));
                KEY_SetAsKeyFrame(
                        CurrentKeyFrame,
                        LocalMap.GlobalMap->MapPoints,
                        LocalMap.GlobalMap->KeyFrames,
                        PantoSLAM.Vocabulary);
            }
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            AddCovisibilityKeyFrame));
                GRAPH_AddKeyFrame(
                        LocalMap.CovisibilityGraph,
                        CurrentKeyFrame,
                        LocalMap.GlobalMap->MapPoints,
                        ID);
            }

#if defined(DEBUG)
            MAP_AssertGraphEqual(
                    *LocalMap.GlobalMap,
                    *LocalMap.CovisibilityGraph);
#endif

            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            CullRecentMapPoints));
                MAP_CullRecentMapPoints(
                        LocalMap.RecentMapPointIndexes,
                        LocalMap.GlobalMap,
                        LocalMap.CovisibilityGraph);
            }

#if defined(DEBUG)
            MAP_AssertGraphEqual(
                    *LocalMap.GlobalMap,
                    *LocalMap.CovisibilityGraph);
#endif

            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            CreateNewMapPoints));
                NewPointIndexes = MAP_CreateNewMapPoints(
                        LocalMap.GlobalMap,
                        CurrentKeyFrame,
                        LocalMap.CovisibilityGraph,
                        ID);
            }

#if defined(DEBUG)
            MAP_AssertGraphEqual(
                    *LocalMap.GlobalMap,
                    *LocalMap.CovisibilityGraph);
#endif
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            CreateLocalMap));
                LocalMap.LocalMap = MAP_CreateLocalMap(
                        *LocalMap.GlobalMap,
                        *LocalMap.CovisibilityGraph,
                        ID);
            }

            for(const u64 MapPointID : NewPointIndexes)
            {
                LocalMap.RecentMapPointIndexes.push_back(MapPointID);
            }
        }

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Created %llu new map points\n", static_cast<u64>(NewPointIndexes.size()));

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Running local bundle adjustment\n");
        bool LocalBACompleted = false;
        {
            typeLocalMappingScopedTimer Timer(
                    Statistics(typeLocalMappingTimingStage::
                        BundleAdjustLocal));
            LocalBACompleted = OP_BundleAdjustLocal(
                    LocalMap.LocalMap,
                    LocalMap.KeyFrameQueue);
        }

        if(LocalBACompleted)
        {
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            CommitLocalMap));
                (void) MAP_CommitLocalMap(
                        LocalMap.GlobalMap,
                        LocalMap.LocalMap);
            }

            // Keep already-recorded keyframe samples aligned with later local
            // BA corrections. This does not block tracking on the mapper.
            for(const typeKeyFrame& OptimizedKeyFrame :
                    LocalMap.LocalMap.KeyFrames)
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            UpdateTrackingTrajectory));
                SLPriv_UpdateTrackingTrajectoryPose(
                        OptimizedKeyFrame.Camera.TimeStamp,
                        OptimizedKeyFrame.Camera);
            }
        }

        LG_Log(LogSeverity::DBG, "[SLAMLoop] Culling local map\n");

        {
            typeLocalMappingScopedTimer TransactionTimer(
                    Statistics(typeLocalMappingTimingStage::
                        CullingTransaction));
            std::scoped_lock Lock(
                    LocalMap.GlobalMap->Mutex,
                    LocalMap.CovisibilityGraph->Mutex);
#if defined(DEBUG)
            MAP_AssertGraphEqual(
                    *LocalMap.GlobalMap,
                    *LocalMap.CovisibilityGraph);
#endif
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::
                            CullObservationEdges));
                MAP_CullObservationEdges(
                        LocalMap.GlobalMap,
                        LocalMap.CovisibilityGraph);
            }
#if !defined(CONFIG_IMU)
            {
                typeLocalMappingScopedTimer Timer(
                        Statistics(typeLocalMappingTimingStage::CullLocalMap));
                MAP_CullLocalMap(
                        LocalMap.GlobalMap,
                        LocalMap.CovisibilityGraph,
                        ID);
            }
#endif

#if defined(DEBUG)
            MAP_AssertGraphEqual(
                    *LocalMap.GlobalMap,
                    *LocalMap.CovisibilityGraph);
#endif
        }

        {
            typeLocalMappingScopedTimer Timer(
                    Statistics(typeLocalMappingTimingStage::MarkProcessed));
            LocalMap.KeyFrameQueue->MarkProcessed();
        }

#if !defined(DEBUG)
        {
            typeLocalMappingScopedTimer Timer(
                    Statistics(typeLocalMappingTimingStage::
                        UpdateVisualization));
            SLPriv_UpdateVisualization(
                    LocalMap.GlobalMap,
                    *LocalMap.VisualizationUpdateTiming);
        }
#endif
    }

    LG_EnableDataSummaryLoggingForCurrentThread(true);
    for(std::size_t StageIndex = 0; StageIndex < Timing.size(); StageIndex++)
    {
        if(Timing[StageIndex].Count == 0)
        {
            continue;
        }
        SL_LogTimingStatistics(
                LocalMappingTimingNames[StageIndex],
                Timing[StageIndex]);
    }
    LG_EnableDataSummaryLoggingForCurrentThread(false);
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
