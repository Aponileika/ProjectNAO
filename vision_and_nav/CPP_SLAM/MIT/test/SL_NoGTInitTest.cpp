#include "SL_SLAM.hpp"
#include <cstdarg>
#include <cstdlib>
#include <cstring>

static_assert(!PANTO_GROUNDTRUTH_INIT);
static_assert(!PANTO_DATASET_REALTIME_MODE);

// Exercise the actual dataset, tracking and mapper without changing the user's
// log files or starting the visualization/dense-reconstruction workers.
static bool AlignmentDataLogged = false;
static std::atomic<u64> TrackingWaitLogCount{0};
static std::atomic<u64> MappingLockCount{0};
static std::atomic<u64> MappingUnlockCount{0};
static std::atomic_bool MappingTopologyLogged{false};
static std::atomic_bool MappingCommitLogged{false};
static std::atomic_bool MappingCullingLogged{false};
void LG_EnableDataSummaryLoggingForCurrentThread(bool) {}
void LG_LogData(const char* Format, ...)
{
    char Message[512];
    va_list Arguments;
    va_start(Arguments, Format);
    std::vsnprintf(Message, sizeof(Message), Format, Arguments);
    va_end(Arguments);
    if(std::strstr(Message, "[SLAMTrackingMapWait]") != nullptr)
    {
        ++TrackingWaitLogCount;
    }
    if(std::strstr(Message, "thread=local_mapping") != nullptr)
    {
        if(std::strstr(Message, "event=locked") != nullptr) ++MappingLockCount;
        if(std::strstr(Message, "event=unlocked") != nullptr) ++MappingUnlockCount;
        if(std::strstr(Message, "stage=topology") != nullptr) MappingTopologyLogged = true;
        if(std::strstr(Message, "stage=commit_local_ba") != nullptr) MappingCommitLogged = true;
        if(std::strstr(Message, "stage=culling_and_initialization") != nullptr) MappingCullingLogged = true;
    }
}
void LG_Log(LogSeverity Severity, const char* Format, ...)
{
    if(Severity == LogSeverity::DATA && std::strstr(Format, "[SLAMGTAlignment] SE3 applied") != nullptr)
    {
        AlignmentDataLogged = true;
    }
    if(Severity != LogSeverity::ERROR && std::strstr(Format, "[StereoNEC Init]") == nullptr &&
       std::strstr(Format, "[SLAMGTAlignment]") == nullptr)
    {
        return;
    }
    va_list Arguments;
    va_start(Arguments, Format);
    std::vfprintf(stderr, Format, Arguments);
    va_end(Arguments);
}

#include "../src/CArenaAlloc.c"
#include "../src/TH_Threading.cpp"
#include "../src/EP_CorrespondingPoints.cpp"
#include "../src/FR_Frames.cpp"
#include "../src/SL_SLAM.cpp"
#include "../src/CM_Camera.cpp"
#include "../src/OP_BA.cpp"
#include "../src/PROJ_ProjectiveUtils.cpp"
#include "../src/PANTO_Utils.cpp"
#include "../src/KEY_KeyFrame.cpp"
#include "../src/PT_PantoImagePoints.cpp"
#include "../src/PT_PantoMapPoints.cpp"
#include "../src/MAP_Mapping.cpp"
#include "../src/DBOW3_DeepBagOfWords.cpp"
#include "../src/INIT_InitializeSLAM.cpp"
#include "../src/GRAPH_PantoGraph.cpp"
#include "../src/IMU_IMUReader.cpp"
#include "../src/IMU_PreIntegration.cpp"
#include "../src/GT_ReadGroundTruth.cpp"
#include "../src/GT_Alignment.cpp"
#include "../src/VIZ_Visualization.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

template<typename T> static T ReadBinary(std::ifstream& File)
{
    T Value{};
    File.read(reinterpret_cast<char*>(&Value), sizeof(Value));
    Require(File.good(), "binary export readable");
    return Value;
}

static Eigen::Vector3d ReadPosition(std::ifstream& File)
{
    const fp64 X = ReadBinary<fp64>(File);
    const fp64 Y = ReadBinary<fp64>(File);
    const fp64 Z = ReadBinary<fp64>(File);
    return {X, Y, Z};
}

static void TestFinalAlignmentAndExports()
{
    Require(PantoSLAM.TrackingTrajectory.size() == PantoSLAM.TrackingTrajectoryRotations.size(), "complete tracked orientations");
    std::vector<typeGTTrajectoryPose> Trajectory;
    for(std::size_t i = 0; i < PantoSLAM.TrackingTrajectory.size(); ++i)
    {
        Trajectory.push_back({PantoSLAM.TrackingTrajectoryTimeStamps[i], PantoSLAM.TrackingTrajectory[i],
                PantoSLAM.TrackingTrajectoryRotations[i]});
    }
    GroundTruth = GT_GetAllMeasurements();
    typeGTAlignmentResult Expected;
    Require(GT_ComputeTrajectoryAlignment(Trajectory, GroundTruth, CM_GetIntrinsics()->T_BS, Expected), "real dataset GT association and fit");
    const typeKeyFrame BeforeFrame = PantoSLAM.GlobalMap->KeyFrames[0];
    const typePantoMapPoint BeforePoint = *PantoSLAM.GlobalMap->MapPoints.begin();
    const Eigen::Vector3d BeforeProjection = BeforeFrame.Camera.Pose.R * (BeforePoint.Point.head<3>() / BeforePoint.Point.w()) +
        BeforeFrame.Camera.Pose.t;
    const Eigen::Vector3d BeforeGravity = *IMU_GetGravity();
    Require(SLPriv_AlignMapToGroundTruth(), "final alignment applied to real map");
    Require(AlignmentDataLogged, "alignment errors logged at DATA severity");
    for(std::size_t i = 0; i < Trajectory.size(); ++i)
    {
        Require(PantoSLAM.TrackingTrajectory[i].isApprox(Expected.Rotation * Trajectory[i].Position + Expected.Translation, 1e-10),
                "every processed frame position aligned in place");
        Require(PantoSLAM.TrackingTrajectoryRotations[i].toRotationMatrix().isApprox(
                    Expected.Rotation * Trajectory[i].Quaternion.toRotationMatrix(), 1e-10), "every processed orientation aligned");
    }
    const typeKeyFrame& Frame = PantoSLAM.GlobalMap->KeyFrames[0];
    const typePantoMapPoint& Point = PantoSLAM.GlobalMap->MapPoints[BeforePoint.ID];
    Require(Frame.NavigationState.Position.isApprox(Expected.Rotation * BeforeFrame.NavigationState.Position + Expected.Translation, 1e-10),
            "body position aligned");
    Require((Frame.NavigationState.Velocity - Expected.Rotation * BeforeFrame.NavigationState.Velocity).norm() < 1e-10,
            "world velocity rotated");
    Require(Frame.NavigationState.GyroBias == BeforeFrame.NavigationState.GyroBias, "body bias unchanged");
    Require(IMU_GetGravity()->isApprox(Expected.Rotation * BeforeGravity, 1e-10), "world gravity rotated");
    const Eigen::Vector3d Projection = Frame.Camera.Pose.R * (Point.Point.head<3>() / Point.Point.w()) + Frame.Camera.Pose.t;
    Require((Projection - BeforeProjection).norm() < 1e-9, "joint pose/landmark transform preserves camera geometry");
    Require(std::abs(Point.Point.norm() - 1.0) < 1e-12, "homogeneous point manifold preserved");

    const std::filesystem::path Root = PANTO_COLMAP_PATH;
    VIZ_WriteRealtime(*PantoSLAM.GlobalMap, PantoSLAM.TrackingTrajectory);
    std::ifstream Tracking(Root / "sparse/snapshots/0/tracking.bin", std::ios::binary);
    Require(ReadBinary<u64>(Tracking) == Trajectory.size(), "live snapshot contains full trajectory");
    Require(ReadPosition(Tracking).isApprox(PantoSLAM.TrackingTrajectory.front(), 1e-12), "live snapshot contains aligned frame positions");
    std::ifstream Images(Root / "sparse/snapshots/0/images.bin", std::ios::binary);
    Require(ReadBinary<u64>(Images) > 0 && ReadBinary<i32>(Images) == 1, "live root camera present");
    const fp64 Qw = ReadBinary<fp64>(Images);
    const Eigen::Vector3d Qxyz = ReadPosition(Images);
    const Eigen::Quaterniond LiveQuaternion(Qw, Qxyz.x(), Qxyz.y(), Qxyz.z());
    Require(LiveQuaternion.toRotationMatrix().isApprox(Frame.Camera.Pose.R, 1e-12), "live camera rotation aligned");
    Require(ReadPosition(Images).isApprox(Frame.Camera.Pose.t, 1e-12), "live camera translation aligned");

    typeDenseMapData DenseMap{};
    typeDenseKeyFrameMap DenseFrame{};
    DenseFrame.KeyFrameID = 0;
    DenseFrame.MapPoints.resize(3, 1);
    DenseFrame.MapPoints.col(0) = Eigen::Vector3f(0, 0, 2);
    DenseMap.KeyFrameMaps.emplace(0, DenseFrame);
    Require(VIZ_WriteReplay(*PantoSLAM.GlobalMap, *PantoSLAM.CovisibilityGraph, DenseMap,
                PantoSLAM.TrackingTrajectory, PantoSLAM.TrackingTrajectoryTimeStamps), "aligned post-run replay written");
    const auto ReplayPath = std::filesystem::directory_iterator(Root / "full_binaries")->path();
    std::ifstream Replay(ReplayPath, std::ios::binary);
    Replay.seekg(24); // PANTOVIZ v1 header
    char Magic[8]{};
    Replay.read(Magic, sizeof(Magic));
    Require(std::memcmp(Magic, "PVBLOCK1", 8) == 0, "replay first keyframe block");
    (void)ReadBinary<u64>(Replay); // block size
    Require(ReadBinary<u64>(Replay) == 0, "replay root keyframe ID");
    (void)ReadBinary<fp64>(Replay); // camera timestamp
    const u64 NumTracking = ReadBinary<u64>(Replay);
    const u64 NumDense = ReadBinary<u64>(Replay);
    Require(NumTracking > 0 && NumDense > 0, "replay contains trajectory and camera-local dense sample");
    Replay.seekg(3 * sizeof(i32) + sizeof(fp32) + 3 * sizeof(u64) + sizeof(i32) +
            2 * sizeof(u64) + 10 * sizeof(fp64), std::ios::cur); // voxel and camera calibration
    const fp64 ReplayQw = ReadBinary<fp64>(Replay);
    const Eigen::Vector3d ReplayQxyz = ReadPosition(Replay);
    const Eigen::Quaterniond ReplayQuaternion(ReplayQw, ReplayQxyz.x(), ReplayQxyz.y(), ReplayQxyz.z());
    Require(ReplayQuaternion.toRotationMatrix().isApprox(Frame.Camera.Pose.R, 1e-12), "saved camera rotation aligned");
    Require(ReadPosition(Replay).isApprox(Frame.Camera.Pose.t, 1e-12), "saved camera translation aligned");
    const u64 NameSize = ReadBinary<u64>(Replay);
    Replay.seekg(static_cast<std::streamoff>(NameSize), std::ios::cur);
    Require(ReadBinary<fp64>(Replay) == PantoSLAM.TrackingTrajectoryTimeStamps.front(), "saved sample timestamp retained");
    Require(ReadPosition(Replay).isApprox(PantoSLAM.TrackingTrajectory.front(), 1e-12), "saved tracking sample aligned");
    Replay.seekg(static_cast<std::streamoff>((NumTracking - 1) * 4 * sizeof(fp64)), std::ios::cur);
    const fp32 X = ReadBinary<fp32>(Replay);
    const fp32 Y = ReadBinary<fp32>(Replay);
    const fp32 Z = ReadBinary<fp32>(Replay);
    const Eigen::Vector3d ExpectedDense = Frame.Camera.Pose.Quaternion * Eigen::Vector3d(0, 0, 2) + Frame.Camera.Pose.tParametrization;
    Require((Eigen::Vector3d(X, Y, Z) - ExpectedDense).norm() < 1e-5, "saved dense geometry follows aligned camera pose");
}

int main()
{
    namespace fs = std::filesystem;
    const fs::path Workspace = fs::current_path();
    const fs::path TestDirectory = fs::temp_directory_path() /
        ("cpp_slam_no_gt_" + std::to_string(PantoClock::now().time_since_epoch().count()));
    fs::create_directories(TestDirectory / PANTO_COLMAP_PATH / "images");
    fs::create_directory_symlink(Workspace / "datasets", TestDirectory / "datasets");
    fs::create_symlink(Workspace / PANTO_VocabFilePath, TestDirectory / PANTO_VocabFilePath);
    fs::current_path(TestDirectory);
    CM_SetIntrinsics();
    SL_InitSlam();
    SLPriv_InitializeMap();
    Require(PantoSLAM.GlobalMap->KeyFrames.active_size() == 1, "one stereo seed without GT");
    Require(PantoSLAM.GlobalMap->MapPoints.active_size() >= PANTO_MIN_NUMBER_INITIAL_MAP_POINTS, "metric stereo landmarks");
    const typeKeyFrame& Seed = PantoSLAM.GlobalMap->KeyFrames[0];
    Require(!Seed.Frame.Descriptors.Descriptors.empty() && !Seed.Frame.RightDescriptors.Descriptors.empty(),
            "serialized replay prepares stereo features");
    Require(Seed.NavigationState.Rwb.allFinite() && Seed.NavigationState.Position.allFinite(), "seed navigation pose initialized");
    Require(Seed.NavigationState.GyroBias.isZero() && Seed.NavigationState.AccelorometerBias.isZero(), "seed biases do not use GT");
    Require(Seed.PreIntegrationData.DeltaT == 0.0 && Seed.PreIntegrationData.DeltaR.isIdentity(), "seed starts a fresh IMU interval");
    Require(std::abs(IMU_GetGravity()->norm() - 9.81) < 1e-9, "accelerometer gravity seed");
    for(const typePantoMapPoint& Point : PantoSLAM.GlobalMap->MapPoints)
    {
        Require(Point.Point.allFinite() && Point.NumObs == 2, "valid left/right seed observations");
        Require(Point.KeyFrameIDs.active_size() == 1 && Point.KeyFrameIDs[0] == 0, "seed observation association");
    }
    typeDecodedDataSetFrame CachedFrame{};
    CachedFrame.Gray = Seed.Frame.Frame;
    CachedFrame.RightGray = Seed.Frame.RightFrame;
    CachedFrame.SourcePath = Seed.ImagePath;
    CachedFrame.TimeStamp = Seed.Camera.TimeStamp;
    Require(FRPriv_FinalizeDataSetFrame(std::move(CachedFrame)).TimeStamp == Seed.Camera.TimeStamp,
            "existing cached frame can be reused");

    typePreIntegration PreIntegrationBetweenKF;
    IMU_InitializePreIntegration(PreIntegrationBetweenKF, Seed.NavigationState);
    typeKeyFrameQueue KeyFrameQueue;
    typeTrackingStatisticsQueue StatisticsQueue;
    typeSPSCQueue<typeDenseLocaMapData> DenseQueue;
    std::atomic_bool VisualOnly{true};
    typeTimingStatistics VisualizationTiming{};
    typeTrackingData TrackingData{
        .PreviousFrameData = PantoSLAM.PreviousFrameData,
        .TrackingMap = {},
        .NewFrame = {},
        .AccumulatedDistance = 0.0,
        .PosePrediction = PantoSLAM.NextFramePosePrediction,
        .TrackingStats = {},
        .VisualOnlyMode = VisualOnly,
        .TrackingStatQueue = &StatisticsQueue,
        .KeyFrameQueue = &KeyFrameQueue,
        .GlobalMap = PantoSLAM.GlobalMap,
        .CovisibilityGraph = PantoSLAM.CovisibilityGraph
    };
    typeLocalMapData LocalMappingData{
        .LocalMap = {},
        .RecentMapPointIndexes = {},
        .TrackingStatQueue = &StatisticsQueue,
        .DenseQueue = &DenseQueue,
        .VisualOnlyMode = VisualOnly,
        .KeyFrameQueue = &KeyFrameQueue,
        .GlobalMap = PantoSLAM.GlobalMap,
        .CovisibilityGraph = PantoSLAM.CovisibilityGraph,
        .VisualizationUpdateTiming = &VisualizationTiming
    };
    bool TrackingLost = false;
    i32 NumProcessedLoops = 0;
    u64 SkippedFrames = 0;
    u64 DenseMessages = 0;
    std::thread DenseConsumer([&DenseQueue, &DenseMessages]()
    {
        typeDenseLocaMapData Data;
        while(DenseQueue.deque(Data))
        {
            ++DenseMessages;
            Require(!Data.LocalKeyFramePoses.empty() &&
                    Data.LocalKeyFramePoses.front().KeyFrameID == Data.DenseData.KeyFrameID,
                    "dense pose snapshot starts with the queued keyframe");
            for(const typeDenseKeyFramePose& Frame : Data.LocalKeyFramePoses)
            {
                Require(Frame.Pose.R.allFinite() && Frame.Pose.t.allFinite(), "finite dense pose payload");
            }
        }
    });
    std::thread Mapper(SLPriv_LocalMappingThread, std::ref(LocalMappingData));
    SLPriv_TrackingThread(TrackingData, 100, PreIntegrationBetweenKF, TrackingLost, NumProcessedLoops, SkippedFrames);
    Mapper.join();
    DenseConsumer.join();
    Require(!TrackingLost && NumProcessedLoops == 100, "dataset tracking completes without GT");
    Require(TrackingWaitLogCount == static_cast<u64>(NumProcessedLoops), "one map-wait DATA summary per tracking loop");
    Require(MappingLockCount > 0 && MappingLockCount == MappingUnlockCount, "all mapping map acquisitions log paired releases");
    Require(MappingTopologyLogged && MappingCommitLogged && MappingCullingLogged, "mapping lock logs identify every processing stage");
    Require(!VisualOnly, "StereoNEC initialization completes from the stereo seed");
    Require(PantoSLAM.GlobalMap->Age >= PANTO_NUM_FORCED_KFS, "configured initialization window collected");
    Require(SkippedFrames == 0, "deterministic dataset replay");
    Require(DenseMessages == PantoSLAM.GlobalMap->Age - 1, "one pose-only dense payload per mapped keyframe");
    for(const typePantoMapPoint& Point : PantoSLAM.GlobalMap->MapPoints)
    {
        Require(Point.DescriptorDistances.Observations.size() == Point.KeyFrameIDs.size() &&
                Point.DescriptorDistances.ActiveIndices.size() == Point.KeyFrameIDs.active_size(),
                "live descriptor cache stays synchronized after mapping and culling");
        std::vector<typeDescriptor> Descriptors;
        for(std::size_t i = 0; i < Point.KeyFrameIDs.size(); ++i)
        {
            const auto& Observation = Point.DescriptorDistances.Observations[i];
            Require(Observation.Active == Point.KeyFrameIDs.contains(i), "removed observations invalidate cached slots");
            if(!Observation.Active) continue;
            Require(Observation.KeyFrameID == Point.KeyFrameIDs[i] && Observation.ImagePointID == Point.ImagePointIDs[i],
                    "cache identifies the current sparse observation");
            Descriptors.push_back(PantoSLAM.GlobalMap->KeyFrames[Point.KeyFrameIDs[i]].Points.ImagePoints[Point.ImagePointIDs[i]].Descriptor);
        }
        if(!Descriptors.empty())
        {
            std::vector<i32> Scratch;
            Require(Point.Descriptor == PT_CalculateNewDescriptor(Descriptors, Scratch),
                    "mapped representatives equal uncached smallest-median selection");
        }
    }
    for(const typePantoMapPoint& Point : TrackingData.TrackingMap.MapPoints)
    {
        Require(Point.DescriptorDistances.Observations.empty(), "tracking snapshots exclude descriptor history caches");
    }
    for(const typeKeyFrame& Frame : PantoSLAM.GlobalMap->KeyFrames)
    {
        Require(Frame.NavigationState.Position.allFinite() && Frame.NavigationState.Velocity.allFinite() &&
                Frame.NavigationState.GyroBias.allFinite(), "finite initialized navigation states");
    }
    TestFinalAlignmentAndExports();
    std::cout << "No-GT initialization, final alignment and exports passed on " << NumProcessedLoops << " dataset frames\n";
    fs::current_path(Workspace);
    fs::remove_all(TestDirectory);
}
