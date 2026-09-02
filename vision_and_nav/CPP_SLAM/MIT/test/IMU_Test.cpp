#include "Config.hpp"
#include "IMU_IMUReader.hpp"
#include "IMU_PreIntegration.hpp"
#include "GT_ReadGroundTruth.hpp"
#include "FR_Frames.hpp"
#include "VIZ_Visualization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../src/IMU_IMUReader.cpp"
#include "../src/IMU_PreIntegration.cpp"
#include "../src/GT_ReadGroundTruth.cpp"
#include "../src/LG_Logging.cpp"
#include "../src/CM_Camera.cpp"
#include "../src/PROJ_ProjectiveUtils.cpp"
#include "../src/VIZ_Visualization.cpp"

static bool IMUTest_GetTimestampedFrame(typePantoFrame& Frame)
{
    static const std::string CameraPath =
        std::string(PANTO_DATASET_BASE_PATH) +
        std::string(panto_dataset_path) +
        "/cam0";

    static std::ifstream FrameList(CameraPath + "/data.csv");
    static bool HeaderSkipped = false;

    if(!FrameList.is_open())
    {
        return false;
    }

    std::string Line;

    if(!HeaderSkipped)
    {
        std::getline(FrameList, Line);
        HeaderSkipped = true;
    }

    while(std::getline(FrameList, Line))
    {
        if(Line.empty() || Line[0] == '#')
        {
            continue;
        }

        std::stringstream Stream(Line);
        std::string TimeStampToken;
        std::string FileName;

        if(!std::getline(Stream, TimeStampToken, ',') ||
           !std::getline(Stream, FileName))
        {
            continue;
        }

        if(!FileName.empty() && FileName.back() == '\r')
        {
            FileName.pop_back();
        }

        const u64 TimeStampNS = std::stoull(TimeStampToken);

        Frame =
        {
            .Frame = {},
            .TimeStamp = static_cast<fp64>(TimeStampNS) * 1e-9,
            .Path = {}
        };

        return true;
    }

    return false;
}

static bool IMUTest_SynchronizeFrameAndGroundTruth(
        typePantoFrame& Frame,
        typeGroundTruth& GroundTruth)
{
    constexpr fp64 TimeTolerance = 1e-6;

    while(std::abs(Frame.TimeStamp - GroundTruth.TimeStamp) > TimeTolerance)
    {
        if(Frame.TimeStamp < GroundTruth.TimeStamp)
        {
            if(!IMUTest_GetTimestampedFrame(Frame))
            {
                return false;
            }
        }
        else
        {
            const typeGroundTruth NextGroundTruth = GT_GetMeasurement();

            if(NextGroundTruth.TimeStamp == 0.0)
            {
                return false;
            }

            GroundTruth = NextGroundTruth;
        }
    }

    return true;
}

static typeNavigationState IMUTest_GetNavigationState(const typeGroundTruth& GroundTruth)
{
    return
    {
        .Rwb = GroundTruth.Orientation.normalized().toRotationMatrix(),
        .Velocity = GroundTruth.Velocity,
        .Position = GroundTruth.Position,
        .GyroBias = GroundTruth.GyroBias,
        .AccelorometerBias = GroundTruth.AccelBias
    };
}

struct typeIMUTestOptions
{
    fp64 PlaybackSpeed = 1.0;
    u64 GroundTruthFrameInterval = 1;
    bool WaitForViewer = true;
    bool HelpRequested = false;
};

struct typeIMUTestIntegratedSample
{
    fp64 TimeStamp;
    Eigen::Vector3d Position;
    u64 CameraFramesSinceGroundTruth;
    u64 IntegrationStepsSinceGroundTruth;
};

static void IMUTest_PrintUsage(const char* ProgramName)
{
    std::cout
        << "Usage: " << ProgramName << " [playback_speed] [options]\n"
        << "\n"
        << "Options:\n"
        << "  --playback-speed <factor>       Visualization playback speed; 0 disables pacing\n"
        << "  --gt-frame-interval <frames>    Camera frames between GT state arrivals (default: 1)\n"
        << "  --no-wait                       Close the viewer when processing finishes\n"
        << "  --help                          Show this help\n"
        << "\n"
        << "The positional playback_speed argument is retained for compatibility.\n";
}

static bool IMUTest_ParseOptions(
        const int argc,
        char* argv[],
        typeIMUTestOptions& Options)
{
    bool PositionalPlaybackSpeedUsed = false;

    for(int i = 1; i < argc; i++)
    {
        const std::string Argument(argv[i]);

        if(Argument == "--help" || Argument == "-h")
        {
            IMUTest_PrintUsage(argv[0]);
            Options.HelpRequested = true;
            return false;
        }

        if(Argument == "--playback-speed")
        {
            if(++i >= argc)
            {
                std::cerr << "--playback-speed requires a value\n";
                return false;
            }

            Options.PlaybackSpeed = std::stod(argv[i]);
            PositionalPlaybackSpeedUsed = true;
            continue;
        }

        if(Argument == "--gt-frame-interval")
        {
            if(++i >= argc)
            {
                std::cerr << "--gt-frame-interval requires a value\n";
                return false;
            }

            Options.GroundTruthFrameInterval = std::stoull(argv[i]);
            continue;
        }

        if(Argument == "--no-wait")
        {
            Options.WaitForViewer = false;
            continue;
        }

        if(!Argument.empty() &&
           Argument[0] != '-' &&
           !PositionalPlaybackSpeedUsed)
        {
            Options.PlaybackSpeed = std::stod(Argument);
            PositionalPlaybackSpeedUsed = true;
            continue;
        }

        std::cerr << "Unknown argument: " << Argument << '\n';
        IMUTest_PrintUsage(argv[0]);
        return false;
    }

    if(Options.PlaybackSpeed < 0.0)
    {
        std::cerr << "Playback speed cannot be negative\n";
        return false;
    }

    if(Options.GroundTruthFrameInterval == 0)
    {
        std::cerr << "Ground-truth frame interval must be at least 1\n";
        return false;
    }

    return true;
}

int main(int argc, char* argv[])
{
    if(panto_dataset != Dataset::EUROC_MAV_VICON_ROOM1_EASY)
    {
        std::cerr << "IMU_Test requires PANTO_ACTIVE_DATASET "
                  << "EUROC_MAV_VICON_ROOM1_EASY\n";
        return 1;
    }

    typeIMUTestOptions Options;

    try
    {
        if(!IMUTest_ParseOptions(argc, argv, Options))
        {
            return Options.HelpRequested ? 0 : 1;
        }
    }
    catch(const std::exception& Error)
    {
        std::cerr << "Invalid command-line value: " << Error.what() << '\n';
        IMUTest_PrintUsage(argv[0]);
        return 1;
    }

    std::cout << "Ground-truth state interval = "
              << Options.GroundTruthFrameInterval
              << " camera frame(s)\n";

    const typeIMUIntrinsics* IMUIntrinsics = IMU_GetIntrinsics();

    std::cout
        << "[IMUIntrinsics] rate = "
        << IMUIntrinsics->RateHz
        << " Hz; gyroscope noise density = "
        << IMUIntrinsics->GyroscopeNoiseDensity
        << "; gyroscope random walk = "
        << IMUIntrinsics->GyroscopeRandomWalk
        << "; accelerometer noise density = "
        << IMUIntrinsics->AccelerometerNoiseDensity
        << "; accelerometer random walk = "
        << IMUIntrinsics->AccelerometerRandomWalk
        << '\n';

    VIZ_InitVisualization();

    const std::vector<typeGroundTruth> AllGroundTruth =
        GT_GetAllMeasurements();
    std::vector<Eigen::Vector3d> GroundTruthPositions;
    GroundTruthPositions.reserve(AllGroundTruth.size());

    for(const typeGroundTruth& GroundTruth : AllGroundTruth)
    {
        GroundTruthPositions.push_back(GroundTruth.Position);
    }

    VIZ_SetIMUTestGroundTruth(GroundTruthPositions);

    std::cout << "Ground-truth trajectory samples = "
              << GroundTruthPositions.size() << '\n';

    typePantoFrame CurrentFrame{};
    typeGroundTruth CurrentGroundTruth = GT_GetMeasurement();

    if(CurrentGroundTruth.TimeStamp == 0.0 ||
       !IMUTest_GetTimestampedFrame(CurrentFrame) ||
       !IMUTest_SynchronizeFrameAndGroundTruth(CurrentFrame, CurrentGroundTruth))
    {
        std::cerr << "Failed to synchronize the first frame and ground truth\n";
        return 1;
    }

    typeIMUMeasurement CurrentIMUMeasurement = IMU_GetMeasurement();

    while(CurrentIMUMeasurement.TimeStamp < CurrentFrame.TimeStamp)
    {
        CurrentIMUMeasurement = IMU_GetMeasurement();

        if(CurrentIMUMeasurement.TimeStamp == 0.0)
        {
            std::cerr << "IMU data ended before the first synchronized frame\n";
            return 1;
        }
    }

    IMU_NewNavigationStateArrival(
            IMUTest_GetNavigationState(CurrentGroundTruth));
    IMU_IngegrationStep(CurrentIMUMeasurement);
    CurrentIMUMeasurement = IMU_GetMeasurement();

    std::vector<Eigen::Vector3d> IntegratedPositions;
    IntegratedPositions.push_back(CurrentGroundTruth.Position);

    VIZ_WriteIMUTest(CurrentGroundTruth.Position);

    fp64 SquaredPositionErrorSum = 0.0;
    u64 NumGroundTruthIntervals = 0;
    u64 CameraFramesSinceGroundTruth = 0;
    u64 IntegrationStepsSinceGroundTruth = 0;
    u64 TotalIntegrationSteps = 0;
    u64 MinimumIntegrationSteps = std::numeric_limits<u64>::max();
    u64 MaximumIntegrationSteps = 0;
    u64 UnreportedTailCameraFrames = 0;
    u64 UnreportedTailIntegrationSteps = 0;

    typePantoFrame NextFrame{};
    Eigen::Vector3d LastIntegratedPosition = CurrentGroundTruth.Position;
    std::vector<typeIMUTestIntegratedSample> IntervalSamples;

    const auto RecordGroundTruthInterval =
        [&](const typeGroundTruth& NextGroundTruth, const bool IsPartial)
        {
            const fp64 PositionError =
                (LastIntegratedPosition - NextGroundTruth.Position).norm();
            const fp64 GroundTruthDeltaT =
                NextGroundTruth.TimeStamp - CurrentGroundTruth.TimeStamp;

            SquaredPositionErrorSum += PositionError * PositionError;
            NumGroundTruthIntervals++;
            TotalIntegrationSteps += IntegrationStepsSinceGroundTruth;
            MinimumIntegrationSteps = std::min(
                    MinimumIntegrationSteps,
                    IntegrationStepsSinceGroundTruth);
            MaximumIntegrationSteps = std::max(
                    MaximumIntegrationSteps,
                    IntegrationStepsSinceGroundTruth);

            std::cout
                << "[IMUTestGTInterval] index = "
                << NumGroundTruthIntervals
                << "; camera frames = "
                << CameraFramesSinceGroundTruth
                << "; IMU integration steps = "
                << IntegrationStepsSinceGroundTruth
                << "; duration = "
                << GroundTruthDeltaT
                << " s; position error = "
                << PositionError
                << " m; partial = "
                << (IsPartial ? "true" : "false")
                << '\n';

            IMU_NewNavigationStateArrival(
                    IMUTest_GetNavigationState(NextGroundTruth));

            CurrentGroundTruth = NextGroundTruth;
            CameraFramesSinceGroundTruth = 0;
            IntegrationStepsSinceGroundTruth = 0;
        };

    const auto RecordLastAvailableGroundTruth =
        [&](const typeGroundTruth& LastGroundTruth)
        {
            if(LastGroundTruth.TimeStamp <= CurrentGroundTruth.TimeStamp ||
               IntervalSamples.empty())
            {
                return false;
            }

            const auto ClosestSample = std::min_element(
                    IntervalSamples.begin(),
                    IntervalSamples.end(),
                    [&](const typeIMUTestIntegratedSample& Left,
                        const typeIMUTestIntegratedSample& Right)
                    {
                        return std::abs(
                            Left.TimeStamp - LastGroundTruth.TimeStamp) <
                            std::abs(
                            Right.TimeStamp - LastGroundTruth.TimeStamp);
                    });

            UnreportedTailCameraFrames =
                CameraFramesSinceGroundTruth -
                ClosestSample->CameraFramesSinceGroundTruth;
            UnreportedTailIntegrationSteps =
                IntegrationStepsSinceGroundTruth -
                ClosestSample->IntegrationStepsSinceGroundTruth;

            CameraFramesSinceGroundTruth =
                ClosestSample->CameraFramesSinceGroundTruth;
            IntegrationStepsSinceGroundTruth =
                ClosestSample->IntegrationStepsSinceGroundTruth;
            LastIntegratedPosition = ClosestSample->Position;

            RecordGroundTruthInterval(LastGroundTruth, true);
            IntervalSamples.clear();
            return true;
        };

    while(IMUTest_GetTimestampedFrame(NextFrame))
    {
        while(CurrentIMUMeasurement.TimeStamp != 0.0 &&
              CurrentIMUMeasurement.TimeStamp <= NextFrame.TimeStamp + 1e-6)
        {
            IMU_IngegrationStep(CurrentIMUMeasurement);
            IntegrationStepsSinceGroundTruth++;
            CurrentIMUMeasurement = IMU_GetMeasurement();
        }

        Eigen::Matrix3d IntegratedRwb;
        Eigen::Vector3d IntegratedPosition;
        IMU_GetPreIntegratedRt(IntegratedRwb, IntegratedPosition);
        LastIntegratedPosition = IntegratedPosition;

        IntegratedPositions.push_back(IntegratedPosition);

        VIZ_WriteIMUTest(IntegratedPosition);

        CameraFramesSinceGroundTruth++;
        IntervalSamples.push_back(
        {
            .TimeStamp = NextFrame.TimeStamp,
            .Position = IntegratedPosition,
            .CameraFramesSinceGroundTruth = CameraFramesSinceGroundTruth,
            .IntegrationStepsSinceGroundTruth =
                IntegrationStepsSinceGroundTruth
        });

        if(CameraFramesSinceGroundTruth >= Options.GroundTruthFrameInterval)
        {
            typeGroundTruth NextGroundTruth = CurrentGroundTruth;

            if(!IMUTest_SynchronizeFrameAndGroundTruth(
                    NextFrame,
                    NextGroundTruth))
            {
                RecordLastAvailableGroundTruth(NextGroundTruth);
                break;
            }

            RecordGroundTruthInterval(NextGroundTruth, false);
            IntervalSamples.clear();
        }

        if(Options.PlaybackSpeed > 0.0 && PANTO_CAMERA_RATE_HZ > 0.0)
        {
            std::this_thread::sleep_for(
                    std::chrono::duration<fp64>(
                        1.0 / (PANTO_CAMERA_RATE_HZ * Options.PlaybackSpeed)));
        }

        CurrentFrame = std::move(NextFrame);

        if(CurrentIMUMeasurement.TimeStamp == 0.0)
        {
            break;
        }
    }

    if(CameraFramesSinceGroundTruth > 0)
    {
        typeGroundTruth FinalGroundTruth = CurrentGroundTruth;

        if(IMUTest_SynchronizeFrameAndGroundTruth(
                CurrentFrame,
                FinalGroundTruth))
        {
            RecordGroundTruthInterval(FinalGroundTruth, true);
            IntervalSamples.clear();
        }
        else
        {
            RecordLastAvailableGroundTruth(FinalGroundTruth);
        }
    }

    const fp64 PositionRMSE = NumGroundTruthIntervals > 0
        ? std::sqrt(
            SquaredPositionErrorSum /
            static_cast<fp64>(NumGroundTruthIntervals))
        : 0.0;

    const fp64 MeanIntegrationSteps = NumGroundTruthIntervals > 0
        ? static_cast<fp64>(TotalIntegrationSteps) /
            static_cast<fp64>(NumGroundTruthIntervals)
        : 0.0;

    std::cout << "[IMUTestSummary] GT intervals = "
              << NumGroundTruthIntervals
              << "; configured camera-frame interval = "
              << Options.GroundTruthFrameInterval
              << "; IMU steps min/mean/max = "
              << (NumGroundTruthIntervals > 0 ? MinimumIntegrationSteps : 0)
              << "/" << MeanIntegrationSteps
              << "/" << MaximumIntegrationSteps
              << "; unreported tail camera frames/IMU steps = "
              << UnreportedTailCameraFrames
              << "/" << UnreportedTailIntegrationSteps
              << "; trajectory positions = "
              << IntegratedPositions.size()
              << "; position RMSE = " << PositionRMSE << " m\n";

    VIZ_FlushIMUTest();

    if(Options.WaitForViewer)
    {
        std::cout << "Visualization complete; press Enter to close it.\n";
        std::cin.get();
    }

    VIZ_StopViewer();

    return NumGroundTruthIntervals > 0 ? 0 : 1;
}
