/*
 * Mac receiver for the Raspberry Pi camera streamer.
 *
 * Build from the CPP_SLAM repository root:
 *   make -C MIT/PiImagereader
 *
 * Monocular examples:
 *   ./MIT/PiImagereader/PantoReceiver
 *   ./MIT/PiImagereader/PantoReceiver --camera-count 1 --camera 1
 *
 * Stereo example (server shown as left, client shown as right):
 *   ./MIT/PiImagereader/PantoReceiver --camera-count 2 \
 *       --server-camera 0 --client-camera 1 --left-role server
 *
 * Options:
 *   --camera-count 1|2        Number of camera streams to receive.
 *   --camera INDEX            Camera used in monocular mode (default: 0).
 *   --server-camera INDEX     Sync-server camera in stereo mode (default: 0).
 *   --client-camera INDEX     Sync-client camera in stereo mode (default: 1).
 *   --left-role server|client Select which sync role is displayed as left.
 *   --frames COUNT            Frames received per camera (default: 600).
 *   --mode display|calib|data|yuv|demo
 *                             Display video, capture images, or request full
 *                             YUV420 for the left stereo camera.
 *   --SGBMDoublePass true|false
 *                             Enable slower right-to-left confidence matching
 *                             in demo mode (default: false).
 *   --calib-dir PATH          Calibration output directory
 *                             (default: calibration_images).
 *   --mac-ip ADDRESS          Mac address advertised to the Pi
 *                             (default: 10.241.223.208).
 *   --help, -h                Print runtime usage information.
 *
 * The receiver starts ~/PantoPI/ImageIO on
 * pantopilot@pantopilotstereoraspberry.local over SSH.
 */

#include <arpa/inet.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <new>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

static constexpr uint16_t DEFAULT_PORT = 50555;
static constexpr size_t DEFAULT_NUM_FRAMES = 600;
static constexpr uint32_t FRAME_WIDTH = 640;
static constexpr uint32_t FRAME_HEIGHT = 480;
static constexpr uint32_t FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT;
static constexpr uint32_t YUV420_FRAME_SIZE = FRAME_SIZE * 3 / 2;
static constexpr double TARGET_FPS = 20.0;
static constexpr const char* DEFAULT_MAC_IP = "10.241.223.208";
static constexpr const char* PI_SSH_HOST =
    "pantopilot@pantopilotstereoraspberry.local";
static constexpr const char* PI_PROGRAM = "~/PantoPI/ImageIO";
static constexpr uint64_t STREAM_HELLO_MAGIC = 0x50414E544F43414DULL;
static constexpr uint64_t STEREO_STREAM_HELLO_MAGIC = 0x50414E544F535452ULL;
static constexpr uint64_t CLOCK_SYNC_MAGIC = 0x50414E544F434C4BULL;
static constexpr uint32_t CLOCK_SYNC_SAMPLES = 8;
static constexpr uint64_t DEMO_POINT_CLOUD_MAGIC = 0x50414E544F50434CULL;

enum class typeSyncRole : uint32_t
{
    Off = 0,
    Server = 1,
    Client = 2
};

enum class typeLeftRole
{
    Server,
    Client
};

enum class typeReceiverMode
{
    Display,
    Calibration,
    DataCollection,
    YUVPreview,
    Demo
};

static bool UsesLeftYUV(const typeReceiverMode Mode)
{
    return Mode == typeReceiverMode::YUVPreview ||
           Mode == typeReceiverMode::Demo;
}

struct typeProgramOptions
{
    size_t CameraCount = 1;
    size_t CameraIndex = 0;
    size_t ServerCameraIndex = 0;
    size_t ClientCameraIndex = 1;
    typeLeftRole LeftRole = typeLeftRole::Server;
    size_t NumFrames = DEFAULT_NUM_FRAMES;
    std::string MacIPAddress = DEFAULT_MAC_IP;
    typeReceiverMode Mode = typeReceiverMode::Display;
    bool SGBMDoublePass = false;
    std::filesystem::path CalibrationDirectory = "calibration_images";
    std::filesystem::path DataDirectory;
};

struct typeDemoVisualizationRuntime
{
    std::filesystem::path OutputDirectory;
    std::filesystem::path FramePath;
    std::filesystem::path TemporaryFramePath;
    pid_t ViewerPID = -1;
    uint64_t FrameSequence = 0;
};

struct typeStreamHello
{
    uint64_t Magic;
    uint32_t CameraIndex;
    uint32_t SyncRole;
    uint32_t Width;
    uint32_t Height;
};

struct typeFrameHeader
{
    uint64_t TimestampNs;
    uint32_t Sequence;
    uint32_t Width;
    uint32_t Height;
    uint32_t PayloadSize;
};

struct typeStereoStreamHello
{
    uint64_t Magic;
    uint32_t ServerCameraIndex;
    uint32_t ClientCameraIndex;
    uint32_t Width;
    uint32_t Height;
    uint32_t ServerPayloadSize;
    uint32_t ClientPayloadSize;
};

struct typeGrayscaleStereoStreamHello
{
    uint64_t Magic;
    uint32_t ServerCameraIndex;
    uint32_t ClientCameraIndex;
    uint32_t Width;
    uint32_t Height;
};

struct typeStereoFrameHeader
{
    uint64_t PairSequence;
    uint64_t ServerTimestampNs;
    uint64_t ClientTimestampNs;
    uint32_t ServerSequence;
    uint32_t ClientSequence;
    uint32_t Width;
    uint32_t Height;
    uint32_t ServerPayloadSize;
    uint32_t ClientPayloadSize;
};

struct typeGrayscaleStereoFrameHeader
{
    uint64_t PairSequence;
    uint64_t ServerTimestampNs;
    uint64_t ClientTimestampNs;
    uint32_t ServerSequence;
    uint32_t ClientSequence;
    uint32_t Width;
    uint32_t Height;
    uint32_t PayloadSizePerImage;
};

static_assert(sizeof(typeStereoStreamHello) == 32);
static_assert(sizeof(typeGrayscaleStereoStreamHello) == 24);
static_assert(sizeof(typeStereoFrameHeader) == 48);
static_assert(sizeof(typeGrayscaleStereoFrameHeader) == 48);

struct typeClockSyncRequest
{
    uint64_t Magic;
    uint32_t Sequence;
    uint32_t Reserved;
    uint64_t MacSendTimeNs;
};

struct typeClockSyncResponse
{
    uint64_t Magic;
    uint32_t Sequence;
    uint32_t Reserved;
    uint64_t PiReceiveTimeNs;
    uint64_t PiSendTimeNs;
};

struct typeStats
{
    double Mean = 0.0;
    double StdDev = 0.0;
};

struct typeStreamRuntime
{
    int SocketFD = -1;
    typeStreamHello Hello{};
    std::string Name;
    double PiMinusMacOffsetNs = 0.0;
    double ClockSyncRoundTripTimeMs = 0.0;

    std::mutex ImageMutex;
    cv::Mat LatestImage;
    cv::Mat LatestLuminanceImage;
    uint64_t LatestImageVersion = 0;

    std::atomic<bool> Finished{false};
    bool Failed = false;
    std::string FailureReason;

    size_t FramesReceived = 0;
    uint64_t DroppedSequenceFrames = 0;
    std::vector<double> ArrivalIntervalsMs;
    std::vector<double> CameraIntervalsMs;
    std::vector<double> CaptureToMatLatencyMs;
};

struct typeStereoRuntime
{
    int SocketFD = -1;
    std::mutex ImageMutex;
    uint64_t LatestImageVersion = 0;
    std::atomic<bool> Finished{false};
    bool Failed = false;
    std::string FailureReason;
    uint32_t ServerPayloadSize = FRAME_SIZE;
    uint32_t ClientPayloadSize = FRAME_SIZE;
    uint64_t MissingPairSequences = 0;
    std::vector<double> TimestampDeltaMs;
};

struct typeCalibrationRuntime
{
    bool Enabled = false;
    bool ServerIsLeft = true;
    std::filesystem::path Directory;
    uint64_t NextCaptureTimestampNs = 0;
    std::atomic<size_t> CapturesSaved{0};
};

struct typeDataCollectionRuntime
{
    struct typePairMetadata
    {
        uint64_t PairSequence;
        uint64_t LeftTimestampNs;
        uint64_t RightTimestampNs;
    };

    bool Enabled = false;
    bool ServerIsLeft = true;
    std::filesystem::path Directory;
    std::vector<typePairMetadata> Pairs;
    std::vector<uint8_t> ImageData;
    size_t PairsSaved = 0;
};

static void PrintUsage(const char* Executable)
{
    std::cerr
        << "Usage:\n"
        << "  " << Executable
        << " [--camera-count 1] [--camera INDEX]\n"
        << "  " << Executable
        << " --camera-count 2 [--server-camera INDEX]"
           " [--client-camera INDEX] [--left-role server|client]\n"
        << "Optional: --frames COUNT --mac-ip ADDRESS"
           " --mode display|calib|data|yuv|demo --calib-dir PATH"
           " --SGBMDoublePass true|false\n\n"
        << "Defaults: one camera (index 0); in stereo, server camera 0 is left"
           " and client camera 1 is right.\n";
}

static bool ParseSizeArgument(const char* Text, size_t& Value)
{
    if(Text == nullptr || *Text == '\0' || *Text == '-')
        return false;

    errno = 0;
    char* End = nullptr;
    const unsigned long long Parsed = std::strtoull(Text, &End, 10);

    if(errno != 0 || End == Text || *End != '\0' ||
       Parsed > std::numeric_limits<size_t>::max())
    {
        return false;
    }

    Value = static_cast<size_t>(Parsed);
    return true;
}

static bool ParseProgramOptions(const int argc, char* argv[],
        typeProgramOptions& Options)
{
    for(int ArgumentIndex = 1; ArgumentIndex < argc; ++ArgumentIndex)
    {
        const std::string_view Argument(argv[ArgumentIndex]);

        if(Argument == "--camera-count" || Argument == "--camera" ||
           Argument == "--server-camera" || Argument == "--client-camera" ||
           Argument == "--frames")
        {
            if(++ArgumentIndex >= argc)
                return false;

            size_t Value = 0;
            if(!ParseSizeArgument(argv[ArgumentIndex], Value))
                return false;

            if(Argument == "--camera-count")
                Options.CameraCount = Value;
            else if(Argument == "--camera")
                Options.CameraIndex = Value;
            else if(Argument == "--server-camera")
                Options.ServerCameraIndex = Value;
            else if(Argument == "--client-camera")
                Options.ClientCameraIndex = Value;
            else if(Argument == "--frames")
                Options.NumFrames = Value;

            continue;
        }

        if(Argument == "--left-role")
        {
            if(++ArgumentIndex >= argc)
                return false;

            const std::string_view Role(argv[ArgumentIndex]);
            if(Role == "server")
                Options.LeftRole = typeLeftRole::Server;
            else if(Role == "client")
                Options.LeftRole = typeLeftRole::Client;
            else
                return false;

            continue;
        }

        if(Argument == "--mode")
        {
            if(++ArgumentIndex >= argc)
                return false;

            const std::string_view Mode(argv[ArgumentIndex]);
            if(Mode == "display")
                Options.Mode = typeReceiverMode::Display;
            else if(Mode == "calib")
                Options.Mode = typeReceiverMode::Calibration;
            else if(Mode == "data")
                Options.Mode = typeReceiverMode::DataCollection;
            else if(Mode == "yuv")
                Options.Mode = typeReceiverMode::YUVPreview;
            else if(Mode == "demo")
                Options.Mode = typeReceiverMode::Demo;
            else
                return false;

            continue;
        }

        if(Argument == "--SGBMDoublePass" ||
           Argument == "--sgbm-double-pass")
        {
            if(++ArgumentIndex >= argc)
                return false;

            const std::string_view Value(argv[ArgumentIndex]);
            if(Value == "true" || Value == "1")
                Options.SGBMDoublePass = true;
            else if(Value == "false" || Value == "0")
                Options.SGBMDoublePass = false;
            else
                return false;

            continue;
        }

        if(Argument == "--calib-dir")
        {
            if(++ArgumentIndex >= argc)
                return false;
            Options.CalibrationDirectory = argv[ArgumentIndex];
            continue;
        }

        if(Argument == "--mac-ip")
        {
            if(++ArgumentIndex >= argc)
                return false;
            Options.MacIPAddress = argv[ArgumentIndex];
            continue;
        }

        if(Argument == "--help" || Argument == "-h")
        {
            PrintUsage(argv[0]);
            std::exit(0);
        }

        return false;
    }

    if((Options.CameraCount != 1 && Options.CameraCount != 2) ||
       Options.NumFrames == 0 || Options.MacIPAddress.empty() ||
       Options.CalibrationDirectory.empty())
    {
        return false;
    }

    if(Options.CameraCount == 2 &&
       Options.ServerCameraIndex == Options.ClientCameraIndex)
    {
        std::cerr << "Server and client camera indices must be different\n";
        return false;
    }

    if(Options.Mode == typeReceiverMode::DataCollection &&
       Options.CameraCount != 2)
    {
        std::cerr << "Data mode requires --camera-count 2\n";
        return false;
    }

    if(UsesLeftYUV(Options.Mode) &&
       Options.CameraCount != 2)
    {
        std::cerr << "YUV mode requires --camera-count 2\n";
        return false;
    }

    return true;
}

static const char* SyncRoleName(const typeSyncRole Role)
{
    switch(Role)
    {
        case typeSyncRole::Off: return "off";
        case typeSyncRole::Server: return "server";
        case typeSyncRole::Client: return "client";
    }

    return "invalid";
}

static typeStats CalculateStats(const std::vector<double>& Values)
{
    if(Values.empty())
        return {};

    double Sum = 0.0;
    double SumSquared = 0.0;

    for(const double Value : Values)
    {
        Sum += Value;
        SumSquared += Value * Value;
    }

    const double N = static_cast<double>(Values.size());
    const double Mean = Sum / N;
    double StdDev = 0.0;

    if(Values.size() > 1)
    {
        const double Variance =
            (SumSquared - (Sum * Sum) / N) / (N - 1.0);
        StdDev = std::sqrt(std::max(0.0, Variance));
    }

    return {.Mean = Mean, .StdDev = StdDev};
}

static bool RecvAll(const int SocketFD, void* Data, size_t Size)
{
    uint8_t* Pointer = static_cast<uint8_t*>(Data);

    while(Size > 0)
    {
        const ssize_t BytesReceived = recv(SocketFD, Pointer, Size, 0);

        if(BytesReceived < 0)
        {
            if(errno == EINTR)
                continue;
            return false;
        }

        if(BytesReceived == 0)
            return false;

        Pointer += static_cast<size_t>(BytesReceived);
        Size -= static_cast<size_t>(BytesReceived);
    }

    return true;
}

static bool SendAll(const int SocketFD, const void* Data, size_t Size)
{
    const uint8_t* Pointer = static_cast<const uint8_t*>(Data);

    while(Size > 0)
    {
        const ssize_t BytesSent = send(SocketFD, Pointer, Size, 0);

        if(BytesSent < 0)
        {
            if(errno == EINTR)
                continue;
            return false;
        }

        if(BytesSent == 0)
            return false;

        Pointer += static_cast<size_t>(BytesSent);
        Size -= static_cast<size_t>(BytesSent);
    }

    return true;
}

static uint64_t MonotonicTimeNs()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

static bool SynchronizeClocks(const int SocketFD, double& PiMinusMacOffsetNs,
        double& BestRoundTripTimeMs)
{
    uint64_t BestRoundTripTimeNs = std::numeric_limits<uint64_t>::max();
    double BestOffsetNs = 0.0;

    for(uint32_t Sequence = 0; Sequence < CLOCK_SYNC_SAMPLES; ++Sequence)
    {
        const uint64_t MacSendTimeNs = MonotonicTimeNs();
        const typeClockSyncRequest Request
        {
            .Magic = CLOCK_SYNC_MAGIC,
            .Sequence = Sequence,
            .Reserved = 0,
            .MacSendTimeNs = MacSendTimeNs
        };

        if(!SendAll(SocketFD, &Request, sizeof(Request)))
            return false;

        typeClockSyncResponse Response{};
        if(!RecvAll(SocketFD, &Response, sizeof(Response)))
            return false;

        const uint64_t MacReceiveTimeNs = MonotonicTimeNs();

        if(Response.Magic != CLOCK_SYNC_MAGIC ||
           Response.Sequence != Sequence ||
           Response.PiSendTimeNs < Response.PiReceiveTimeNs)
        {
            return false;
        }

        const uint64_t PiProcessingTimeNs =
            Response.PiSendTimeNs - Response.PiReceiveTimeNs;
        const uint64_t TotalTimeNs = MacReceiveTimeNs - MacSendTimeNs;
        const uint64_t RoundTripTimeNs = TotalTimeNs >= PiProcessingTimeNs ?
            TotalTimeNs - PiProcessingTimeNs : 0;

        const double OffsetNs = 0.5 *
            (static_cast<double>(Response.PiReceiveTimeNs) -
             static_cast<double>(MacSendTimeNs) +
             static_cast<double>(Response.PiSendTimeNs) -
             static_cast<double>(MacReceiveTimeNs));

        if(RoundTripTimeNs < BestRoundTripTimeNs)
        {
            BestRoundTripTimeNs = RoundTripTimeNs;
            BestOffsetNs = OffsetNs;
        }
    }

    PiMinusMacOffsetNs = BestOffsetNs;
    BestRoundTripTimeMs = static_cast<double>(BestRoundTripTimeNs) / 1e6;
    return BestRoundTripTimeNs != std::numeric_limits<uint64_t>::max();
}

static int CreateServerSocket(const uint16_t Port, const int Backlog)
{
    const int ServerFD = socket(AF_INET, SOCK_STREAM, 0);

    if(ServerFD < 0)
    {
        perror("socket");
        return -1;
    }

    int Enable = 1;
    setsockopt(ServerFD, SOL_SOCKET, SO_REUSEADDR, &Enable, sizeof(Enable));

    sockaddr_in Address{};
    Address.sin_family = AF_INET;
    Address.sin_addr.s_addr = INADDR_ANY;
    Address.sin_port = htons(Port);

    if(bind(ServerFD, reinterpret_cast<sockaddr*>(&Address),
            sizeof(Address)) < 0)
    {
        perror("bind");
        close(ServerFD);
        return -1;
    }

    if(listen(ServerFD, Backlog) < 0)
    {
        perror("listen");
        close(ServerFD);
        return -1;
    }

    return ServerFD;
}

static bool StartPiProgram(const typeProgramOptions& Options)
{
    std::string Command =
        "ssh -T " + std::string(PI_SSH_HOST) + " '" + PI_PROGRAM + " " +
        Options.MacIPAddress;

    if(Options.CameraCount == 1)
    {
        Command += " --camera-count 1 --camera " +
            std::to_string(Options.CameraIndex) + " --sync off";
    }
    else
    {
        Command += " --camera-count 2 --server-camera " +
            std::to_string(Options.ServerCameraIndex) +
            " --client-camera " + std::to_string(Options.ClientCameraIndex);
        if(UsesLeftYUV(Options.Mode))
        {
            Command += " --mode left-yuv --left-role ";
            Command += Options.LeftRole == typeLeftRole::Server ?
                "server" : "client";
        }
    }

    Command += "' &";
    return std::system(Command.c_str()) == 0;
}

static bool ReadStreamHello(const int SocketFD, typeStreamHello& Hello)
{
    if(!RecvAll(SocketFD, &Hello, sizeof(Hello)))
        return false;

    return Hello.Magic == STREAM_HELLO_MAGIC &&
           Hello.Width == FRAME_WIDTH && Hello.Height == FRAME_HEIGHT &&
           Hello.SyncRole <= static_cast<uint32_t>(typeSyncRole::Client);
}

static bool InitializeConnection(typeStreamRuntime& Stream, const int SocketFD,
        const typeStreamHello& Hello)
{
    Stream.SocketFD = SocketFD;
    Stream.Hello = Hello;
    Stream.LatestImage = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);

    if(!SynchronizeClocks(SocketFD, Stream.PiMinusMacOffsetNs,
            Stream.ClockSyncRoundTripTimeMs))
    {
        std::cerr << "Clock synchronization failed for camera "
                  << Hello.CameraIndex << '\n';
        return false;
    }

    std::cout
        << "Camera " << Hello.CameraIndex
        << " connected; sync role = "
        << SyncRoleName(static_cast<typeSyncRole>(Hello.SyncRole))
        << "; best clock-sync RTT = "
        << Stream.ClockSyncRoundTripTimeMs << " ms\n";

    return true;
}

static bool AcceptStreams(const int ServerFD, const typeProgramOptions& Options,
        typeStreamRuntime& First, typeStreamRuntime& Second)
{
    for(size_t ConnectionIndex = 0;
        ConnectionIndex < Options.CameraCount;
        ++ConnectionIndex)
    {
        const int PiFD = accept(ServerFD, nullptr, nullptr);
        if(PiFD < 0)
        {
            perror("accept");
            return false;
        }

        typeStreamHello Hello{};
        if(!ReadStreamHello(PiFD, Hello))
        {
            std::cerr << "Invalid stream-identification message from Pi\n";
            close(PiFD);
            return false;
        }

        typeStreamRuntime* Destination = nullptr;

        if(Options.CameraCount == 1)
        {
            if(Hello.CameraIndex != Options.CameraIndex ||
               Hello.SyncRole != static_cast<uint32_t>(typeSyncRole::Off))
            {
                std::cerr << "Unexpected mono camera stream: camera "
                          << Hello.CameraIndex << ", role " << Hello.SyncRole
                          << '\n';
                close(PiFD);
                return false;
            }
            Destination = &First;
        }
        else if(Hello.SyncRole == static_cast<uint32_t>(typeSyncRole::Server) &&
                Hello.CameraIndex == Options.ServerCameraIndex)
        {
            Destination = &First;
        }
        else if(Hello.SyncRole == static_cast<uint32_t>(typeSyncRole::Client) &&
                Hello.CameraIndex == Options.ClientCameraIndex)
        {
            Destination = &Second;
        }
        else
        {
            std::cerr << "Unexpected stereo camera stream: camera "
                      << Hello.CameraIndex << ", role " << Hello.SyncRole
                      << '\n';
            close(PiFD);
            return false;
        }

        if(Destination->SocketFD >= 0)
        {
            std::cerr << "Duplicate Pi camera stream\n";
            close(PiFD);
            return false;
        }

        if(!InitializeConnection(*Destination, PiFD, Hello))
        {
            close(PiFD);
            Destination->SocketFD = -1;
            return false;
        }
    }

    return First.SocketFD >= 0 &&
           (Options.CameraCount == 1 || Second.SocketFD >= 0);
}

static bool AcceptStereoStream(const int ServerFD,
        const typeProgramOptions& Options, typeStereoRuntime& Stereo,
        typeStreamRuntime& ServerStream, typeStreamRuntime& ClientStream)
{
    const int PiFD = accept(ServerFD, nullptr, nullptr);
    if(PiFD < 0)
    {
        perror("accept");
        return false;
    }

    const uint32_t ExpectedServerPayloadSize =
        UsesLeftYUV(Options.Mode) &&
        Options.LeftRole == typeLeftRole::Server ?
            YUV420_FRAME_SIZE : FRAME_SIZE;
    const uint32_t ExpectedClientPayloadSize =
        UsesLeftYUV(Options.Mode) &&
        Options.LeftRole == typeLeftRole::Client ?
            YUV420_FRAME_SIZE : FRAME_SIZE;
    uint32_t ServerCameraIndex = 0;
    uint32_t ClientCameraIndex = 0;
    uint32_t ServerPayloadSize = FRAME_SIZE;
    uint32_t ClientPayloadSize = FRAME_SIZE;

    bool ValidHello = false;
    if(UsesLeftYUV(Options.Mode))
    {
        typeStereoStreamHello Hello{};
        ValidHello = RecvAll(PiFD, &Hello, sizeof(Hello)) &&
            Hello.Magic == STEREO_STREAM_HELLO_MAGIC &&
            Hello.Width == FRAME_WIDTH && Hello.Height == FRAME_HEIGHT;
        ServerCameraIndex = Hello.ServerCameraIndex;
        ClientCameraIndex = Hello.ClientCameraIndex;
        ServerPayloadSize = Hello.ServerPayloadSize;
        ClientPayloadSize = Hello.ClientPayloadSize;
    }
    else
    {
        typeGrayscaleStereoStreamHello Hello{};
        ValidHello = RecvAll(PiFD, &Hello, sizeof(Hello)) &&
            Hello.Magic == STEREO_STREAM_HELLO_MAGIC &&
            Hello.Width == FRAME_WIDTH && Hello.Height == FRAME_HEIGHT;
        ServerCameraIndex = Hello.ServerCameraIndex;
        ClientCameraIndex = Hello.ClientCameraIndex;
    }

    if(!ValidHello || ServerCameraIndex != Options.ServerCameraIndex ||
       ClientCameraIndex != Options.ClientCameraIndex ||
       ServerPayloadSize != ExpectedServerPayloadSize ||
       ClientPayloadSize != ExpectedClientPayloadSize)
    {
        std::cerr << "Invalid stereo stream identification or payload format\n";
        close(PiFD);
        return false;
    }

    double PiMinusMacOffsetNs = 0.0;
    double ClockSyncRoundTripTimeMs = 0.0;
    if(!SynchronizeClocks(PiFD, PiMinusMacOffsetNs,
            ClockSyncRoundTripTimeMs))
    {
        std::cerr << "Clock synchronization failed for stereo stream\n";
        close(PiFD);
        return false;
    }

    Stereo.SocketFD = PiFD;
    Stereo.ServerPayloadSize = ServerPayloadSize;
    Stereo.ClientPayloadSize = ClientPayloadSize;
    ServerStream.Hello = {
        .Magic = STREAM_HELLO_MAGIC,
        .CameraIndex = ServerCameraIndex,
        .SyncRole = static_cast<uint32_t>(typeSyncRole::Server),
        .Width = FRAME_WIDTH,
        .Height = FRAME_HEIGHT
    };
    ClientStream.Hello = {
        .Magic = STREAM_HELLO_MAGIC,
        .CameraIndex = ClientCameraIndex,
        .SyncRole = static_cast<uint32_t>(typeSyncRole::Client),
        .Width = FRAME_WIDTH,
        .Height = FRAME_HEIGHT
    };
    ServerStream.PiMinusMacOffsetNs = PiMinusMacOffsetNs;
    ClientStream.PiMinusMacOffsetNs = PiMinusMacOffsetNs;
    ServerStream.ClockSyncRoundTripTimeMs = ClockSyncRoundTripTimeMs;
    ClientStream.ClockSyncRoundTripTimeMs = ClockSyncRoundTripTimeMs;
    ServerStream.LatestImage = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
    ClientStream.LatestImage = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);

    std::cout << "Stereo pair stream connected; server camera "
              << ServerCameraIndex << ", client camera "
              << ClientCameraIndex << "; best clock-sync RTT = "
              << ClockSyncRoundTripTimeMs << " ms\n";
    return true;
}

static void FailStream(typeStreamRuntime& Stream, const std::string& Reason)
{
    Stream.Failed = true;
    Stream.FailureReason = Reason;
    Stream.Finished.store(true, std::memory_order_release);
}

static bool PrepareCalibrationDirectory(const typeProgramOptions& Options,
        typeCalibrationRuntime& Calibration)
{
    Calibration.Enabled = Options.Mode == typeReceiverMode::Calibration;
    Calibration.ServerIsLeft = Options.LeftRole == typeLeftRole::Server;
    Calibration.Directory = Options.CalibrationDirectory;
    if(!Calibration.Enabled)
        return true;

    std::error_code Error;
    if(Options.CameraCount == 1)
    {
        std::filesystem::create_directories(
            Calibration.Directory / "mono", Error);
    }
    else
    {
        std::filesystem::create_directories(
            Calibration.Directory / "left", Error);
        if(!Error)
            std::filesystem::create_directories(
                Calibration.Directory / "right", Error);
    }

    if(Error)
    {
        std::cerr << "Failed to create calibration directory: "
                  << Error.message() << '\n';
        return false;
    }

    std::cout << "Calibration mode: showing the live feed and saving one "
                 "capture per second under "
              << std::filesystem::absolute(Calibration.Directory) << '\n';
    return true;
}

static bool PrepareDataDirectory(const typeProgramOptions& Options,
        typeDataCollectionRuntime& DataCollection)
{
    DataCollection.Enabled =
        Options.Mode == typeReceiverMode::DataCollection;
    DataCollection.ServerIsLeft =
        Options.LeftRole == typeLeftRole::Server;
    DataCollection.Directory = Options.DataDirectory;
    if(!DataCollection.Enabled)
        return true;

    std::error_code Error;
    std::filesystem::create_directories(
        DataCollection.Directory / "left", Error);
    if(!Error)
    {
        std::filesystem::create_directories(
            DataCollection.Directory / "right", Error);
    }

    if(Error)
    {
        std::cerr << "Failed to create data directory: "
                  << Error.message() << '\n';
        return false;
    }

    std::cout << "Data mode: showing the live feed and saving every "
                 "stereo pair at the full stream rate. Frames will be "
                 "buffered in memory, then written under "
              << DataCollection.Directory << '\n';

    const size_t BytesPerPair = 2 * FRAME_SIZE;
    if(Options.NumFrames >
       std::numeric_limits<size_t>::max() / BytesPerPair)
    {
        std::cerr << "Requested data capture is too large\n";
        return false;
    }

    try
    {
        DataCollection.Pairs.reserve(Options.NumFrames);
        DataCollection.ImageData.reserve(Options.NumFrames * BytesPerPair);
    }
    catch(const std::bad_alloc&)
    {
        std::cerr << "Unable to reserve memory for data capture\n";
        return false;
    }

    std::cout << "Data buffer capacity: "
              << (Options.NumFrames * BytesPerPair) / (1024 * 1024)
              << " MiB for " << Options.NumFrames << " stereo pairs\n";
    return true;
}

static bool CalibrationCaptureDue(typeCalibrationRuntime& Calibration,
        const uint64_t TimestampNs)
{
    static constexpr uint64_t CAPTURE_INTERVAL_NS = 1'000'000'000ULL;

    if(Calibration.NextCaptureTimestampNs == 0)
        Calibration.NextCaptureTimestampNs = TimestampNs;

    if(TimestampNs < Calibration.NextCaptureTimestampNs)
        return false;

    do
    {
        Calibration.NextCaptureTimestampNs += CAPTURE_INTERVAL_NS;
    }
    while(Calibration.NextCaptureTimestampNs <= TimestampNs);

    return true;
}

static std::filesystem::path CalibrationImagePath(
        const typeCalibrationRuntime& Calibration, const char* CameraFolder,
        const size_t CaptureIndex, const uint64_t TimestampNs)
{
    std::ostringstream FileName;
    FileName << std::setfill('0') << std::setw(6) << CaptureIndex
             << '_' << TimestampNs << ".png";
    return Calibration.Directory / CameraFolder / FileName.str();
}

static bool SaveCalibrationImage(const cv::Mat& Image,
        const std::filesystem::path& Path, std::string& FailureReason)
{
    try
    {
        if(cv::imwrite(Path.string(), Image))
            return true;
        FailureReason = "OpenCV failed to write " + Path.string();
    }
    catch(const cv::Exception& Exception)
    {
        FailureReason = "Failed to write " + Path.string() + ": " +
            Exception.what();
    }
    return false;
}

static std::filesystem::path DataImagePath(
        const typeDataCollectionRuntime& DataCollection,
        const char* CameraFolder, const uint64_t PairSequence,
        const uint64_t TimestampNs)
{
    std::ostringstream FileName;
    FileName << std::setfill('0') << std::setw(8) << PairSequence
             << '_' << TimestampNs << ".pgm";
    return DataCollection.Directory / CameraFolder / FileName.str();
}

static bool SaveUncompressedGrayscaleImage(const uint8_t* ImageData,
        const std::filesystem::path& Path, std::string& FailureReason)
{
    std::ofstream File(Path, std::ios::binary);
    if(!File)
    {
        FailureReason = "Failed to open " + Path.string();
        return false;
    }

    File << "P5\n" << FRAME_WIDTH << ' ' << FRAME_HEIGHT << "\n255\n";
    File.write(reinterpret_cast<const char*>(ImageData), FRAME_SIZE);

    if(!File)
    {
        FailureReason = "Failed to write " + Path.string();
        return false;
    }
    return true;
}

static bool WriteDataCollection(typeDataCollectionRuntime& DataCollection,
        std::string& FailureReason)
{
    std::cout << "Writing " << DataCollection.Pairs.size()
              << " buffered stereo pairs...\n";

    for(size_t PairIndex = 0;
        PairIndex < DataCollection.Pairs.size(); ++PairIndex)
    {
        const typeDataCollectionRuntime::typePairMetadata& Pair =
            DataCollection.Pairs[PairIndex];
        const uint8_t* LeftImage = DataCollection.ImageData.data() +
            PairIndex * 2 * FRAME_SIZE;
        const uint8_t* RightImage = LeftImage + FRAME_SIZE;
        const std::filesystem::path LeftPath = DataImagePath(
            DataCollection, "left", Pair.PairSequence,
            Pair.LeftTimestampNs);
        const std::filesystem::path RightPath = DataImagePath(
            DataCollection, "right", Pair.PairSequence,
            Pair.RightTimestampNs);

        if(!SaveUncompressedGrayscaleImage(LeftImage, LeftPath,
                FailureReason))
        {
            return false;
        }
        if(!SaveUncompressedGrayscaleImage(RightImage, RightPath,
                FailureReason))
        {
            std::error_code RemoveError;
            std::filesystem::remove(LeftPath, RemoveError);
            return false;
        }
        ++DataCollection.PairsSaved;
    }
    return true;
}

static void ReceiveStream(typeStreamRuntime& Stream, const size_t NumFrames,
        typeCalibrationRuntime& Calibration)
{
    Stream.ArrivalIntervalsMs.reserve(NumFrames > 0 ? NumFrames - 1 : 0);
    Stream.CameraIntervalsMs.reserve(NumFrames > 0 ? NumFrames - 1 : 0);
    Stream.CaptureToMatLatencyMs.reserve(NumFrames);

    cv::Mat ReceiveImage(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
    std::chrono::steady_clock::time_point PreviousArrivalTime{};
    uint64_t PreviousCameraTimestampNs = 0;
    uint32_t PreviousSequence = 0;

    while(Stream.FramesReceived < NumFrames)
    {
        typeFrameHeader Header{};
        if(!RecvAll(Stream.SocketFD, &Header, sizeof(Header)))
        {
            FailStream(Stream, "connection closed while reading frame header");
            return;
        }

        if(Header.Width != FRAME_WIDTH || Header.Height != FRAME_HEIGHT ||
           Header.PayloadSize != FRAME_SIZE)
        {
            FailStream(Stream, "invalid frame dimensions or payload size");
            return;
        }

        if(!RecvAll(Stream.SocketFD, ReceiveImage.data, Header.PayloadSize))
        {
            FailStream(Stream, "connection closed while reading image payload");
            return;
        }

        const auto ArrivalTime = std::chrono::steady_clock::now();
        const uint64_t ArrivalTimeNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                ArrivalTime.time_since_epoch()).count());

        const double LatencyMs =
            (static_cast<double>(ArrivalTimeNs) -
             static_cast<double>(Header.TimestampNs) +
             Stream.PiMinusMacOffsetNs) / 1e6;

        if(std::isfinite(LatencyMs))
            Stream.CaptureToMatLatencyMs.push_back(LatencyMs);

        if(Stream.FramesReceived > 0)
        {
            Stream.ArrivalIntervalsMs.push_back(
                std::chrono::duration<double, std::milli>(
                    ArrivalTime - PreviousArrivalTime).count());

            Stream.CameraIntervalsMs.push_back(
                static_cast<double>(Header.TimestampNs -
                    PreviousCameraTimestampNs) / 1e6);

            if(Header.Sequence > PreviousSequence + 1)
            {
                Stream.DroppedSequenceFrames +=
                    static_cast<uint64_t>(Header.Sequence - PreviousSequence - 1);
            }
        }

        PreviousArrivalTime = ArrivalTime;
        PreviousCameraTimestampNs = Header.TimestampNs;
        PreviousSequence = Header.Sequence;
        ++Stream.FramesReceived;

        if(Calibration.Enabled &&
           CalibrationCaptureDue(Calibration, Header.TimestampNs))
        {
            const size_t CaptureIndex =
                Calibration.CapturesSaved.load(std::memory_order_relaxed);
            const std::filesystem::path Path = CalibrationImagePath(
                Calibration, "mono", CaptureIndex,
                Header.TimestampNs);
            if(!SaveCalibrationImage(ReceiveImage, Path,
                    Stream.FailureReason))
            {
                Stream.Failed = true;
                break;
            }
            Calibration.CapturesSaved.fetch_add(
                1, std::memory_order_release);
        }

        std::lock_guard<std::mutex> Lock(Stream.ImageMutex);
        ReceiveImage.copyTo(Stream.LatestImage);
        ++Stream.LatestImageVersion;
    }

    Stream.Finished.store(true, std::memory_order_release);
}

static void ReceiveStereoStream(typeStereoRuntime& Stereo,
        typeStreamRuntime& ServerStream, typeStreamRuntime& ClientStream,
        const size_t NumPairs, typeCalibrationRuntime& Calibration,
        typeDataCollectionRuntime& DataCollection)
{
    ServerStream.ArrivalIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ClientStream.ArrivalIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ServerStream.CameraIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ClientStream.CameraIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ServerStream.CaptureToMatLatencyMs.reserve(NumPairs);
    ClientStream.CaptureToMatLatencyMs.reserve(NumPairs);
    Stereo.TimestampDeltaMs.reserve(NumPairs);

    std::vector<uint8_t> ServerPayload(Stereo.ServerPayloadSize);
    std::vector<uint8_t> ClientPayload(Stereo.ClientPayloadSize);
    std::chrono::steady_clock::time_point PreviousArrivalTime{};
    uint64_t PreviousServerTimestampNs = 0;
    uint64_t PreviousClientTimestampNs = 0;
    uint64_t PreviousPairSequence = 0;
    uint32_t PreviousServerSequence = 0;
    uint32_t PreviousClientSequence = 0;

    while(ServerStream.FramesReceived < NumPairs)
    {
        typeStereoFrameHeader Header{};
        bool HeaderReceived = false;
        if(Stereo.ServerPayloadSize == FRAME_SIZE &&
           Stereo.ClientPayloadSize == FRAME_SIZE)
        {
            typeGrayscaleStereoFrameHeader GrayscaleHeader{};
            HeaderReceived = RecvAll(Stereo.SocketFD, &GrayscaleHeader,
                sizeof(GrayscaleHeader));
            Header = {
                .PairSequence = GrayscaleHeader.PairSequence,
                .ServerTimestampNs = GrayscaleHeader.ServerTimestampNs,
                .ClientTimestampNs = GrayscaleHeader.ClientTimestampNs,
                .ServerSequence = GrayscaleHeader.ServerSequence,
                .ClientSequence = GrayscaleHeader.ClientSequence,
                .Width = GrayscaleHeader.Width,
                .Height = GrayscaleHeader.Height,
                .ServerPayloadSize = GrayscaleHeader.PayloadSizePerImage,
                .ClientPayloadSize = GrayscaleHeader.PayloadSizePerImage
            };
        }
        else
        {
            HeaderReceived = RecvAll(Stereo.SocketFD, &Header,
                sizeof(Header));
        }

        if(!HeaderReceived)
        {
            Stereo.Failed = true;
            Stereo.FailureReason = "connection closed while reading pair header";
            break;
        }

        const uint64_t TimestampDeltaNs =
            Header.ServerTimestampNs >= Header.ClientTimestampNs ?
            Header.ServerTimestampNs - Header.ClientTimestampNs :
            Header.ClientTimestampNs - Header.ServerTimestampNs;

        if(Header.Width != FRAME_WIDTH || Header.Height != FRAME_HEIGHT ||
           Header.ServerPayloadSize != Stereo.ServerPayloadSize ||
           Header.ClientPayloadSize != Stereo.ClientPayloadSize ||
           TimestampDeltaNs > 5'000'000ULL)
        {
            Stereo.Failed = true;
            Stereo.FailureReason = "invalid or unsynchronized stereo pair";
            break;
        }

        if(!RecvAll(Stereo.SocketFD, ServerPayload.data(),
                ServerPayload.size()) ||
           !RecvAll(Stereo.SocketFD, ClientPayload.data(),
                ClientPayload.size()))
        {
            Stereo.Failed = true;
            Stereo.FailureReason = "connection closed while reading pair payload";
            break;
        }

        cv::Mat ServerLuminanceImage(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1,
            ServerPayload.data());
        cv::Mat ClientLuminanceImage(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1,
            ClientPayload.data());
        cv::Mat ServerImage = ServerLuminanceImage;
        cv::Mat ClientImage = ClientLuminanceImage;
        cv::Mat ServerColorImage;
        cv::Mat ClientColorImage;
        if(Stereo.ServerPayloadSize == YUV420_FRAME_SIZE)
        {
            const cv::Mat ServerYUV(FRAME_HEIGHT * 3 / 2, FRAME_WIDTH,
                CV_8UC1, ServerPayload.data());
            cv::cvtColor(ServerYUV, ServerColorImage,
                cv::COLOR_YUV2BGR_I420);
            ServerImage = ServerColorImage;
        }
        if(Stereo.ClientPayloadSize == YUV420_FRAME_SIZE)
        {
            const cv::Mat ClientYUV(FRAME_HEIGHT * 3 / 2, FRAME_WIDTH,
                CV_8UC1, ClientPayload.data());
            cv::cvtColor(ClientYUV, ClientColorImage,
                cv::COLOR_YUV2BGR_I420);
            ClientImage = ClientColorImage;
        }

        const auto ArrivalTime = std::chrono::steady_clock::now();
        const uint64_t ArrivalTimeNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                ArrivalTime.time_since_epoch()).count());

        const auto RecordFrame = [&](typeStreamRuntime& Stream,
                const uint64_t TimestampNs, const uint32_t Sequence,
                uint64_t& PreviousTimestampNs, uint32_t& PreviousSequence)
        {
            const double LatencyMs =
                (static_cast<double>(ArrivalTimeNs) -
                 static_cast<double>(TimestampNs) +
                 Stream.PiMinusMacOffsetNs) / 1e6;
            if(std::isfinite(LatencyMs))
                Stream.CaptureToMatLatencyMs.push_back(LatencyMs);

            if(Stream.FramesReceived > 0)
            {
                Stream.ArrivalIntervalsMs.push_back(
                    std::chrono::duration<double, std::milli>(
                        ArrivalTime - PreviousArrivalTime).count());
                Stream.CameraIntervalsMs.push_back(
                    static_cast<double>(TimestampNs - PreviousTimestampNs) /
                    1e6);
                if(Sequence > PreviousSequence + 1)
                    Stream.DroppedSequenceFrames +=
                        static_cast<uint64_t>(Sequence - PreviousSequence - 1);
            }

            PreviousTimestampNs = TimestampNs;
            PreviousSequence = Sequence;
            ++Stream.FramesReceived;
        };

        RecordFrame(ServerStream, Header.ServerTimestampNs,
            Header.ServerSequence, PreviousServerTimestampNs,
            PreviousServerSequence);
        RecordFrame(ClientStream, Header.ClientTimestampNs,
            Header.ClientSequence, PreviousClientTimestampNs,
            PreviousClientSequence);

        if(ServerStream.FramesReceived > 1 &&
           Header.PairSequence > PreviousPairSequence + 1)
        {
            Stereo.MissingPairSequences +=
                Header.PairSequence - PreviousPairSequence - 1;
        }
        PreviousPairSequence = Header.PairSequence;
        PreviousArrivalTime = ArrivalTime;
        Stereo.TimestampDeltaMs.push_back(
            static_cast<double>(TimestampDeltaNs) / 1e6);

        const uint64_t PairTimestampNs =
            Header.ServerTimestampNs / 2 + Header.ClientTimestampNs / 2;
        if(Calibration.Enabled &&
           CalibrationCaptureDue(Calibration, PairTimestampNs))
        {
            const size_t CaptureIndex =
                Calibration.CapturesSaved.load(std::memory_order_relaxed);
            const cv::Mat& LeftImage = Calibration.ServerIsLeft ?
                ServerImage : ClientImage;
            const cv::Mat& RightImage = Calibration.ServerIsLeft ?
                ClientImage : ServerImage;
            const std::filesystem::path LeftPath = CalibrationImagePath(
                Calibration, "left", CaptureIndex,
                PairTimestampNs);
            const std::filesystem::path RightPath = CalibrationImagePath(
                Calibration, "right", CaptureIndex,
                PairTimestampNs);

            if(!SaveCalibrationImage(LeftImage, LeftPath,
                    Stereo.FailureReason))
            {
                Stereo.Failed = true;
                break;
            }
            if(!SaveCalibrationImage(RightImage, RightPath,
                    Stereo.FailureReason))
            {
                std::error_code RemoveError;
                std::filesystem::remove(LeftPath, RemoveError);
                Stereo.Failed = true;
                break;
            }
            Calibration.CapturesSaved.fetch_add(
                1, std::memory_order_release);
        }

        if(DataCollection.Enabled)
        {
            const cv::Mat& LeftImage = DataCollection.ServerIsLeft ?
                ServerImage : ClientImage;
            const cv::Mat& RightImage = DataCollection.ServerIsLeft ?
                ClientImage : ServerImage;
            const uint64_t LeftTimestampNs = DataCollection.ServerIsLeft ?
                Header.ServerTimestampNs : Header.ClientTimestampNs;
            const uint64_t RightTimestampNs = DataCollection.ServerIsLeft ?
                Header.ClientTimestampNs : Header.ServerTimestampNs;

            DataCollection.Pairs.push_back({
                .PairSequence = Header.PairSequence,
                .LeftTimestampNs = LeftTimestampNs,
                .RightTimestampNs = RightTimestampNs
            });
            const size_t DataOffset = DataCollection.ImageData.size();
            DataCollection.ImageData.resize(DataOffset + 2 * FRAME_SIZE);
            std::memcpy(DataCollection.ImageData.data() + DataOffset,
                LeftImage.data, FRAME_SIZE);
            std::memcpy(DataCollection.ImageData.data() + DataOffset +
                FRAME_SIZE, RightImage.data, FRAME_SIZE);
        }

        std::lock_guard<std::mutex> Lock(Stereo.ImageMutex);
        ServerImage.copyTo(ServerStream.LatestImage);
        ClientImage.copyTo(ClientStream.LatestImage);
        ServerLuminanceImage.copyTo(ServerStream.LatestLuminanceImage);
        ClientLuminanceImage.copyTo(ClientStream.LatestLuminanceImage);
        ++Stereo.LatestImageVersion;
    }

    ServerStream.Finished.store(true, std::memory_order_release);
    ClientStream.Finished.store(true, std::memory_order_release);
    Stereo.Finished.store(true, std::memory_order_release);
}

static bool UpdateStereoDisplayImages(typeStereoRuntime& Stereo,
        typeStreamRuntime& LeftStream, typeStreamRuntime& RightStream,
        uint64_t& DisplayedVersion, cv::Mat& LeftImage, cv::Mat& RightImage,
        cv::Mat& LeftLuminanceImage, cv::Mat& RightLuminanceImage)
{
    std::lock_guard<std::mutex> Lock(Stereo.ImageMutex);
    if(Stereo.LatestImageVersion == 0 ||
       Stereo.LatestImageVersion == DisplayedVersion)
    {
        return false;
    }

    LeftStream.LatestImage.copyTo(LeftImage);
    RightStream.LatestImage.copyTo(RightImage);
    LeftStream.LatestLuminanceImage.copyTo(LeftLuminanceImage);
    RightStream.LatestLuminanceImage.copyTo(RightLuminanceImage);
    DisplayedVersion = Stereo.LatestImageVersion;
    return true;
}

static bool UpdateDisplayImage(typeStreamRuntime& Stream,
        uint64_t& DisplayedVersion, cv::Mat& DisplayImage)
{
    std::lock_guard<std::mutex> Lock(Stream.ImageMutex);

    if(Stream.LatestImageVersion == 0 ||
       Stream.LatestImageVersion == DisplayedVersion)
    {
        return false;
    }

    Stream.LatestImage.copyTo(DisplayImage);
    DisplayedVersion = Stream.LatestImageVersion;
    return true;
}

static std::filesystem::path FindDemoPythonInterpreter(
        std::filesystem::path SearchDirectory)
{
    while(!SearchDirectory.empty())
    {
        const std::filesystem::path Candidate =
            SearchDirectory / ".venv/bin/python";
        if(std::filesystem::exists(Candidate))
            return Candidate;

        const std::filesystem::path Parent = SearchDirectory.parent_path();
        if(Parent == SearchDirectory)
            break;
        SearchDirectory = Parent;
    }
    return {};
}

static bool StartDemoVisualization(const std::filesystem::path& ExecutablePath,
        typeDemoVisualizationRuntime& Visualization)
{
    const std::filesystem::path ExecutableDirectory =
        std::filesystem::absolute(ExecutablePath).lexically_normal().parent_path();
    const std::filesystem::path ViewerScript =
        ExecutableDirectory / "demo_pointcloud_viewer.py";
    const std::filesystem::path Python =
        FindDemoPythonInterpreter(ExecutableDirectory);

    if(Python.empty() || !std::filesystem::exists(ViewerScript))
    {
        std::cerr << "Demo viewer dependencies not found\n"
                  << "  Python: " << Python << '\n'
                  << "  Script: " << ViewerScript << '\n';
        return false;
    }

    Visualization.OutputDirectory = ExecutableDirectory / "demo_pointcloud";
    Visualization.FramePath = Visualization.OutputDirectory / "points.bin";
    Visualization.TemporaryFramePath =
        Visualization.OutputDirectory / "points.bin.tmp";

    std::error_code Error;
    std::filesystem::create_directories(Visualization.OutputDirectory, Error);
    if(Error)
    {
        std::cerr << "Failed to create demo point-cloud directory: "
                  << Error.message() << '\n';
        return false;
    }
    std::filesystem::remove(Visualization.FramePath, Error);
    Error.clear();
    std::filesystem::remove(Visualization.TemporaryFramePath, Error);

    const std::string PythonString = Python.string();
    const std::string ScriptString = ViewerScript.string();
    const std::string OutputString = Visualization.OutputDirectory.string();
    const std::string ParentPID = std::to_string(getpid());
    const pid_t PID = fork();
    if(PID < 0)
    {
        perror("fork demo viewer");
        return false;
    }
    if(PID == 0)
    {
        execl(PythonString.c_str(), PythonString.c_str(),
            ScriptString.c_str(), OutputString.c_str(), ParentPID.c_str(),
            static_cast<char*>(nullptr));
        perror("execl demo viewer");
        _exit(127);
    }

    Visualization.ViewerPID = PID;
    std::cout << "Started demo Viser point-cloud viewer (PID "
              << PID << ")\n";
    return true;
}

static void StopDemoVisualization(typeDemoVisualizationRuntime& Visualization)
{
    if(Visualization.ViewerPID <= 0)
        return;

    kill(Visualization.ViewerPID, SIGTERM);
    while(waitpid(Visualization.ViewerPID, nullptr, 0) < 0 && errno == EINTR)
    {
    }
    Visualization.ViewerPID = -1;
}

static bool PublishDemoPointCloud(
        typeDemoVisualizationRuntime& Visualization,
        const std::vector<float>& Positions,
        const std::vector<uint8_t>& Colors)
{
    const uint64_t NumPoints = Positions.size() / 3;
    if(Positions.size() != NumPoints * 3 || Colors.size() != NumPoints * 3)
        return false;

    std::ofstream File(Visualization.TemporaryFramePath,
        std::ios::binary | std::ios::trunc);
    if(!File)
        return false;

    const uint64_t FrameSequence = Visualization.FrameSequence++;
    File.write(reinterpret_cast<const char*>(&DEMO_POINT_CLOUD_MAGIC),
        sizeof(DEMO_POINT_CLOUD_MAGIC));
    File.write(reinterpret_cast<const char*>(&FrameSequence),
        sizeof(FrameSequence));
    File.write(reinterpret_cast<const char*>(&NumPoints), sizeof(NumPoints));
    File.write(reinterpret_cast<const char*>(Positions.data()),
        static_cast<std::streamsize>(Positions.size() * sizeof(float)));
    File.write(reinterpret_cast<const char*>(Colors.data()),
        static_cast<std::streamsize>(Colors.size()));
    File.close();
    if(!File)
        return false;

    std::error_code Error;
    std::filesystem::rename(Visualization.TemporaryFramePath,
        Visualization.FramePath, Error);
    return !Error;
}

static void ProcessDemoStereoPair(const cv::Mat& LeftLuminance,
        const cv::Mat& RightLuminance, const cv::Mat& LeftBGR,
        typeDemoVisualizationRuntime& Visualization,
        const bool SGBMDoublePass)
{
    static constexpr int BlockSize = 7;
    static constexpr int MinDisparity = -128;
    static constexpr int NumDisparities = 128;
    static constexpr int PointSampleStride = 3;
    static constexpr float MinimumDepthMetres = 0.15F;
    static constexpr float MaximumDepthMetres = 15.0F;

    static const cv::Matx33d KLeft(
        583.656905612597, 0.0, 318.246701709208,
        0.0, 584.913010873283, 219.459363187465,
        0.0, 0.0, 1.0);
    static const cv::Matx33d KRight(
        585.964174869895, 0.0, 316.930217707653,
        0.0, 586.841937275282, 217.008369998399,
        0.0, 0.0, 1.0);
    static const cv::Vec<double, 5> DLeft(
        -0.070231934465, 0.581467630762, -0.011830893759,
        -0.001925557061, -1.100253544397);
    static const cv::Vec<double, 5> DRight(
        -0.06616119276960, 0.4795788187738, -0.01156431233099,
        -0.0007034267423764, -0.8721983929016);
    static const cv::Matx33d RLeftRectification(
        0.999512054949, -0.003047586781, -0.031086399372,
        0.002905116148, 0.999985074594, -0.004627190226,
        0.031100037159, 0.004534622810, 0.999505990420);
    static const cv::Matx33d RRightRectification(
        0.9988392751028, 0.0009248128518370, -0.04815856344735,
        -0.0007041022937771, 0.9999891731673, 0.004599759581089,
        0.04816229595941, -0.004560511970603, 0.9988291119999);
    static const cv::Matx34d PLeft(
        614.558600234117, 0.0, 346.102619171143, 0.0,
        0.0, 614.558600234117, 206.618324279785, 0.0,
        0.0, 0.0, 1.0, 0.0);
    static const cv::Matx34d PRight(
        614.558600234117, 0.0, 346.102619171143, 42.563475269922,
        0.0, 614.558600234117, 206.618324279785, 0.0,
        0.0, 0.0, 1.0, 0.0);
    static const cv::Matx44d Q(
        1.0, 0.0, 0.0, -346.102619171143,
        0.0, 1.0, 0.0, -206.618324279785,
        0.0, 0.0, 0.0, 614.558600234117,
        0.0, 0.0, -14.438637736623, 0.0);

    struct typeDemoPipeline
    {
        cv::Mat LeftMap1;
        cv::Mat LeftMap2;
        cv::Mat RightMap1;
        cv::Mat RightMap2;
        cv::Ptr<cv::StereoSGBM> Stereo;
        cv::Ptr<cv::StereoMatcher> RightMatcher;
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> WLS;
        bool DoublePass;

        explicit typeDemoPipeline(const bool UseDoublePass)
            : DoublePass(UseDoublePass)
        {
            const cv::Size ImageSize(FRAME_WIDTH, FRAME_HEIGHT);
            cv::initUndistortRectifyMap(KLeft, DLeft,
                RLeftRectification, PLeft, ImageSize, CV_16SC2,
                LeftMap1, LeftMap2);
            cv::initUndistortRectifyMap(KRight, DRight,
                RRightRectification, PRight, ImageSize, CV_16SC2,
                RightMap1, RightMap2);

            Stereo = cv::StereoSGBM::create(
                MinDisparity, NumDisparities, BlockSize);
            Stereo->setP1(8 * BlockSize * BlockSize);
            Stereo->setP2(32 * BlockSize * BlockSize);
            Stereo->setUniquenessRatio(5);
            Stereo->setSpeckleWindowSize(50);
            Stereo->setSpeckleRange(2);
            Stereo->setDisp12MaxDiff(1);
            Stereo->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

            WLS = cv::ximgproc::createDisparityWLSFilterGeneric(DoublePass);
            if(DoublePass)
                RightMatcher = cv::ximgproc::createRightMatcher(Stereo);
            WLS->setLambda(500.0);
            WLS->setSigmaColor(0.0001);
        }
    };
    static typeDemoPipeline Pipeline(SGBMDoublePass);

    if(LeftLuminance.empty() || RightLuminance.empty() || LeftBGR.empty())
        return;

    cv::Mat LeftRectified;
    cv::Mat RightRectified;
    cv::Mat LeftColorRectified;
    cv::remap(LeftLuminance, LeftRectified,
        Pipeline.LeftMap1, Pipeline.LeftMap2, cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
    cv::remap(RightLuminance, RightRectified,
        Pipeline.RightMap1, Pipeline.RightMap2, cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
    cv::remap(LeftBGR, LeftColorRectified,
        Pipeline.LeftMap1, Pipeline.LeftMap2, cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
    cv::Mat Disparity16;
    Pipeline.Stereo->compute(LeftRectified, RightRectified, Disparity16);

    cv::Mat FilteredDisparity16;
    if(Pipeline.DoublePass)
    {
        cv::Mat RightDisparity16;
        Pipeline.RightMatcher->compute(
            RightRectified, LeftRectified, RightDisparity16);
        Pipeline.WLS->filter(Disparity16, LeftRectified,
            FilteredDisparity16, RightDisparity16,
            cv::Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT));
    }
    else
    {
        const int InvalidDisparity = (MinDisparity - 1) * 16;
        cv::Mat WLSInput = Disparity16.clone();
        WLSInput.setTo(0, Disparity16 <= InvalidDisparity);
        Pipeline.WLS->filter(
            WLSInput, LeftRectified, FilteredDisparity16);
    }
    const cv::Mat ValidDisparityMask =
        (FilteredDisparity16 >= MinDisparity * 16) &
        (FilteredDisparity16 < 0);

    cv::Mat DisparityDisplay;
    FilteredDisparity16.convertTo(DisparityDisplay, CV_8U,
        -255.0 / (NumDisparities * 16.0), 0.0);
    DisparityDisplay.setTo(0, ~ValidDisparityMask);

    cv::Mat ColorDisparity;
    cv::applyColorMap(DisparityDisplay, ColorDisparity, cv::COLORMAP_TURBO);
    ColorDisparity.setTo(cv::Scalar(0, 0, 0), ~ValidDisparityMask);
    cv::imshow("Demo Disparity", ColorDisparity);

    cv::Mat Disparity32;
    cv::Mat Points3D;
    FilteredDisparity16.convertTo(Disparity32, CV_32F, 1.0 / 16.0);
    cv::reprojectImageTo3D(Disparity32, Points3D, Q, false, CV_32F);

    const size_t MaximumPointCount =
        ((FRAME_WIDTH + PointSampleStride - 1) / PointSampleStride) *
        ((FRAME_HEIGHT + PointSampleStride - 1) / PointSampleStride);
    std::vector<float> Positions;
    std::vector<uint8_t> Colors;
    Positions.reserve(MaximumPointCount * 3);
    Colors.reserve(MaximumPointCount * 3);

    for(int Y = 0; Y < Points3D.rows; Y += PointSampleStride)
    {
        for(int X = 0; X < Points3D.cols; X += PointSampleStride)
        {
            if(ValidDisparityMask.at<uint8_t>(Y, X) == 0)
                continue;

            const cv::Vec3f Point = Points3D.at<cv::Vec3f>(Y, X);
            if(!std::isfinite(Point[0]) || !std::isfinite(Point[1]) ||
               !std::isfinite(Point[2]) ||
               Point[2] < MinimumDepthMetres ||
               Point[2] > MaximumDepthMetres)
            {
                continue;
            }

            // OpenCV has Y down and Z forward. Viser has Y up and the
            // initial camera below looks along negative Z.
            Positions.push_back(Point[0]);
            Positions.push_back(-Point[1]);
            Positions.push_back(-Point[2]);

            const cv::Vec3b BGR = LeftColorRectified.at<cv::Vec3b>(Y, X);
            Colors.push_back(BGR[2]);
            Colors.push_back(BGR[1]);
            Colors.push_back(BGR[0]);
        }
    }

    if(!PublishDemoPointCloud(Visualization, Positions, Colors))
        std::cerr << "Failed to publish demo point cloud\n";
}

static void DrawCalibrationCaptureIndicator(cv::Mat& Image,
        const size_t CapturesSaved, const bool CaptureJustSaved)
{
    if(Image.channels() == 1)
        cv::cvtColor(Image, Image, cv::COLOR_GRAY2BGR);

    const cv::Scalar IndicatorColor = CaptureJustSaved ?
        cv::Scalar(0, 255, 0) : cv::Scalar(210, 210, 210);
    const std::string Label = CaptureJustSaved ?
        "CAPTURED #" + std::to_string(CapturesSaved) :
        "Saved: " + std::to_string(CapturesSaved);

    cv::rectangle(Image, cv::Point(10, 10), cv::Point(245, 58),
        cv::Scalar(0, 0, 0), cv::FILLED);
    cv::circle(Image, cv::Point(32, 34), 10, IndicatorColor, cv::FILLED);
    cv::putText(Image, Label, cv::Point(52, 43), cv::FONT_HERSHEY_SIMPLEX,
        0.72, IndicatorColor, 2, cv::LINE_AA);
}

static void PrintStreamStats(const typeStreamRuntime& Stream)
{
    const typeStats ArrivalStats = CalculateStats(Stream.ArrivalIntervalsMs);
    const typeStats CameraStats = CalculateStats(Stream.CameraIntervalsMs);
    const typeStats LatencyStats =
        CalculateStats(Stream.CaptureToMatLatencyMs);

    const double ArrivalFPS = ArrivalStats.Mean > 0.0 ?
        1000.0 / ArrivalStats.Mean : 0.0;
    const double CameraFPS = CameraStats.Mean > 0.0 ?
        1000.0 / CameraStats.Mean : 0.0;

    std::cout
        << "\n========================================\n"
        << ' ' << Stream.Name << " (camera " << Stream.Hello.CameraIndex
        << ", "
        << SyncRoleName(static_cast<typeSyncRole>(Stream.Hello.SyncRole))
        << ")\n"
        << "========================================\n"
        << " Frames received:          " << Stream.FramesReceived << '\n'
        << " Missing sequence frames:  " << Stream.DroppedSequenceFrames << '\n'
        << " Target FPS:               " << TARGET_FPS << " Hz\n"
        << " CAMERA TIMESTAMPS\n"
        << " Mean interval:            " << CameraStats.Mean << " ms\n"
        << " Std. dev.:                " << CameraStats.StdDev << " ms\n"
        << " Equivalent FPS:           " << CameraFPS << " Hz\n"
        << " MAC ARRIVAL TIMING\n"
        << " Mean interval:            " << ArrivalStats.Mean << " ms\n"
        << " Std. dev.:                " << ArrivalStats.StdDev << " ms\n"
        << " Equivalent FPS:           " << ArrivalFPS << " Hz\n"
        << " CAPTURE TO CV::MAT LATENCY\n"
        << " Clock sync best RTT:      "
        << Stream.ClockSyncRoundTripTimeMs << " ms\n"
        << " Mean latency:             " << LatencyStats.Mean << " ms\n"
        << " Std. dev.:                " << LatencyStats.StdDev << " ms\n"
        << " Samples:                  "
        << Stream.CaptureToMatLatencyMs.size() << '\n';

    if(Stream.Failed)
        std::cout << " Stream error:              " << Stream.FailureReason << '\n';
}

int main(int argc, char* argv[])
{
    typeProgramOptions Options{};
    Options.DataDirectory =
        std::filesystem::absolute(argv[0]).lexically_normal().parent_path() /
            "data";
    if(!ParseProgramOptions(argc, argv, Options))
    {
        PrintUsage(argv[0]);
        return 1;
    }

    typeCalibrationRuntime Calibration{};
    if(!PrepareCalibrationDirectory(Options, Calibration))
        return 1;

    typeDataCollectionRuntime DataCollection{};
    if(!PrepareDataDirectory(Options, DataCollection))
        return 1;

    const int ServerFD =
        CreateServerSocket(DEFAULT_PORT, static_cast<int>(Options.CameraCount));
    if(ServerFD < 0)
        return 1;

    std::cout
        << "Listening on port " << DEFAULT_PORT << " for "
        << (Options.CameraCount == 1 ? "one camera stream" :
            "one paired stereo stream") << '\n';

    if(!StartPiProgram(Options))
    {
        std::cerr << "Failed to start Pi program\n";
        close(ServerFD);
        return 1;
    }

    typeStreamRuntime ServerOrMonoStream{};
    typeStreamRuntime ClientStream{};
    typeStereoRuntime Stereo{};

    const bool Accepted = Options.CameraCount == 1 ?
        AcceptStreams(ServerFD, Options, ServerOrMonoStream, ClientStream) :
        AcceptStereoStream(ServerFD, Options, Stereo,
            ServerOrMonoStream, ClientStream);

    if(!Accepted)
    {
        if(ServerOrMonoStream.SocketFD >= 0)
            close(ServerOrMonoStream.SocketFD);
        if(ClientStream.SocketFD >= 0)
            close(ClientStream.SocketFD);
        if(Stereo.SocketFD >= 0)
            close(Stereo.SocketFD);
        close(ServerFD);
        return 1;
    }

    close(ServerFD);

    typeDemoVisualizationRuntime DemoVisualization{};
    if(Options.Mode == typeReceiverMode::Demo &&
       !StartDemoVisualization(argv[0], DemoVisualization))
    {
        if(Options.CameraCount == 1)
            close(ServerOrMonoStream.SocketFD);
        else
            close(Stereo.SocketFD);
        return 1;
    }
    if(Options.Mode == typeReceiverMode::Demo)
    {
        std::cout << "Demo WLS mode: "
                  << (Options.SGBMDoublePass ?
                      "double pass with left-right confidence" :
                      "single pass, high FPS")
                  << '\n';
    }

    typeStreamRuntime* LeftStream = &ServerOrMonoStream;
    typeStreamRuntime* RightStream = nullptr;

    if(Options.CameraCount == 1)
    {
        ServerOrMonoStream.Name = "Mono";
    }
    else if(Options.LeftRole == typeLeftRole::Server)
    {
        ServerOrMonoStream.Name = "Left";
        ClientStream.Name = "Right";
        RightStream = &ClientStream;
    }
    else
    {
        ClientStream.Name = "Left";
        ServerOrMonoStream.Name = "Right";
        LeftStream = &ClientStream;
        RightStream = &ServerOrMonoStream;
    }

    std::cout << "Receiving " << Options.NumFrames << " frame(s) per camera";
    if(RightStream != nullptr)
    {
        std::cout
            << "; left = camera " << LeftStream->Hello.CameraIndex << " ("
            << SyncRoleName(static_cast<typeSyncRole>(LeftStream->Hello.SyncRole))
            << "), right = camera " << RightStream->Hello.CameraIndex << " ("
            << SyncRoleName(static_cast<typeSyncRole>(RightStream->Hello.SyncRole))
            << ')';
    }
    std::cout << '\n';

    std::thread Receiver;
    if(Options.CameraCount == 1)
        Receiver = std::thread(ReceiveStream,
            std::ref(ServerOrMonoStream), Options.NumFrames,
            std::ref(Calibration));
    else
        Receiver = std::thread(ReceiveStereoStream, std::ref(Stereo),
            std::ref(ServerOrMonoStream), std::ref(ClientStream),
            Options.NumFrames, std::ref(Calibration),
            std::ref(DataCollection));

    uint64_t LeftDisplayedVersion = 0;
    cv::Mat LeftDisplayImage;
    cv::Mat RightDisplayImage;
    cv::Mat LeftLuminanceImage;
    cv::Mat RightLuminanceImage;
    cv::Mat RightColorDisplayImage;
    cv::Mat StereoDisplayImage;
    size_t DisplayedCaptureCount = 0;
    std::chrono::steady_clock::time_point CaptureIndicatorUntil{};

    while(Options.CameraCount == 1 ?
          !ServerOrMonoStream.Finished.load(std::memory_order_acquire) :
          !Stereo.Finished.load(std::memory_order_acquire))
    {
        const auto CurrentTime = std::chrono::steady_clock::now();
        const size_t CapturesSaved =
            Calibration.CapturesSaved.load(std::memory_order_acquire);
        if(CapturesSaved != DisplayedCaptureCount)
        {
            DisplayedCaptureCount = CapturesSaved;
            CaptureIndicatorUntil =
                CurrentTime + std::chrono::milliseconds(500);
        }
        const bool CaptureJustSaved =
            CurrentTime < CaptureIndicatorUntil;

        if(RightStream != nullptr)
        {
            const bool Updated = UpdateStereoDisplayImages(Stereo,
                *LeftStream, *RightStream, LeftDisplayedVersion,
                LeftDisplayImage, RightDisplayImage,
                LeftLuminanceImage, RightLuminanceImage);

            if(Updated)
            {
                if(Options.Mode == typeReceiverMode::Demo)
                {
                    ProcessDemoStereoPair(LeftLuminanceImage,
                        RightLuminanceImage, LeftDisplayImage,
                        DemoVisualization, Options.SGBMDoublePass);
                }

                const cv::Mat* RightImageForDisplay = &RightDisplayImage;
                if(LeftDisplayImage.channels() == 3 &&
                   RightDisplayImage.channels() == 1)
                {
                    cv::cvtColor(RightDisplayImage, RightColorDisplayImage,
                        cv::COLOR_GRAY2BGR);
                    RightImageForDisplay = &RightColorDisplayImage;
                }
                cv::hconcat(LeftDisplayImage, *RightImageForDisplay,
                    StereoDisplayImage);
                if(Calibration.Enabled)
                {
                    DrawCalibrationCaptureIndicator(StereoDisplayImage,
                        DisplayedCaptureCount, CaptureJustSaved);
                }
                cv::imshow("Stereo Cameras", StereoDisplayImage);
            }
        }
        else if(UpdateDisplayImage(*LeftStream,
                    LeftDisplayedVersion, LeftDisplayImage))
        {
            if(Calibration.Enabled)
            {
                DrawCalibrationCaptureIndicator(LeftDisplayImage,
                    DisplayedCaptureCount, CaptureJustSaved);
            }
            cv::imshow("Pi Camera", LeftDisplayImage);
        }

        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    Receiver.join();

    if(Options.CameraCount == 1)
        close(ServerOrMonoStream.SocketFD);
    else
        close(Stereo.SocketFD);

    cv::destroyAllWindows();
    StopDemoVisualization(DemoVisualization);

    if(DataCollection.Enabled &&
       !WriteDataCollection(DataCollection, Stereo.FailureReason))
    {
        Stereo.Failed = true;
        std::cerr << Stereo.FailureReason << '\n';
    }

    if(Calibration.Enabled)
    {
        std::cout << "Calibration captures saved: "
                  << Calibration.CapturesSaved.load(std::memory_order_acquire)
                  << '\n';
    }

    if(DataCollection.Enabled)
    {
        std::cout << "Data stereo pairs captured: "
                  << DataCollection.Pairs.size() << '\n'
                  << "Data stereo pairs saved: "
                  << DataCollection.PairsSaved << '\n';
    }

    if(Options.CameraCount == 1)
    {
        PrintStreamStats(ServerOrMonoStream);
    }
    else
    {
        PrintStreamStats(*LeftStream);
        PrintStreamStats(*RightStream);

        const typeStats DeltaStats = CalculateStats(Stereo.TimestampDeltaMs);
        std::cout << "\nSTEREO PAIRING\n"
                  << " Pairs missing after Pi queue: "
                  << Stereo.MissingPairSequences << '\n'
                  << " Mean timestamp delta:        "
                  << DeltaStats.Mean << " ms\n"
                  << " Timestamp delta std. dev.:   "
                  << DeltaStats.StdDev << " ms\n";
        if(Stereo.Failed)
            std::cout << " Stereo error:                "
                      << Stereo.FailureReason << '\n';
    }

    return ServerOrMonoStream.Failed || ClientStream.Failed || Stereo.Failed ?
        1 : 0;
}
