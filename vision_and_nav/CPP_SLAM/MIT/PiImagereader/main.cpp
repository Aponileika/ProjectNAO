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
 *   --mac-ip ADDRESS          Mac address advertised to the Pi
 *                             (default: 192.168.0.95).
 *   --help, -h                Print runtime usage information.
 *
 * The receiver starts ~/PantoPI/ImageIO on
 * pantopilot@pantopilotstereoraspberry.local over SSH.
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

static constexpr uint16_t DEFAULT_PORT = 50555;
static constexpr size_t DEFAULT_NUM_FRAMES = 600;
static constexpr uint32_t FRAME_WIDTH = 640;
static constexpr uint32_t FRAME_HEIGHT = 480;
static constexpr uint32_t FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT;
static constexpr double TARGET_FPS = 15.0;
static constexpr const char* DEFAULT_MAC_IP = "10.241.223.208";
static constexpr const char* PI_SSH_HOST =
    "pantopilot@pantopilotstereoraspberry.local";
static constexpr const char* PI_PROGRAM = "~/PantoPI/ImageIO";
static constexpr uint64_t STREAM_HELLO_MAGIC = 0x50414E544F43414DULL;
static constexpr uint64_t STEREO_STREAM_HELLO_MAGIC = 0x50414E544F535452ULL;
static constexpr uint64_t CLOCK_SYNC_MAGIC = 0x50414E544F434C4BULL;
static constexpr uint32_t CLOCK_SYNC_SAMPLES = 8;

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

struct typeProgramOptions
{
    size_t CameraCount = 1;
    size_t CameraIndex = 0;
    size_t ServerCameraIndex = 0;
    size_t ClientCameraIndex = 1;
    typeLeftRole LeftRole = typeLeftRole::Server;
    size_t NumFrames = DEFAULT_NUM_FRAMES;
    std::string MacIPAddress = DEFAULT_MAC_IP;
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
    uint32_t PayloadSizePerImage;
};

static_assert(sizeof(typeStereoStreamHello) == 24);
static_assert(sizeof(typeStereoFrameHeader) == 48);

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
    uint64_t MissingPairSequences = 0;
    std::vector<double> TimestampDeltaMs;
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
        << "Optional: --frames COUNT --mac-ip ADDRESS\n\n"
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
       Options.NumFrames == 0 || Options.MacIPAddress.empty())
    {
        return false;
    }

    if(Options.CameraCount == 2 &&
       Options.ServerCameraIndex == Options.ClientCameraIndex)
    {
        std::cerr << "Server and client camera indices must be different\n";
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

    typeStereoStreamHello Hello{};
    if(!RecvAll(PiFD, &Hello, sizeof(Hello)) ||
       Hello.Magic != STEREO_STREAM_HELLO_MAGIC ||
       Hello.ServerCameraIndex != Options.ServerCameraIndex ||
       Hello.ClientCameraIndex != Options.ClientCameraIndex ||
       Hello.Width != FRAME_WIDTH || Hello.Height != FRAME_HEIGHT)
    {
        std::cerr << "Invalid stereo stream-identification message from Pi\n";
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
    ServerStream.Hello = {
        .Magic = STREAM_HELLO_MAGIC,
        .CameraIndex = Hello.ServerCameraIndex,
        .SyncRole = static_cast<uint32_t>(typeSyncRole::Server),
        .Width = Hello.Width,
        .Height = Hello.Height
    };
    ClientStream.Hello = {
        .Magic = STREAM_HELLO_MAGIC,
        .CameraIndex = Hello.ClientCameraIndex,
        .SyncRole = static_cast<uint32_t>(typeSyncRole::Client),
        .Width = Hello.Width,
        .Height = Hello.Height
    };
    ServerStream.PiMinusMacOffsetNs = PiMinusMacOffsetNs;
    ClientStream.PiMinusMacOffsetNs = PiMinusMacOffsetNs;
    ServerStream.ClockSyncRoundTripTimeMs = ClockSyncRoundTripTimeMs;
    ClientStream.ClockSyncRoundTripTimeMs = ClockSyncRoundTripTimeMs;
    ServerStream.LatestImage = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
    ClientStream.LatestImage = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);

    std::cout << "Stereo pair stream connected; server camera "
              << Hello.ServerCameraIndex << ", client camera "
              << Hello.ClientCameraIndex << "; best clock-sync RTT = "
              << ClockSyncRoundTripTimeMs << " ms\n";
    return true;
}

static void FailStream(typeStreamRuntime& Stream, const std::string& Reason)
{
    Stream.Failed = true;
    Stream.FailureReason = Reason;
    Stream.Finished.store(true, std::memory_order_release);
}

static void ReceiveStream(typeStreamRuntime& Stream, const size_t NumFrames)
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

        {
            std::lock_guard<std::mutex> Lock(Stream.ImageMutex);
            ReceiveImage.copyTo(Stream.LatestImage);
            ++Stream.LatestImageVersion;
        }
    }

    Stream.Finished.store(true, std::memory_order_release);
}

static void ReceiveStereoStream(typeStereoRuntime& Stereo,
        typeStreamRuntime& ServerStream, typeStreamRuntime& ClientStream,
        const size_t NumPairs)
{
    ServerStream.ArrivalIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ClientStream.ArrivalIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ServerStream.CameraIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ClientStream.CameraIntervalsMs.reserve(NumPairs > 0 ? NumPairs - 1 : 0);
    ServerStream.CaptureToMatLatencyMs.reserve(NumPairs);
    ClientStream.CaptureToMatLatencyMs.reserve(NumPairs);
    Stereo.TimestampDeltaMs.reserve(NumPairs);

    cv::Mat ServerImage(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
    cv::Mat ClientImage(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
    std::chrono::steady_clock::time_point PreviousArrivalTime{};
    uint64_t PreviousServerTimestampNs = 0;
    uint64_t PreviousClientTimestampNs = 0;
    uint64_t PreviousPairSequence = 0;
    uint32_t PreviousServerSequence = 0;
    uint32_t PreviousClientSequence = 0;

    while(ServerStream.FramesReceived < NumPairs)
    {
        typeStereoFrameHeader Header{};
        if(!RecvAll(Stereo.SocketFD, &Header, sizeof(Header)))
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
           Header.PayloadSizePerImage != FRAME_SIZE ||
           TimestampDeltaNs > 5'000'000ULL)
        {
            Stereo.Failed = true;
            Stereo.FailureReason = "invalid or unsynchronized stereo pair";
            break;
        }

        if(!RecvAll(Stereo.SocketFD, ServerImage.data, FRAME_SIZE) ||
           !RecvAll(Stereo.SocketFD, ClientImage.data, FRAME_SIZE))
        {
            Stereo.Failed = true;
            Stereo.FailureReason = "connection closed while reading pair payload";
            break;
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

        {
            std::lock_guard<std::mutex> Lock(Stereo.ImageMutex);
            ServerImage.copyTo(ServerStream.LatestImage);
            ClientImage.copyTo(ClientStream.LatestImage);
            ++Stereo.LatestImageVersion;
        }
    }

    ServerStream.Finished.store(true, std::memory_order_release);
    ClientStream.Finished.store(true, std::memory_order_release);
    Stereo.Finished.store(true, std::memory_order_release);
}

static bool UpdateStereoDisplayImages(typeStereoRuntime& Stereo,
        typeStreamRuntime& LeftStream, typeStreamRuntime& RightStream,
        uint64_t& DisplayedVersion, cv::Mat& LeftImage, cv::Mat& RightImage)
{
    std::lock_guard<std::mutex> Lock(Stereo.ImageMutex);
    if(Stereo.LatestImageVersion == 0 ||
       Stereo.LatestImageVersion == DisplayedVersion)
    {
        return false;
    }

    LeftStream.LatestImage.copyTo(LeftImage);
    RightStream.LatestImage.copyTo(RightImage);
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
    if(!ParseProgramOptions(argc, argv, Options))
    {
        PrintUsage(argv[0]);
        return 1;
    }

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
            std::ref(ServerOrMonoStream), Options.NumFrames);
    else
        Receiver = std::thread(ReceiveStereoStream, std::ref(Stereo),
            std::ref(ServerOrMonoStream), std::ref(ClientStream),
            Options.NumFrames);

    uint64_t LeftDisplayedVersion = 0;
    cv::Mat LeftDisplayImage;
    cv::Mat RightDisplayImage;
    cv::Mat StereoDisplayImage;

    while(Options.CameraCount == 1 ?
          !ServerOrMonoStream.Finished.load(std::memory_order_acquire) :
          !Stereo.Finished.load(std::memory_order_acquire))
    {
        if(RightStream != nullptr)
        {
            const bool Updated = UpdateStereoDisplayImages(Stereo,
                *LeftStream, *RightStream, LeftDisplayedVersion,
                LeftDisplayImage, RightDisplayImage);

            if(Updated)
            {
                cv::hconcat(
                    LeftDisplayImage,
                    RightDisplayImage,
                    StereoDisplayImage);
                cv::imshow("Stereo Cameras", StereoDisplayImage);
            }
        }
        else if(UpdateDisplayImage(*LeftStream,
                    LeftDisplayedVersion, LeftDisplayImage))
        {
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
