/*
 * TODO, make the raspberry pi program recieve an IP as an argument
 *
 * */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <vector>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

const char* PantoSSH = "ssh pantopilot@pantopilotstereoraspberry.local";

struct typeFrameHeader
{
    uint64_t TimestampNs;
    uint32_t Sequence;
    uint32_t Width;
    uint32_t Height;
    uint32_t PayloadSize;
};

static constexpr uint64_t CLOCK_SYNC_MAGIC = 0x50414E544F434C4BULL;
static constexpr uint32_t CLOCK_SYNC_SAMPLES = 8;

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
    double Mean;
    double StdDev;
};

typeStats CalculateStats(
    const std::vector<double>& Values)
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

    const double N =
        static_cast<double>(Values.size());

    const double Mean =
        Sum / N;

    double StdDev = 0.0;

    if(Values.size() > 1)
    {
        const double Variance =
            (
                SumSquared -
                (Sum * Sum) / N
            ) /
            (N - 1.0);

        StdDev =
            std::sqrt(
                std::max(0.0, Variance)
            );
    }

    return
    {
        .Mean = Mean,
        .StdDev = StdDev
    };
}

bool RecvAll(int SocketFD, void* Data, size_t Size)
{
    uint8_t* Ptr = static_cast<uint8_t*>(Data);

    while(Size > 0)
    {
        ssize_t BytesReceived =
            recv(SocketFD, Ptr, Size, 0);

        if(BytesReceived < 0)
        {
            if(errno == EINTR)
                continue;

            return false;
        }

        if(BytesReceived == 0)
            return false;

        Ptr  += BytesReceived;
        Size -= static_cast<size_t>(BytesReceived);
    }

    return true;
}

bool SendAll(int SocketFD, const void* Data, size_t Size)
{
    const uint8_t* Ptr = static_cast<const uint8_t*>(Data);

    while(Size > 0)
    {
        const ssize_t BytesSent = send(SocketFD, Ptr, Size, 0);

        if(BytesSent < 0)
        {
            if(errno == EINTR)
                continue;

            return false;
        }

        if(BytesSent == 0)
            return false;

        Ptr += static_cast<size_t>(BytesSent);
        Size -= static_cast<size_t>(BytesSent);
    }

    return true;
}

uint64_t MonotonicTimeNs()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool SynchronizeClocks(int SocketFD, double& PiMinusMacOffsetNs,
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
        const uint64_t RoundTripTimeNs =
            TotalTimeNs >= PiProcessingTimeNs ?
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

int CreateServerSocket(uint16_t Port)
{
    int ServerFD =
        socket(AF_INET, SOCK_STREAM, 0);

    if(ServerFD < 0)
    {
        perror("socket");
        return -1;
    }

    int Enable = 1;

    setsockopt(
        ServerFD,
        SOL_SOCKET,
        SO_REUSEADDR,
        &Enable,
        sizeof(Enable)
    );

    sockaddr_in Address{};

    Address.sin_family = AF_INET;
    Address.sin_addr.s_addr = INADDR_ANY;
    Address.sin_port = htons(Port);

    if(bind(
        ServerFD, reinterpret_cast<sockaddr*>(&Address),
        sizeof(Address)) < 0)
    {
        perror("bind");
        close(ServerFD);
        return -1;
    }

    if(listen(ServerFD, 1) < 0)
    {
        perror("listen");
        close(ServerFD);
        return -1;
    }

    return ServerFD;
}

bool StartPiProgram()
{
    const char* Command =
        "ssh pantopilot@pantopilotstereoraspberry.local"
        "'~/PantoPI/ImageIO'";

    return std::system(Command) == 0;
}

int main()
{
    constexpr uint16_t PORT = 50555;
    constexpr int NUM_FRAMES = 600;

    int ServerFD = CreateServerSocket(PORT);

    if(ServerFD < 0)
        return 1;

    std::cout
        << "Listening on port "
        << PORT
        << '\n';

    const int SSHResult =
        std::system(
            "ssh -T pantopilot@pantopilotstereoraspberry.local "
            "'~/PantoPI/ImageIO 192.168.0.95' &"
        );

    if(SSHResult != 0)
    {
        std::cerr << "Failed to start Pi program\n";
        close(ServerFD);
        return 1;
    }

    std::cout << "Waiting for Pi connection...\n";

    int PiFD =
        accept(ServerFD, nullptr, nullptr);

    if(PiFD < 0)
    {
        perror("accept");
        close(ServerFD);
        return 1;
    }

    std::cout << "Pi connected\n";

    double PiMinusMacOffsetNs = 0.0;
    double ClockSyncRoundTripTimeMs = 0.0;
    if(!SynchronizeClocks(
                PiFD,
                PiMinusMacOffsetNs,
                ClockSyncRoundTripTimeMs))
    {
        std::cerr << "Clock synchronization failed\n";
        close(PiFD);
        close(ServerFD);
        return 1;
    }

    std::cout
        << "Clock synchronized; best RTT = "
        << ClockSyncRoundTripTimeMs
        << " ms\n";

    int NumFrames = 0;

    std::vector<double> ArrivalIntervalsMs;
    std::vector<double> CameraIntervalsMs;
    std::vector<double> CaptureToMatLatencyMs;

    ArrivalIntervalsMs.reserve(NUM_FRAMES - 1);
    CameraIntervalsMs.reserve(NUM_FRAMES - 1);
    CaptureToMatLatencyMs.reserve(NUM_FRAMES);

    std::chrono::steady_clock::time_point PreviousArrivalTime;
    uint64_t PreviousCameraTimestampNs = 0;


    while(NumFrames < NUM_FRAMES)
    {
        typeFrameHeader Header;

        if(!RecvAll(
            PiFD,
            &Header,
            sizeof(Header)))
        {
            std::cerr << "Connection closed\n";
            break;
        }

        cv::Mat Image(
            Header.Height,
            Header.Width,
            CV_8UC1
        );

        if(!RecvAll(
            PiFD,
            Image.data,
            Header.PayloadSize))
        {
            std::cerr << "Connection closed\n";
            break;
        }

        const auto CurrentArrivalTime = std::chrono::steady_clock::now();
        const uint64_t CurrentArrivalTimeNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                CurrentArrivalTime.time_since_epoch()).count());

        const double LatencyMs =
            (static_cast<double>(CurrentArrivalTimeNs) -
             static_cast<double>(Header.TimestampNs) +
             PiMinusMacOffsetNs) / 1e6;

        if(std::isfinite(LatencyMs))
        {
            CaptureToMatLatencyMs.push_back(LatencyMs);
        }

        if(NumFrames > 0)
        {
            const double ArrivalIntervalMs =
                std::chrono::duration<double, std::milli>(
                        CurrentArrivalTime - PreviousArrivalTime
                        ).count();

            ArrivalIntervalsMs.push_back(
                    ArrivalIntervalMs
                    );

            const double CameraIntervalMs =
                static_cast<double>(
                        Header.TimestampNs -
                        PreviousCameraTimestampNs
                        ) / 1e6;

            CameraIntervalsMs.push_back(
                    CameraIntervalMs
                    );
        }

        PreviousArrivalTime =
            CurrentArrivalTime;

        PreviousCameraTimestampNs =
            Header.TimestampNs;

        NumFrames++;

        cv::imshow("Pi Camera", Image);
        cv::waitKey(1);
    }


    close(PiFD);
    close(ServerFD);

    const typeStats ArrivalStats = CalculateStats( ArrivalIntervalsMs);
    const typeStats CameraStats = CalculateStats( CameraIntervalsMs);
    const typeStats LatencyStats = CalculateStats(CaptureToMatLatencyMs);

    constexpr double TARGET_FPS = 20.0;
    constexpr double TARGET_INTERVAL_MS = 1000.0 / TARGET_FPS;

    const double ArrivalFPS = 1000.0 / ArrivalStats.Mean;

    const double CameraFPS = 1000.0 / CameraStats.Mean;

    std::cout
        << "\n========================================\n"
        << " Frame Timing Results\n"
        << "========================================\n"
        << " Frames received:          "
        << NumFrames
        << '\n'
        << '\n'
        << " Target FPS:               "
        << TARGET_FPS
        << " Hz\n"
        << " Target interval:          "
        << TARGET_INTERVAL_MS
        << " ms\n"
        << '\n'
        << " CAMERA TIMESTAMPS\n"
        << " Mean interval:            "
        << CameraStats.Mean
        << " ms\n"
        << " Std. dev.:                "
        << CameraStats.StdDev
        << " ms\n"
        << " Equivalent FPS:           "
        << CameraFPS
        << " Hz\n"
        << '\n'
        << " MAC ARRIVAL TIMING\n"
        << " Mean interval:            "
        << ArrivalStats.Mean
        << " ms\n"
        << " Std. dev.:                "
        << ArrivalStats.StdDev
        << " ms\n"
        << " Equivalent FPS:           "
        << ArrivalFPS
        << " Hz\n"
        << '\n'
        << " CAPTURE TO CV::MAT LATENCY\n"
        << " Clock sync best RTT:      "
        << ClockSyncRoundTripTimeMs
        << " ms\n"
        << " Mean latency:             "
        << LatencyStats.Mean
        << " ms\n"
        << " Std. dev.:                "
        << LatencyStats.StdDev
        << " ms\n"
        << " Samples:                  "
        << CaptureToMatLatencyMs.size()
        << '\n'
        << "========================================\n";
    return 0;
}
