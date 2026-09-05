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
#include <cstring>
#include <vector>
#include <chrono>
#include <cmath>
#include <iostream>
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

        if(BytesReceived <= 0)
            return false;

        Ptr  += BytesReceived;
        Size -= static_cast<size_t>(BytesReceived);
    }

    return true;
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

    int NumFrames = 0;

    std::vector<double> ArrivalIntervalsMs;
    std::vector<double> CameraIntervalsMs;

    ArrivalIntervalsMs.reserve(NUM_FRAMES - 1);
    CameraIntervalsMs.reserve(NUM_FRAMES - 1);

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
        << "========================================\n";
    return 0;
}
