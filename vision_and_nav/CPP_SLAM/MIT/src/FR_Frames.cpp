#include "../include/FR_Frames.hpp"
#include <chrono>

namespace 
{
    struct capture
    {
        cv::VideoCapture cap;
        bool isInit;
        i32 FrameIndex;
    };
    struct capture cap;
}

namespace 
{
    struct dataset_read 
    {
        std::string path;
        i32 curr_frame;
        i32 frame_idx;
    };
    struct dataset_read reader;
}

using PantoClock = std::chrono::steady_clock;

namespace 
{
    const PantoClock::time_point StartTime = PantoClock::now();
}

cv::Mat __FR_GetFrameDataSet();
cv::Mat __FR_GetFrameWebCam(void);

int FR_InitFrameGetter()
{
    if(PANTO_USE_DATASET == true)
    {
        reader.path = std::string(PANTO_DATASET_BASE_PATH) +
        std::string(panto_dataset_path) + "/" +
        std::string(panto_sequence_path) + "/";
        reader.curr_frame = 1;
        reader.frame_idx = 0;
        LG_Log(LogSeverity::DBG, "[FR_InitFrameGetter] Dataset path: %s\n", reader.path.c_str());
    }
    else
    {
        if(!cap.cap.open(0))
        {
            std::cerr << "Failed to open camera\n";
            return 1;
        }
        cap.isInit = true;
        cap.FrameIndex = 0;

    }
    return 0;
}

typePantoFrame FR_GetFrame(void)
{
    typePantoFrame PantoFrame{};
    if(PANTO_USE_DATASET == true)
    {
        cv::Mat Frame = __FR_GetFrameDataSet();
        const PantoClock::time_point CurrentTime = PantoClock::now();
        const fp64 TimeStamp = std::chrono::duration(CurrentTime - StartTime).count();
        PantoFrame.Frame = Frame; 
        PantoFrame.TimeStamp = TimeStamp;
    }
    else
    {
        cv::Mat Frame = __FR_GetFrameWebCam();
        const PantoClock::time_point CurrentTime = PantoClock::now();
        const fp64 TimeStamp = std::chrono::duration(CurrentTime - StartTime).count();
        PantoFrame.Frame = Frame; 
        PantoFrame.TimeStamp = TimeStamp;
    }
    return PantoFrame;
}

cv::Mat __FR_GetFrameWebCam(void)
{
    if(!cap.isInit)
    {
        std::cerr << "Camera not initialized when calling FR_GetFrame\n";
    }

    cv::Mat frame;
    //get one frame
    if(!cap.cap.read(frame) || frame.empty())
    {
        std::cerr << "FR_GetFrame failed to get a frame\n";
        return {};
    }
    std::string path = "./colmap/images/frame" + std::to_string(cap.FrameIndex) + ".png";
    LG_Log(LogSeverity::DBG, "[FR_GetFrame] writing frame file %s\n", path.c_str());
    bool ret = cv::imwrite(path, frame);
    if(!ret)
    {
        perror("Failed to write image in getframe");
        return {};
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

cv::Mat __FR_GetFrameDataSet()
{
    std::string curr_frame = reader.path + "frame" + std::to_string(reader.curr_frame) + ".png";
    LG_Log(LogSeverity::DBG, "[__FR_GetFrameDataSet] Getting frame %s\n", curr_frame.c_str());
    cv::Mat frame = cv::imread(curr_frame, cv::IMREAD_COLOR);
    if(frame.empty())
    {
        LG_Log(LogSeverity::DBG, "[__FR_GetFrameDataSet] cv::imread returned empty frame\n");
        return {};
    }

    std::string path_write = "./colmap/images/frame" + std::to_string(reader.frame_idx) + ".png";
    cv::imwrite(path_write, frame);
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    reader.curr_frame++;
    reader.frame_idx++;
    return gray;
}
