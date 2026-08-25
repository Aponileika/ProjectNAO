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
        i32 CurrentFrameIndex;
        i32 frame_idx;
    };
    struct dataset_read reader;
}

namespace 
{
    const PantoClock::time_point StartTime = PantoClock::now();
}

typePantoFrame __FR_GetFrameDataSet();
cv::Mat __FR_GetFrameWebCam(void);

int FR_InitFrameGetter()
{
    if(PANTO_USE_DATASET == true)
    {
        reader.path = std::string(PANTO_DATASET_BASE_PATH) +
        std::string(panto_dataset_path) + "/" +
        std::string(panto_sequence_path) + "/";
        reader.CurrentFrameIndex = 1;
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
        typePantoFrame Frame = __FR_GetFrameDataSet();
        PantoFrame = Frame;
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

typePantoFrame __FR_GetFrameDataSet()
{
    std::string FramePath = reader.path + "frame" + std::to_string(reader.CurrentFrameIndex) + ".png";
    LG_Log(LogSeverity::DBG, "[__FR_GetFrameDataSet] Getting frame %s\n", FramePath.c_str());
    cv::Mat Frame = cv::imread(FramePath, cv::IMREAD_COLOR);
    const PantoClock::time_point CurrentTime = PantoClock::now();
    const fp64 TimeStamp = std::chrono::duration<fp64>(CurrentTime - StartTime).count();
    if(Frame.empty())
    {
        LG_Log(LogSeverity::DBG, "[__FR_GetFrameDataSet] cv::imread returned empty frame\n");
        return {};
    }

    std::string WritePath= "./colmap/images/frame" + std::to_string(reader.frame_idx) + ".png";
    cv::imwrite(WritePath, Frame);
    cv::Mat Gray;
    cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
    reader.CurrentFrameIndex++;
    reader.frame_idx++;
    return 
    {
        .Frame = Gray,
        .TimeStamp = TimeStamp,
        .Path = WritePath
    };
}
