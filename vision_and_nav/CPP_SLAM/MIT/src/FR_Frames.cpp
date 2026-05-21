#include "../include/FR_Frames.hpp"

namespace {
    struct capture
    {
        cv::VideoCapture cap;
        bool isInit;
    };
    struct capture cap;
}

int FR_InitFrameGetter()
{
    if(!cap.cap.open(0))
    {
        std::cerr << "Failed to open camera\n";
        return 1;
    }
    cap.isInit = true;
    return 0;
}

cv::Mat FR_GetFrame(int idx)
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
    if(idx >= 0)
    {
        std::string path = "./colmap/images/frame" + std::to_string(idx) + ".png";
        LG_Log("[FR_GetFrame] writing frame file %s\n", path.c_str());
        bool ret = cv::imwrite(path, frame);
        if(!ret)
        {
            perror("Failed to write image in getframe");
            return {};
        }
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    return gray;
}
