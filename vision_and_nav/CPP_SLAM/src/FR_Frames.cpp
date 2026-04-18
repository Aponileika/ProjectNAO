#include "FR_Frames.hpp"

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

cv::Mat FR_GetFrame()
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
    return frame;
}
