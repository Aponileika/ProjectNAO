#ifndef __FR_FRAMES_HPP
#define __FR_FRAMES_HPP
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "LG_Logging.hpp"
#include "CArenaAlloc.h"
#include "Config.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include "EP_CorrespondingPoints.hpp"

typedef struct
{
    cv::Mat Frame;
    fp64 TimeStamp;
    std::string Path;
    DescRet Descriptors;
}typePantoFrame;

int FR_InitFrameGetter();
typePantoFrame FR_GetFrame(void);
fp64 FR_PeekNextFrameTimeStamp(void);
fp64 FR_SkipNextFrame(void);

#endif //__FR_FRAMES_HPP
