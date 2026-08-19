#ifndef __FR_FRAMES_HPP
#define __FR_FRAMES_HPP
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "LG_Logging.hpp"
#include "CArenaAlloc.h"
#include "Config.hpp"

typedef struct
{
    cv::Mat Frame;
    fp64 TimeStamp;
}typePantoFrame;

int FR_InitFrameGetter();
typePantoFrame FR_GetFrame(void);

#endif //__FR_FRAMES_HPP
