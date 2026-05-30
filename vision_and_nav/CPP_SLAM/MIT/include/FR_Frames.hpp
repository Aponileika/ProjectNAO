#ifndef __FR_FRAMES_HPP
#define __FR_FRAMES_HPP
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "LG_Logging.hpp"
#include "CArenaAlloc.h"

#define USE_DATASET 1
// #define DATSET_PATH "/Users/Jonathan/Programmering/FIA/ProjectNAO/vision_and_nav/CPP_SLAM/datasets/tum/rgbd_dataset_freiburg1_xyz/"
// #define SEQUENCE "rgb_ordered/"
#define DATSET_PATH "./datasets/tsbb33-datasets/"
#define SEQUENCE "turtle/"

int FR_InitFrameGetter();
cv::Mat FR_GetFrame(int idx);

#endif //__FR_FRAMES_HPP
