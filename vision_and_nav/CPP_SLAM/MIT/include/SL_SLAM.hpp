#ifndef __SL__SLAM_HPP_
#define __SL__SLAM_HPP_
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <utility>
#include <stdio.h>
#include "VW_Views.hpp"
#include "PT_Points.hpp"
#include "OB_Observations.hpp"
#include "FR_Frames.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "LG_Logging.hpp"
#include "OP_BA.hpp"
#include "VIZ_Visualization.hpp"
#include "PANTO_Utils.hpp"

struct SLAM
{
};

void SL_InitSlam();
void SL_SlamLoop(i32 num_loops);

#endif //__SL__SLAM_HPP_
