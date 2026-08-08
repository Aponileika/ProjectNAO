#ifndef __SL__SLAM_HPP_
#define __SL__SLAM_HPP_
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <utility>
#include <stdio.h>
#include "LG_Logging.hpp"
#include "OP_BA.hpp"
#include "VIZ_Visualization.hpp"
#include "PANTO_Utils.hpp"
#include "MAP_Mapping.hpp"
#include "KEY_Keyframe.hpp"
#include "PT_PantoImagePoint.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"

struct SLAM
{
    typeMap GlobalMap;
    typeMap LocalMap;
};

void SL_InitSlam();
void SL_SlamLoop(i32 num_loops);

#endif //__SL__SLAM_HPP_
