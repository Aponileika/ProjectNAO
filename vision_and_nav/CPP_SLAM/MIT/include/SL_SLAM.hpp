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
#include "FEAT_Features.hpp"

struct SLAM
{
    struct ViewSet* Tview;
    struct PointSet* Tpoints;
    struct ObservationSet* Tobs;

    //Current set of cameras (views) being considered for pnp
    //Note that the second view needs to be determined,
    //when it is SLAM can continue.
    std::pair<cv::Mat, cv::Mat> frame_pair;
};

void SL_InitSlam();
void SL_SlamLoop(i32 num_loops);

#endif //__SL__SLAM_HPP_
