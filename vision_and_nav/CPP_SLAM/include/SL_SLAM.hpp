#ifndef __SL__SLAM_HPP_
#define __SL__SLAM_HPP_
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <utility>
#include "VW_Views.hpp"
#include "PT_Points.hpp"
#include "OB_Observations.hpp"
#include "FR_Frames.hpp"
#include "EP_CorrespondingPoints.hpp"

#define RANSACMETHOD cv::LMEDS
#define PROBECORRECT 0.95f
#define RANSACEPIXELT 2.0f
#define RANSACMAXITERS 1000

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
void SL_SlamLoop();

#endif //__SL__SLAM_HPP_
