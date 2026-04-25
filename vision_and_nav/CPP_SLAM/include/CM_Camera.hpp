#ifndef __CM_CAMERA_HPP_
#define __CM_CAMERA_HPP_
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>
#include "CArenaAlloc.h"
#include "LG_Logging.hpp"

struct CameraIntrinsics
{
    cv::Matx33d K;
    cv::Vec<fp64, 5> distcoeffs;
};

struct Param
{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
};

struct Camera
{
    //reference to global params
    struct CameraIntrinsics* intrinsics;
    cv::Mat R;
    cv::Mat t;
    struct Param* p;
};

extern struct CameraIntrinsics* ci;

void CM_SetIntrinsics(std::string path);
struct CameraIntrinsics* CM_GetIntrinsics();
struct Camera CM_CreateCam(cv::Mat R, cv::Mat t);
void CM_SetParametrization(struct Camera* cam);
void CM_SetRtfromParam(struct Camera* cam);

#endif //__CM_CAMERA_HPP_
