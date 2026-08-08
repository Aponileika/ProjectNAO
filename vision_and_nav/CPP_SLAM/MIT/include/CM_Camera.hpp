#ifndef __CM_CAMERA_HPP_
#define __CM_CAMERA_HPP_
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <Eigen/Geometry>
#include "CArenaAlloc.h"
#include "LG_Logging.hpp"

struct CameraIntrinsics
{
    Eigen::Matrix3d K;
        
    fp64 k1{};
    fp64 k2{};
    fp64 p1{};
    fp64 p2{};
    fp64 k3{};
};

typedef struct 
{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
}Param;

typedef struct 
{
    //reference to global params
    struct CameraIntrinsics* Intrinsics;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    Param Parameters;
    std::string image_name;
}Camera;


void CM_SetIntrinsics();
struct CameraIntrinsics* CM_GetIntrinsics();
Camera CM_CreateCam(cv::Mat R, cv::Mat t, i32 idx);
void CM_SetParametrization(Camera* cam);
void CM_SetRtfromParam(Camera* cam);

#endif //__CM_CAMERA_HPP_
